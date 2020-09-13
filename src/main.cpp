//Settings
//#define OTA 1
//#define DEBUG 1
#define NETWORK 1

#include <Arduino.h>
#include "AS5048A.h"
#include <AccelStepper.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <PID_v1.h>
#include "keys.h"

#ifdef OTA
#include <ArduinoOTA.h>
#endif

Preferences preferences;

const char *mqtt_server = MQTT_SERVER;

WiFiClient espClient;
PubSubClient client(espClient);

#define STEPPIN 33
#define DIRPIN 32
#define EPIN 27
#define STEPPER_SPEED 1500

#define MAX_STEPS 3000
#define MAX_ENCODER 16384
#define STEPS_PER_REV 200

//Positions
#define RIGHT_SETPOINT 8000
#define MID_SETPOINT 0
#define LEFT_SETPOINT -8000

#define VEL_MOVE_THRESHOLD 1000
#define STOP_THRESHOLD 10
#define START_THRESHOLD 23

AccelStepper stepper(AccelStepper::DRIVER, STEPPIN, DIRPIN);

#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCLK 18
#define VSPI_SS 5

AS5048A encoder(VSPI_SS, VSPI_MISO, VSPI_MOSI, VSPI_SCLK, false);

uint16_t previousEncoder;
int32_t absStepperPos = 0;
int32_t absStepperPosStable = 0;
byte positionState = 0; //2 = right, 1 = left, 0 = mid

bool targetReached = false;
double previousSetpoint;

double Setpoint = MID_SETPOINT;
double Input, Output;
double Pk = 2;
double Ik = 0;
double Dk = .3;

PID PID(&Input, &Output, &Setpoint, Pk, Ik, Dk, DIRECT);

//Smoothing
const uint8_t numReadings = 10;
int32_t readings[numReadings]; // the readings from the analog input
int32_t readIndex = 0;         // the index of the current reading
int32_t total = 0;             // the running total
int32_t average = 0;           // the average

//Timing
uint32_t encoderDelay = 50;
#define SLEEP_TIMEOUT 10000
bool timeout = false;
unsigned long previousMillis = 0;

void setup_wifi()
{
    stepper.disableOutputs();
    delay(10);
// We start by connecting to a WiFi network
#ifdef DEBUG
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_NAME);
#endif

    WiFi.begin(WIFI_NAME, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
#ifdef DEBUG
        Serial.print(".");
#endif
    }

    randomSeed(micros());
#ifdef DEBUG
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
#endif
}

void callback(char *topic, byte *payload, unsigned int length)
{

#ifdef DEBUG
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
#endif

    payload[length] = '\0';
    String msg = String((char *)payload);
#ifdef DEBUG
    Serial.println(msg);
#endif

    if (strcmp(topic, "home-assistant/smart_table/set") == 0)
    {

#ifdef DEBUG
        Serial.println("Changing position");
#endif

        if (msg == "right")
            Setpoint = RIGHT_SETPOINT;
        if (msg == "mid")
            Setpoint = MID_SETPOINT;
        if (msg == "left")
            Setpoint = LEFT_SETPOINT;
    }
}

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
#ifdef DEBUG
        Serial.print("Attempting MQTT connection...");
#endif
        // Create a random client ID
        String clientId = "Smart_Table-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientId.c_str()))
        {
#ifdef DEBUG
            Serial.println("connected");
#endif
            // Once connected, publish an announcement...
            client.publish("home-assistant/smart_table/availability", "online");
            // ... and resubscribe
            client.subscribe("home-assistant/smart_table/set");
            client.subscribe("home-assistant/smart_table/set_position");
        }
        else
        {
#ifdef DEBUG
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
#endif
            // Wait 5 seconds before retrying
            vTaskDelay(5000);
        }
    }
}

void smooth(int32_t &inputVal)
{
    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = inputVal;
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings)
    {
        // ...wrap around to the beginning:
        readIndex = 0;
    }

    // calculate the average:
    average = total / numReadings;
    inputVal = average;
}

void runStepper(void *parameter)
{
    while (true)
    {
#ifdef DEBUG
        if (Serial.available() > 0)
        {
            Setpoint = Serial.parseInt();
        }
#endif
        if (!targetReached)
        {
            stepper.runSpeed();
        }
        vTaskDelay(1);
    }
}

void readEncoder(void *parameter)
{
    while (true)
    {
        uint16_t rawEncoder = encoder.getRawRotation();
        unsigned long currentMillis = millis();

        int16_t encoderVelocity = rawEncoder - previousEncoder;
        if (encoderVelocity > (MAX_ENCODER / 2))
        {
            encoderVelocity = (rawEncoder - MAX_ENCODER) - previousEncoder;
        }
        else if (encoderVelocity < -(MAX_ENCODER / 2))
        {
            encoderVelocity = (rawEncoder + MAX_ENCODER) - previousEncoder;
        }

        absStepperPos += encoderVelocity;

        previousEncoder = rawEncoder;

        if (encoderVelocity >= VEL_MOVE_THRESHOLD && targetReached)
        {
#ifdef DEBUG
            Serial.println("Change position right!");
#endif

            if (positionState == 0)
            {
                positionState = 2;
                Setpoint = RIGHT_SETPOINT;
            }
            else
            {
                positionState = 0;
                Setpoint = MID_SETPOINT;
            }
        }
        else if (encoderVelocity <= -VEL_MOVE_THRESHOLD && targetReached)
        {
#ifdef DEBUG
            Serial.println("Change position left!");
#endif

            if (positionState == 0)
            {
                positionState = 1;
                Setpoint = LEFT_SETPOINT;
            }
            else
            {
                positionState = 0;
                Setpoint = MID_SETPOINT;
            }
        }
 

        smooth(absStepperPos);

        Input = absStepperPos;

        PID.Compute();

        stepper.setSpeed(Output);

        if (Setpoint != previousSetpoint && targetReached)
        {
            targetReached = false;
            absStepperPos = absStepperPosStable + encoderVelocity;
        } else if (abs(encoderVelocity) >= START_THRESHOLD && targetReached)
        {
            targetReached = false;
            absStepperPos = absStepperPosStable + encoderVelocity;
        }

        if (abs(Output) <= STOP_THRESHOLD && !targetReached)
        {
            absStepperPosStable = absStepperPos;
            previousSetpoint = Setpoint;
            encoderDelay = 100;
            targetReached = true;
            preferences.putInt("absStepperPos", absStepperPos);
            preferences.putInt("positionState", positionState);
            previousMillis = currentMillis;
#ifdef DEBUG
            Serial.println("Stopped");
#endif
        }
        if (!targetReached && abs(Output) >= STOP_THRESHOLD)
        {
            encoderDelay = 50;
            timeout = false;
            stepper.enableOutputs();
#ifdef DEBUG
            Serial.println("Starting");
#endif
        }

        if (targetReached && (currentMillis - previousMillis) >= SLEEP_TIMEOUT && !timeout)
        {
            stepper.disableOutputs();
            encoderDelay = 1000;
            timeout = true;
#ifdef DEBUG
            Serial.println("Timeout");
#endif
        }

#ifdef DEBUG

        Serial.print(absStepperPos);
        Serial.print(",   ");
        Serial.print(absStepperPosStable);
        Serial.print(",   ");
        Serial.print(Setpoint);
        Serial.print(",   ");
        Serial.println(Output);
#endif

        vTaskDelay(encoderDelay);
    }
}

void network(void *parameter)
{
    while (true)
    {
        if (!client.connected())
        {
            reconnect();
        }
        client.loop();

#ifdef OTA
        ArduinoOTA.handle();
#endif
        vTaskDelay(50);
    }
}

void keepAlive(void *parameter)
{
    while (true) //Publish alive message to home assistant every minute
    {
        client.publish("home-assistant/smart_table/availability", "online");
        vTaskDelay(60000);
    }
}

void setup()
{
#ifdef DEBUG
    Serial.begin(115200);
#endif

    stepper.setMaxSpeed(200000);
    stepper.setAcceleration(1000);
    stepper.setEnablePin(EPIN);
    stepper.enableOutputs();

    encoder.begin();

    PID.SetMode(AUTOMATIC);
    PID.SetOutputLimits(-STEPPER_SPEED, STEPPER_SPEED);

    preferences.begin("table", false);

    positionState = preferences.getInt("positionState", 0);
    absStepperPos = preferences.getInt("absStepperPos", 0);
    absStepperPosStable = absStepperPos;

    for (uint16_t i = 0; i <= numReadings; i++)
    {
        smooth(absStepperPos);
        absStepperPos = absStepperPosStable;
    }

#ifdef DEBUG
    Serial.print("Last stepper pos");
    Serial.println(absStepperPos);
#endif

    previousEncoder = encoder.getRawRotation();

#ifdef NETWORK
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

#ifdef OTA
    ArduinoOTA.begin();
#endif

    xTaskCreatePinnedToCore(
        keepAlive,                // Function that should be called
        "Maintain wifi and MQTT", // Name of the task (for debugging)
        2000,                     // Stack size (bytes)
        NULL,                     // Parameter to pass
        2,                        // Task priority
        NULL,                     // Task handle
        0                         // Core you want to run the task on (0 or 1)
    );

    xTaskCreatePinnedToCore(
        network,                  // Function that should be called
        "Maintain wifi and MQTT", // Name of the task (for debugging)
        2000,                     // Stack size (bytes)
        NULL,                     // Parameter to pass
        2,                        // Task priority
        NULL,                     // Task handle
        1                         // Core you want to run the task on (0 or 1)
    );
#endif

    xTaskCreatePinnedToCore(
        runStepper,             // Function that should be called
        "Run Stepper at Speed", // Name of the task (for debugging)
        1000,                   // Stack size (bytes)
        NULL,                   // Parameter to pass
        1,                      // Task priority
        NULL,                   // Task handle
        0                       // Core you want to run the task on (0 or 1)
    );

    xTaskCreatePinnedToCore(
        readEncoder,                 // Function that should be called
        "Read encoder and calc PID", // Name of the task (for debugging)
        2000,                        // Stack size (bytes)
        NULL,                        // Parameter to pass
        1,                           // Task priority
        NULL,                        // Task handle
        1                            // Core you want to run the task on (0 or 1)
    );
}

void loop()
{
}