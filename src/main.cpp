//Settings
//#define OTA 1
// #define DEBUG 1
// #define NETWORK 1

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

//Positions
#define MAX_ENCODER 16384

#define RIGHT_SETPOINT 40000
#define MID_SETPOINT 0
#define LEFT_SETPOINT -40000

#define VEL_MOVE_THRESHOLD 140
#define STOP_THRESHOLD 10
#define START_THRESHOLD 23

#define STEPPIN 33
#define DIRPIN 32
#define EPIN 27
#define STEPPER_SPEED 1200
AccelStepper stepper(AccelStepper::DRIVER, STEPPIN, DIRPIN);

#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCLK 18
#define VSPI_SS 5
AS5048A encoder(VSPI_SS, VSPI_MISO, VSPI_MOSI, VSPI_SCLK);

uint16_t previousEncoder;
int32_t absStepperPos = 0;
int32_t absStepperPosStable = 0;
byte positionState = 0; //2 = right, 1 = left, 0 = mid

bool targetReached = false;
double previousSetpoint;

double Setpoint = MID_SETPOINT;
double Input, Output;
double Pk = .8;
double Ik = 0;
double Dk = 0;
PID PID(&Input, &Output, &Setpoint, Pk, Ik, Dk, DIRECT);

//Timing
uint32_t encoderDelay = 30;
#define SLEEP_TIMEOUT 60000
#define COSTING_DELAY 50
bool timeout = false;
unsigned long previousMillis = 0;
bool flag = true;

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
        {
            Setpoint = RIGHT_SETPOINT;
            positionState = 2;
        }
        else if (msg == "mid")
        {
            Setpoint = MID_SETPOINT;
            positionState = 0;
        }
        else if (msg == "left")
        {
            Setpoint = LEFT_SETPOINT;
            positionState = 1;
        }
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

void runStepper(void *parameter)
{
    while (true)
    {
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

        //Change target if pushed
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

        //Enable movement and reset encoder
        if ((Setpoint != previousSetpoint && targetReached) || (abs(encoderVelocity) >= START_THRESHOLD && targetReached))
        {
            targetReached = false;
            timeout = false;
            encoderDelay = 30;

            absStepperPos = absStepperPosStable + encoderVelocity;
            absStepperPosStable += encoderVelocity;

            stepper.enableOutputs();
        }

        //Compute PID
        Input = absStepperPos;
        PID.Compute();
        stepper.setSpeed(Output);

        //Disable movement and save encoder position
        if (abs(Output) <= STOP_THRESHOLD && abs(encoderVelocity) <= START_THRESHOLD && !targetReached)
        {
            absStepperPosStable = absStepperPos;
            previousSetpoint = Setpoint;
            targetReached = true;
            previousMillis = currentMillis;
        }

        //Timeout to turn off stepper and save position
        if (targetReached && (currentMillis - previousMillis) >= SLEEP_TIMEOUT && !timeout)
        {
            stepper.disableOutputs();
            encoderDelay = COSTING_DELAY;
            timeout = true;
            preferences.putInt("absStepperPos", absStepperPos);
            preferences.putInt("positionState", positionState);
#ifdef DEBUG
            Serial.println("Timeout");
#endif
        }

#ifdef DEBUG

        Serial.print(encoderVelocity);
        Serial.print(",   ");
        Serial.print(absStepperPos);
        Serial.print(",   ");
        Serial.print(absStepperPosStable);
        Serial.print(",   ");
        Serial.print(Setpoint);
        Serial.print(",   ");
        Serial.print(Output);
        Serial.print(",   ");
        Serial.print(targetReached);
        Serial.print(",   ");
        Serial.println(currentMillis - previousMillis);

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
        vTaskDelay(10);
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
        3000,                     // Stack size (bytes)
        NULL,                     // Parameter to pass
        2,                        // Task priority
        NULL,                     // Task handle
        0                         // Core you want to run the task on (0 or 1)
    );

    xTaskCreatePinnedToCore(
        network,                  // Function that should be called
        "Maintain wifi and MQTT", // Name of the task (for debugging)
        3000,                     // Stack size (bytes)
        NULL,                     // Parameter to pass
        2,                        // Task priority
        NULL,                     // Task handle
        1                         // Core you want to run the task on (0 or 1)
    );   
#endif

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
    previousEncoder = encoder.getRawRotation();

#ifdef DEBUG
    Serial.print("Last stepper pos");
    Serial.println(absStepperPos);
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
