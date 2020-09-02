//Settings
//#define OTA 1
#define DEBUG 1

#include <Arduino.h>
#include "AS5048A.h"
#include <AccelStepper.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include "keys.h"

#ifdef OTA
#include <ArduinoOTA.h>
#endif

const char *mqtt_server = MQTT_SERVER;

WiFiClient espClient;
PubSubClient client(espClient);

#define STEPPIN 33
#define DIRPIN 32
#define EPIN 27
#define STEPPER_SPEED 1000

#define MAX_STEPS 3000
#define MAX_ENCODER 17000

//Positions
#define RIGHT_SETPOINT 1000
#define MID_SETPOINT 0
#define LEFT_SETPOINT -1000

AccelStepper stepper(AccelStepper::DRIVER, STEPPIN, DIRPIN);

#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SCLK 18
#define VSPI_SS 5

AS5048A encoder(VSPI_SS, VSPI_MISO, VSPI_MOSI, VSPI_SCLK, false);

uint16_t previousEncoder;
uint8_t positionState; //2 = right, 1 = left, 0 = mid
int16_t Setpoint;

//Smoothing
const int numReadings = 10;
int readings[numReadings]; // the readings from the analog input
int readIndex = 0;         // the index of the current reading
int total = 0;             // the running total
int average = 0;           // the average

//Timing
unsigned long aliveMsg = 0;
unsigned long eventLoop = 0;

void setup_wifi()
{
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_NAME);

    WiFi.begin(WIFI_NAME, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    randomSeed(micros());

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

long parse_long(byte *payload, unsigned int &length)
{
    char buffer[128];

    memcpy(buffer, payload, length);
    buffer[length] = '\0';

    // Convert it to integer
    char *end = nullptr;
    long value = strtol(buffer, &end, 10);

    // Check for conversion errors
    if (end == buffer || errno == ERANGE)
        return 0; // Conversion error occurred
    else
        return value;
}

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");

    if (strcmp(topic, "set"))
    {
        char *state = (char *)payload;

        if (strcmp(state, "right"))
            Setpoint = LEFT_SETPOINT;
        if (strcmp(state, "mid"))
            Setpoint = MID_SETPOINT;
        if (strcmp(state, "left"))
            Setpoint = RIGHT_SETPOINT;
    }

    if (strcmp(topic, "set_position"))
    {
        long value = parse_long(payload, length);
        Serial.print("Requested Position:");
        Serial.println(value);
    }

    //Serial.print(msg);
    Serial.println();
}

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        stepper.disableOutputs();
        // Create a random client ID
        String clientId = "Smart_Table-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientId.c_str()))
        {
            Serial.println("connected");
            // Once connected, publish an announcement...
            client.publish("home-assistant/smart_table/availability", "online");
            // ... and resubscribe
            client.subscribe("home-assistant/smart_table/set");
            client.subscribe("home-assistant/smart_table/set_position");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void smooth(uint16_t &inputVal)
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
    // send it to the computer as ASCII digits
    inputVal = average;
}

void setup()
{
    Serial.begin(115200);

    //setup_wifi();
    //client.setServer(mqtt_server, 1883);
    //client.setCallback(callback);

#ifdef OTA
    ArduinoOTA.begin();
#endif

    // stepper.setMaxSpeed(200000);
    // stepper.setAcceleration(10000);
    //stepper.setEnablePin(EPIN);

    //PID.SetMode(AUTOMATIC);
    //PID.SetTunings(Kp, Ki, Kd);

    encoder.begin();
}

void loop()
{
    //ArduinoOTA.handle();

    // if (!client.connected())
    // {
    //     reconnect();
    // }
    // client.loop();

    unsigned long now = millis();

    // if (now - aliveMsg > 60000) //Publish alive message to home assistant every minute
    // {
    //     aliveMsg = now;
    //     client.publish("home-assistant/smart_table/availability", "online");
    // }

    if (Serial.available() > 0)
    {
        Setpoint = Serial.parseInt();
    }

    if (now - eventLoop > 50) //Event loop every 100 mills
    {
        eventLoop = now;

        uint16_t rawEncoder = encoder.getRawRotation();

#ifdef DEBUG
        Serial.print(rawEncoder);
        Serial.print(",   ");
#endif

        smooth(rawEncoder);

        int16_t velocity = rawEncoder - previousEncoder;

        previousEncoder = rawEncoder;

#ifdef DEBUG
        Serial.print(rawEncoder);
        Serial.print(",   ");
        Serial.println(velocity);
#endif

        if (velocity >= 1000)
        {
#ifdef DEBUG
            Serial.println("Change position right!");
#endif
        }
        else if (velocity <= -1000)
        {
#ifdef DEBUG
            Serial.println("Change position left!");
#endif
        }
        else
        {

            //TODO: moveback to setpoint
        }

        //TODO: if no movement for some time turn off motor

        //stepper.moveTo(Output);
        //stepper.setSpeed(STEPPER_SPEED);

        // if (stepper.distanceToGo() == 0) {
        //     stepper.disableOutputs();
        // } else {
        //     stepper.enableOutputs();
        // }
    }

    //stepper.runSpeedToPosition();
}