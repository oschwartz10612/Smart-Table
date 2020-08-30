#include <Arduino.h>
#include "AS5048A.h"
#include <AccelStepper.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <PID_v1.h>

#include "keys.h"

const char *mqtt_server = MQTT_SERVER;

#define STEPPIN 33
#define DIRPIN 32
#define EPIN 27
#define ENCODERPIN 5 //IO5
#define STEPPER_SPEED 1000

#define MAX_STEPS 1000
#define MAX_ENCODER 8192

//Positions
#define RIGHT_SETPOINT 1000
#define MID_SETPOINT 0
#define LEFT_SETPOINT -1000

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long aliveMsg = 0;
unsigned long eventLoop = 0;

AccelStepper stepper(AccelStepper::DRIVER, STEPPIN, DIRPIN);

double Setpoint, Input, Output;

double Kp = 1, Ki = 0.05, Kd = 0.25;

PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

AS5048A encoder(ENCODERPIN, false);

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
        char *state = (char*)payload;

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

void setup()
{
    Serial.begin(115200);

    //setup_wifi();
    //client.setServer(mqtt_server, 1883);
    //client.setCallback(callback);

    //ArduinoOTA.begin();

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

    if (now - eventLoop > 500) //Event loop every 100 mills
    {
        eventLoop = now;

        //int16_t rawEncoder = encoder.getRotation();

        //Input = map(rawEncoder, -MAX_ENCODER, MAX_ENCODER, -MAX_STEPS, MAX_STEPS);

        uint16_t val = encoder.getRawRotation();
        Serial.print("Got rotation of: 0x");
        Serial.println(val, HEX);
        Serial.print("State: ");
        encoder.printState();
        Serial.print("Errors: ");
        Serial.println(encoder.getErrors());

        //Serial.print("Setpoint:");
        //Serial.println(Setpoint);

        //PID.Compute();

        //Serial.print("Output:");
        //Serial.println(Output);

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