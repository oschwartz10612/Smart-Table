#include <Arduino.h>
#include "AS5048A.h"
#include <AccelStepper.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "keys.h"

const char *mqtt_server = MQTT_SERVER;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

#define STEPPIN 14
#define DIRPIN 4
#define EPIN 5

AccelStepper stepper(AccelStepper::DRIVER, STEPPIN, DIRPIN);

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

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");

    payload[length] = '\0';
    String msg = String((char *)payload);

    Serial.print(msg);
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
    ArduinoOTA.begin();

    stepper.setMaxSpeed(200000);
    stepper.setAcceleration(10000);
    stepper.setEnablePin(EPIN);
    stepper.disableOutputs();

    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

void loop()
{
    ArduinoOTA.handle();

    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    unsigned long now = millis();
    if (now - lastMsg > 60000)
    {
        lastMsg = now;
        client.publish("home-assistant/smart_table/availability", "online");
    }
}