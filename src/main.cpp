#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include <ArduinoHttpClient.h>
#include <WiFi.h>
#include "arduino_secrets.h"

const int GPSReceivePin = 4, GPSTransmitPin = 2;
const int WaterLevelSensor = A5, POWER_PIN = 7;
const int TemperatureSensor = A1;

const uint32_t GPSBaud = 9600; //Default baud of NEO-6M is 9600


char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
char serverAddress[] = SECRET_ADDRESS;  // server address
int port = SECRET_PORT;

WiFiClient wifi;
HttpClient client = HttpClient(wifi, serverAddress, port);
int status = WL_IDLE_STATUS;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPSReceivePin, GPSTransmitPin);

void connectWithWifi() {
    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to Network named: ");
        Serial.println(ssid);
        // print the network name (SSID);

        // Connect to WPA/WPA2 network:
        status = WiFi.begin(ssid, pass);
    }

    Serial.println("Connected!");
}

void setup() {
    Serial.begin(9600);
    gpsSerial.begin(GPSBaud);
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, LOW);
    connectWithWifi();
}

void checkLocation() {
    TinyGPSLocation &location =gps.location;
    
    if (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isValid()) {
                double latitude = gps.location.lat();
                double longitude = gps.location.lng();
                Serial.print("lat");
                Serial.println(latitude, 6);
                Serial.print("long");
                Serial.println(longitude, 6);
            } else {
                Serial.println("Location not yet available.");
            }
        }
    }
}

void checkWaterHeight() {
    int waterLevel = analogRead(WaterLevelSensor); // read the analog waterLevel from sensor
    digitalWrite(POWER_PIN, HIGH);
    delay(50);
    Serial.print("Water level: ");
    Serial.print(waterLevel);
    Serial.println("mm");
    digitalWrite(POWER_PIN, LOW);
}

float checkTemperature() {
    int sensorValue = analogRead(TemperatureSensor);
    float voltage = sensorValue * (5.0 / 1023.0);
    float temperature = voltage - 0.5;
    temperature = temperature / 0.01;

    return temperature - 10;
}

void sendRequest() {
    String contentType = "application/json";
    String postData = "{\n"
                      "    \"tabel\": \"parameters\",\n"
                      "    \"meetEenheid\": \"PH\",\n"
                      "    \"naam\": \"zuurgraad5\" \n"
                      "}";

    client.post("/api/waterkwaliteit", contentType, postData);

    // read the status code and body of the response
    int statusCode = client.responseStatusCode();
    String response = client.responseBody();

    if (statusCode != 200) {
        Serial.print("Status code: ");
        Serial.println(statusCode);
        Serial.print("Response: ");
        Serial.println(response);
    } else {
        Serial.println("Successfully sent info");
    }
}

void loop() {
    checkLocation();
    checkWaterHeight();
    Serial.print("Temperature: ");
    Serial.println(checkTemperature());
//    sendRequest();
}