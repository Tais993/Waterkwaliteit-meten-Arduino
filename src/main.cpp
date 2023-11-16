#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Arduino.h"


const int GPSReceivePin = 4, GPSTransmitPin = 2, WaterLevelSensor = A5, POWER_PIN = 7;

const uint32_t GPSBaud = 9600; //Default baud of NEO-6M is 9600


TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPSReceivePin, GPSTransmitPin);

void setup() {
    Serial.begin(9600);
    gpsSerial.begin(GPSBaud);
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, LOW);
}

void checkLocation() {
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
    int sensorValue = analogRead(A0);
    float voltage = sensorValue * (5.0 / 1023.0);
    float temperature = voltage - 0.5;
    temperature = temperature / 0.01;

    return temperature - 10;
}

void loop() {
    checkLocation();
    checkWaterHeight();
    Serial.println(checkTemperature());
}