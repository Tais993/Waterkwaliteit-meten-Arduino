#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Arduino.h"
#include <OneWire.h>
#include <DallasTemperature.h>


const int GPSReceivePin = 4, GPSTransmitPin = 2;

const uint32_t GPSBaud = 9600; //Default baud of NEO-6M is 9600


TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPSReceivePin, GPSTransmitPin);

void setup() {
    Serial.begin(9600);
    gpsSerial.begin(GPSBaud);
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

void loop() {
    checkLocation();
}