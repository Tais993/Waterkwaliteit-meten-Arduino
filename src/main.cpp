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

class PostRequestInfo{
public:
    String name;
    String meetEenheid;

};


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

#include "Arduino.h"
/*
  # This sample code is used to test the pH meter V1.1.
  # Editor : YouYou
  # Date   : 2014.06.23
  # Ver    : 1.1
  # Product: analog pH meter
  # SKU    : SEN0161
*/
#define SensorPin A2            //pH meter Analog output to Arduino Analog Input 0
#define Offset 41.02740741      //deviation compensate
#define LED 8
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
#define uart Serial
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex = 0;
void setup(void)

{
    pinMode(LED, OUTPUT);
    uart.begin(9600);
    uart.println("pH meter experiment!");    //Test the uart monitor
}

double avergearray(int* arr, int number) {
    int i;
    int max, min;
    double avg;
    long amount = 0;
    if (number <= 0) {
        uart.println("Error number for the array to avraging!/n");
        return 0;
    }
    if (number < 5) { //less than 5, calculated directly statistics
        for (i = 0; i < number; i++) {
            amount += arr[i];
        }
        avg = amount / number;
        return avg;
    } else {
        if (arr[0] < arr[1]) {
            min = arr[0]; max = arr[1];
        }
        else {
            min = arr[1]; max = arr[0];
        }
        for (i = 2; i < number; i++) {
            if (arr[i] < min) {
                amount += min;      //arr<min
                min = arr[i];
            } else {
                if (arr[i] > max) {
                    amount += max;  //arr>max
                    max = arr[i];
                } else {
                    amount += arr[i]; //min<=arr<=max
                }
            }//if
        }//for
        avg = (double)amount / (number - 2);
    }//if
    return avg;
}

void checkPh() {
    static unsigned long samplingTime = millis();
    static unsigned long printTime = millis();
    static float pHValue, voltage;
    if (millis() - samplingTime > samplingInterval)
    {
        pHArray[pHArrayIndex++] = analogRead(SensorPin);
        if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
        voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
        pHValue = -19.18518519 * voltage + Offset;
        samplingTime = millis();
    }
    if (millis() - printTime > printInterval)  //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
    {
        uart.print("Voltage:");
        uart.print(voltage, 2);
        uart.print("    pH value: ");
        uart.println(pHValue, 2);
        digitalWrite(LED, digitalRead(LED) ^ 1);
        printTime = millis();
    }
}


void loop() {
    checkLocation();
    checkWaterHeight();
    checkPh();
    Serial.print("Temperature: ");
    Serial.println(checkTemperature());
//    sendRequest();
}