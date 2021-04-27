#include <SPI.h>
#include <mySD.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BasicLinearAlgebra.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiMulti.h>
#include "config.h"

using namespace BLA;

Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;
Servo servo;
File dataFile;

TaskHandle_t Task1;
TaskHandle_t Task2;
WiFiMulti wifiMulti;
const char *AP_SSID = WIFI_SSID;
const char *AP_PWD = WIFI_PASSWORD;

float altitude, velocity, acceleration, ax, ay, az, kalmanAltitude;
float liftoffAltitude, apogeeAltitude;
float s, v, a;
float prevAltitude = 2000;
int counter = 0;
int apogeeCounter = 0;
int liftoffcounter = 0;
bool isLaunch = false;
bool isApogee1 = false;
bool isApogee2 = false;
bool isApogee3 = false;
bool state = false;
bool state2 = false;
unsigned long currentMillis, startMillis, duration;

String dataMessage;

float q = 0.0013;

// The system dynamics
BLA::Matrix<3, 3> A = {1.0, 0.1, 0.005,
                       0, 1.0, 0.1,
                       0, 0, 1.0};

// Relationship between measurement and states
BLA::Matrix<2, 3> H = {1.0, 0, 0,
                       0, 0, 1.0};

// Initial posteriori estimate error covariance
BLA::Matrix<3, 3> P = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

// Measurement error covariance
BLA::Matrix<2, 2> R = {0.25, 0,
                       0, 0.75};

// Process noise covariance
BLA::Matrix<3, 3> Q = {q, 0, 0,
                       0, q, 0,
                       0, 0, q};

// Identity Matrix
BLA::Matrix<3, 3> I = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

BLA::Matrix<3, 1> x_hat = {1500.0,
                           0.0,
                           0.0};

BLA::Matrix<2, 1> Y = {0.0,
                       0.0};

void init_components();
void logSDCard();
void appendFile(const char *path, const char *message);
void detectLiftOff(float altitude);
void detectApogee1(float altitude);
void detectApogee2(float velocity, long time);
void detectApogee3(float acceleration, long time);
void startWriting();
void deploy_parachute();
void get_readings();
void kalmanUpdate();
void postDataToServer();

void setup()
{
    Serial.begin(BAUD_RATE);
    delay(SETUP_DELAY);
    init_components();
    delay(SETUP_DELAY);

    xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
    delay(SETUP_DELAY);

    xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
    delay(SETUP_DELAY);

    wifiMulti.addAP(AP_SSID, AP_PWD);
    postDataToServer();
}

void loop()
{
    //    delay(50);
}

// Write the sensor readings on the SD card
void logSDCard()
{
    dataMessage = String(counter) + "," + String(altitude) + "," + String(s) + "," + String(v) + "," + String(a) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(isLaunch) + "," + String(isApogee1) + "," + String(isApogee2) + "," + String(isApogee3) + ".";
    //Serial.print("Save data: ");
    //Serial.println(dataMessage);
    appendFile(dataMessage.c_str());
    Serial.print(altitude);
    Serial.print(" , ");
    Serial.println(s);
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(const char *message)
{
    dataFile = SD.open(LOG_FILE, FILE_WRITE);
    if (!dataFile)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (dataFile.println(message))
    {
        //Serial.println("Message appended\n");
    }
    else
    {
        Serial.println("Append failed");
    }
    dataFile.close();
}

void startWriting()
{
    dataFile = SD.open(LOG_FILE, FILE_WRITE);
    if (dataFile)
    {
        Serial.println("Start writing to test2");
        dataFile.println("Index, Altitude, kalmanAltitude, kalmanVelocity, kalmanAcceleration, ax, ay, az, isLaunch, isApogee1, isApogee2, isApogee3  \r\n");
        dataFile.close();
    }
    else
    {
        Serial.println("File already exists");
    }
}

void detectLiftOff(float altitude1)
{
    if (liftoffcounter == MAX_LIFTOFF_COUNTER)
    {
        isLaunch = true;
        startMillis = millis();
    }
    //Serial.println(altitude1 - prevAltitude);
    if ((altitude1 - prevAltitude) > LIFTOFF_DEVIATION)
    {
        liftoffcounter += 1;
    }
    else
    {
        liftoffcounter = 0;
    }
    //Serial.println(liftoffcounter);
    prevAltitude = altitude1;
}

void detectApogee1(float altitude2)
{
    if (isLaunch == true)
    {
        if (isApogee1 == false)
        {
            if (apogeeCounter == MAX_APOGEE_COUNTER)
            {
                isApogee1 = true;
            }
            if ((prevAltitude - altitude2) > APOGEE_DEVIATION)
            {
                apogeeCounter += 1;
            }
            else
            {
                apogeeCounter = 0;
            }

            prevAltitude = altitude2;
        }
    }
}

void detectApogee2(float velocity1, long time1)
{
    if (isLaunch == true)
    {
        if (isApogee2 == false)
        {
            if (time1 > TIME_TO_CHECK_APOGEE)
            {
                if ((velocity1 <= 1) && (velocity1 >= -1))
                {
                    isApogee2 = true;
                }
            }
        }
    }
}

void detectApogee3(float acceleration1, long time2)
{
    if (isLaunch == true)
    {
        if (isApogee3 == false)
        {
            if (time2 >= TIME_TO_CHECK_APOGEE)
            {
                if ((acceleration1 <= 1) && (acceleration1 >= -1))
                {
                    isApogee3 = true;
                }
            }
        }
    }
}

void init_components()
{
    Serial.println("BMP180 test!");
    if (!bmp.begin())
    {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1)
        {
            delay(SHORT_DELAY);
        }
    }
    Serial.println("BMP180 Found!");

    Serial.println("MPU6050 test!");
    if (!mpu.begin())
    {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        while (1)
        {
            delay(SHORT_DELAY);
        }
    }
    Serial.println("MPU6050 Found!");

    Serial.print("\nInitializing SD card...");
    pinMode(SS, OUTPUT);
    if (!SD.begin(12, 27, 26, 14))
    {
        Serial.println("initialization failed.");
        while (1)
        {
            delay(SHORT_DELAY);
        }
    }
    else
    {
        Serial.println("Wiring is correct and a card is present.");
    }
    Serial.println("initialization done.");

    startWriting();

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    Serial.println("Servo test!");

    servo.attach(SERVO_PIN);
    servo.write(0);
    Serial.println("Servo Initialization done!");
}

void deploy_parachute()
{
    if (isApogee2 == true && (state == false))
    {
        servo.write(SERVO_OPEN_ANGLE);
        state = true;
    }
    if ((isApogee1 == true) && (state == false))
    {
        servo.write(SERVO_OPEN_ANGLE);
        state = true;
    }

    if ((duration == SERVO_DURATION) && (state == false))
    {
        servo.write(SERVO_OPEN_ANGLE);
        state = true;
    }
}

void get_readings()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z;
}

void kalmanUpdate()
{
    //Measurement matrix
    BLA::Matrix<2, 1> Z = {altitude,
                           az};
    //Predicted state estimate
    BLA::Matrix<3, 1> x_hat_minus = A * x_hat;
    //Predicted estimate covariance
    BLA::Matrix<3, 3> P_minus = A * P * (~A) + Q;
    //Kalman gain
    BLA::Matrix<3, 2> K = P_minus * (~H) * ((H * P_minus * (~H) + R)).Inverse();
    //Measurement residual
    Y = Z - (H * x_hat_minus);
    //Updated state estimate
    x_hat = x_hat_minus + K * Y;
    //Updated estimate covariance
    P = (I - K * H) * P_minus;
    Y = Z - (H * x_hat_minus);

    s = x_hat(0);
    v = x_hat(1);
    a = x_hat(2);
}

void Task1code(void *pvParameters)
{
    for (;;)
    {
        get_readings();
    }
}

void Task2code(void *pvParameters)
{
    for (;;)
    {
        kalmanUpdate();
        duration = currentMillis - startMillis;
        digitalWrite(BUZZER_PIN, HIGH);
        detectLiftOff(s);
        detectApogee1(s);
        //detectApogee2(v, duration);
        //detectApogee3(a, duration);
        deploy_parachute();
        counter++;
        logSDCard();
    }
}

void postDataToServer()
{

    Serial.println("Posting JSON data to server...");
    // Block until we are able to connect to the WiFi access point
    if (wifiMulti.run() == WL_CONNECTED)
    {

        HTTPClient http;
        http.begin(SERVER_POST_URL);
        http.addHeader("Content-Type", "application/json");

        StaticJsonDocument<200> doc;
        doc["status"] = "Done";

        String requestBody;
        serializeJson(doc, requestBody);

        int httpResponseCode = http.POST(requestBody);

        if (httpResponseCode > 0)
        {
            String response = http.getString();
            Serial.println(response);
        }
        else
        {
            Serial.printf("Error occurred while sending HTTP POST: %s\n", http.errorToString(httpResponseCode).c_str());
        }
    }
}
