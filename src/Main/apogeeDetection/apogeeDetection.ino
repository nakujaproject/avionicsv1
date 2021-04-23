#include <SPI.h>
#include <mySD.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BasicLinearAlgebra.h>
#include <ESP32Servo.h>

#define seaLevelPressure_hPa 1024

using namespace BLA;

Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;
Servo servo;

int counter = 0;
float altitude, velocity, acceleration, ax, ay, az, kalmanAltitude;
float liftoffAltitude, apogeeAltitude;
float s, v, a, reac, res;
int prevAltitude = 2000;
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
File dataFile;

float q = 0.0001;

// The system dynamics
BLA::Matrix<3, 3> A = {1.0, 0.1, 0.01,
                       0, 1.0, 0.1,
                       0, 0, 1};

// Relationship between measurement and states
BLA::Matrix<2, 3> H = {1.0, 0, 0,
                       0, 0, 1.0};

// Initial posteriori estimate error covariance
BLA::Matrix<3, 3> P = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

// Measurement error covariance
BLA::Matrix<2, 2> R = {0.25, 0,
                       0, 0.888};

// Process noise covariance
BLA::Matrix<3, 3> Q = {q, 0, 0,
                       0, q, 0,
                       0, 0, q};

// Identity Matrix
BLA::Matrix<3, 3> I = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

BLA::Matrix<3, 1> x_hat = {1550.0,
                           0.0,
                           0.0};

BLA::Matrix<2, 1> Y = {0.0,
                       0.0};

void init_components();
void logSDCard();
void writeFile(const char *path, const char *message);
void appendFile(const char *path, const char *message);
void detectLiftOff(float altitude);
void detectApogee1(float altitude);
void detectApogee2(float velocity, long time);
void detectApogee3(float acceleration, long time);
void startWriting();
void deploy_parachute();
void get_readings();
void kalmanUpdate();

TaskHandle_t Task1, Task2;

void setup()
{
    Serial.begin(115200);
    delay(2000);
    init_components();
    delay(2000);
    xTaskCreatePinnedToCore(
        codeForTask0,
        "led1Task",
        2000,
        NULL,
        1,
        &Task1,
        0);
    delay(500); // needed to start-up task1

    xTaskCreatePinnedToCore(
        codeForTask1,
        "led2Task",
        2000,
        NULL,
        1,
        &Task2,
        1);
}

void loop()
{
    delay(50);
}

// Write the sensor readings on the SD card
void logSDCard()
{
    dataMessage = String(counter) + "," + String(altitude) + "," + String(s) + "," + String(v) + "," + String(a) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(isLaunch) + "," + String(isApogee1) + "," + String(isApogee2) + "," + String(isApogee3) + ","
                                                                                                                                                                                                                                                                                   "\r\n";
    Serial.print("Save data: ");
    Serial.println(dataMessage);
    appendFile(dataMessage.c_str());
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(const char *message)
{
    dataFile = SD.open("test2.txt", FILE_WRITE);
    if (!dataFile)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (dataFile.println(message))
    {
        Serial.println("File written");
    }
    else
    {
        Serial.println("Write failed");
    }
    dataFile.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(const char *message)
{
    dataFile = SD.open("test2.txt", FILE_WRITE);
    if (!dataFile)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (dataFile.println(message))
    {
        Serial.println("Message appended");
    }
    else
    {
        Serial.println("Append failed");
    }
    dataFile.close();
}

void startWriting()
{
    dataFile = SD.open("test2.txt", FILE_WRITE);
    if (dataFile)
    {
        Serial.println("Start writing to test2");
        dataFile.println("Index, Altitude, kalmanAltitude, kalmanVelocity, kalmanAcceleration, ax, ay, az, res, reac, isApogee1, isApogee2, isApogee3  \r\n");
        dataFile.close();
    }
    else
    {
        Serial.println("File already exists");
    }
}

void detectLiftOff(float altitude)
{
    int alt = (int)altitude;
    if (currentMillis >= 10000)
    {
        if (liftoffcounter == 5)
        {
            isLaunch = true;
            startMillis = millis();
        }
        if (alt > prevAltitude)
        {
            liftoffcounter = liftoffcounter + 1;
        }
        else
        {
            liftoffcounter = 0;
        }
        prevAltitude = alt;
    }
}

void detectApogee1(float altitude)
{
    if (isLaunch == true)
    {
        if (isApogee1 == false)
        {
            if (altitude > liftoffAltitude)
            {
                if (apogeeCounter == 3)
                {
                    isApogee1 = true;
                }
                if (altitude < prevAltitude)
                {
                    apogeeCounter = apogeeCounter + 1;
                }
                else
                {
                    apogeeCounter = 0;
                }
            }
            prevAltitude = altitude;
        }
    }
}

void detectApogee2(float velocity, long time)
{
    if (isLaunch == true)
    {
        if (isApogee2 == false)
        {
            if (time > 2000)
            {
                if ((velocity <= 1) && (velocity >= -1))
                {
                    isApogee2 = true;
                }
            }
        }
    }
}

void detectApogee3(float acceleration, long time)
{
    if (isLaunch == true)
    {
        if (isApogee3 == false)
        {
            if (time >= 2000)
            {
                if ((acceleration <= 1) && (acceleration >= -1))
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
            delay(10);
        }
    }
    Serial.println("BMP180 Found!");

    Serial.println("MPU6050 test!");
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    Serial.print("\nInitializing SD card...");
    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    pinMode(SS, OUTPUT);
    if (!SD.begin(12, 27, 26, 14))
    {
        Serial.println("initialization failed.");
        while (1)
        {
            delay(10);
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

    servo.attach(16);
    servo.write(0);
    Serial.println("Servo Initialization done!");
}

void deploy_parachute()
{
    if (isApogee2 == true && (state == false))
    {
        servo.write(90);
        Serial.println("Hello");
        state = true;
    }
    if ((isApogee1 == true) && (state == false))
    {
        Serial.println("World");
        servo.write(90);
        state = true;
    }

    if ((duration == 30000) && (state == false))
    {
        Serial.println("World");
        servo.write(90);
        state = true;
    }
}

void get_readings()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    altitude = bmp.readAltitude(seaLevelPressure_hPa * 100);
    ax = a.acceleration.x;
    ay = a.acceleration.y;
    az = -1 * a.acceleration.z;
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

void codeForTask0(void *parameter)
{
    for (;;)
    {
        Serial.print("This Task runs on Core: ");
        Serial.println(xPortGetCoreID());

        duration = currentMillis - startMillis;

        detectLiftOff(s);
        detectApogee1(s);
        detectApogee2(v, duration);
        detectApogee3(a, duration);

        logSDCard();
        counter++;
        deploy_parachute();
        delay(50);
    }
}

void codeForTask1(void *parameter)
{
    for (;;)
    {
        Serial.print("This Task runs on Core: ");
        Serial.println(xPortGetCoreID());

        currentMillis = millis();
        get_readings();
        kalmanUpdate();
        delay(50);
    }
}
