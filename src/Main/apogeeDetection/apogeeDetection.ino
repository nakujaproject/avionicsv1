                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      #include <SPI.h>
#include <mySD.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BasicLinearAlgebra.h>
#include <ESP32Servo.h>

#include <HTTPClient.h>

#include "config.h"

String header;

using namespace BLA;

Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;
Servo servo;
File dataFile;

WiFiServer server(80);

TaskHandle_t Task1;
TaskHandle_t Task2;


int check_server = 0;
float altitude, velocity, acceleration, ax, ay, az, kalmanAltitude;
float liftoffAltitude, apogeeAltitude;
float s, v, a;
float prevAltitude = 2000;
float prevAltitude2 = 1000;
int counter = 0;
int apogeeCounter = 0;
int liftoffcounter = 0;
bool isLaunch = false;
bool isApogee1 = false;
bool isApogee2 = false;
bool isApogee3 = false;
bool state = false;
bool state2 = false;
unsigned long currentMillis, startMillis;
long currentltime = millis();
unsigned long startMillis2;
long duration = currentltime - startMillis2;

String dataMessage;

float q = 0.0001;

float T = 0.1;

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
BLA::Matrix<3, 3> Q = {q, q, q,
                       q, q, q,
                       q, q, q};

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

void Task1code(void *pvParameters);
void Task2code(void *pvParameters);

void setup()
{
    Serial.begin(BAUD_RATE);
    delay(SETUP_DELAY);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(SERVO_PIN, OUTPUT);
      
    init_components();
    buzzer(ALLCOMPONENTS);
    delay(SETUP_DELAY);
    
    connectwifi();
    buzzer(WIFICONNECTED);
    
    xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 0);
    delay(SETUP_DELAY);

    xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);
    delay(SETUP_DELAY);


  // Connect to Wi-Fi network with SSID and password
 

}

unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

String output4State = "off";

void loop(){
  }

void buzzer(int num)
{
      for(int c =0; c <= num; c++){
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      Serial.println("buzz");
      digitalWrite(BUZZER_PIN, LOW);
      delay(100);
      }
  }

void connectwifi()
{
   Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("Connecting");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  }

  unsigned long currenttime9;
unsigned long starttime9;
unsigned long duration3;
unsigned long prevtime9 = 0;

void Task1code(void *pvParameters)
{
    for (;;)
    {
//      currenttime9 = millis();
//      duration3 = (currenttime9 - prevtime9);
//      Serial.println(duration3);
//      prevtime9 = currenttime9;
        wifi();
        get_readings();
        
        

    }
}

unsigned long currenttime8;
unsigned long starttime8;
unsigned long duration2;
unsigned long prevtime8 = 0;
void Task2code(void *pvParameters)
{
    starttime8 = millis();
    
    for (;;)
    {  
      currenttime8 = millis();
 
     if((currenttime8 - starttime8) <= 15000){
          digitalWrite(BUZZER_PIN, HIGH);
          }
          else{
            digitalWrite(BUZZER_PIN, LOW);
            }


       duration = currenttime8 - startMillis2;
        
        
        kalmanUpdate();
     //   duration = currentMillis - startMillis;
        //digitalWrite(BUZZER_PIN, HIGH);
//    Serial.print(altitude);
//    Serial.print(" , ");
  //  Serial.println(s);        
        
        Serial.print(s);
        Serial.println("....");
        //logSDCard();
        
        if((currenttime8 - starttime8) >= 60000){  
        detectLiftOff(s);
        detectApogee1(s);
        detectApogee2(v, duration);
        detectApogee3(a, duration);
        }
        
        

        deploy_parachute();
        counter++;
        logSDCard();
    }
}

//function to initialize sensors and the sd card module
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



void get_readings()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    ax = a.acceleration.x + 0.35;
    ay = a.acceleration.y -0.3;
    az = a.acceleration.z -10.31;
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
       // Serial.println("Message appended\n");
    }
    else
    {
        Serial.println("Append failed");
    }
    dataFile.close();
}

// Write the sensor readings on the SD card
void logSDCard()
{
    dataMessage = String(counter) + "," + String(altitude) + "," + String(s) + "," + String(v) + "," + String(a) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(isLaunch) + "," + String(isApogee1) + "," + String(isApogee2) + "," + String(isApogee3) + ".";
//    Serial.print("Save data: ");
   Serial.println(dataMessage);
    appendFile(dataMessage.c_str());
//    Serial.print(altitude);
//    Serial.print(" , ");
//    Serial.println(s);
}

//Function to detect rocket launch
void detectLiftOff(float altitude1)
{

    if (isLaunch == false){
    if (liftoffcounter == MAX_LIFTOFF_COUNTER)
    {
        isLaunch = true;
//        startMillis2 = millis();
//        Serial.print(startMillis2);
//        Serial.println("...");
    }
//    Serial.println(prevAltitude);
    if ((altitude1 - prevAltitude) > LIFTOFF_DEVIATION)
    {
        liftoffcounter += 1;
    }
    else
    {
        liftoffcounter = 0;
    }

    prevAltitude = altitude1;
}
}

//Apogee detection using altitude values
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
            //Serial.println(prevAltitude);
            if ((prevAltitude2 - altitude2) > APOGEE_DEVIATION)
            {
                apogeeCounter += 1;
            }
            else
            {
                apogeeCounter = 0;
            }
            



            prevAltitude2 = altitude2;
        }
    }
}



//currentltime = millis();
//long duration = currentltime - startMillis2;

//Apogee detection using velocity
void detectApogee2(float velocity1, long time1)
{
    if (isLaunch == true)
    {
        if (isApogee2 == false)
        {
            
            Serial.print(time1);
            Serial.println("..");

            
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

//function to restart the esp32 via WiFi
void wifi()
{
    WiFiClient client = server.available();   // Listen for incoming clients
//  Serial.println("Looping.");
 int var = 0;
  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();         
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // RESET AND DONE
 
            
            if (header.indexOf("GET /4/on") >= 0) {
              Serial.println("GPIO 4 on");
              output4State = "on";

              
             
            } else if (header.indexOf("GET /4/off") >= 0) {
              Serial.println("GPIO 4 off");
              output4State = "off";
              
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>NAKUJA AVIONICS</h1>");
            

               
            // Display RESET AND DONE BUTTON  
            client.println("<p>RESTART ESP- State " + output4State + "</p>");
            // If the output4State is off, it displays the ON button       
            if (output4State=="off") {
              client.println("<p><a href=\"/4/on\"><button class=\"button\">RESET</button></a></p>");
            } else {
              client.println("<p>HIT DONE FOR RESET BUTTON </p>");
              client.println("<p><a href=\"/4/off\"><button class=\"button button2\">DONE</button></a></p>");
              //delay(1000);
              
              var = 1;
            }
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            client.stop();
            
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    
    if (var == 1){
              ESP.restart();
              }
    Serial.println("");
    delay(100);
  }

}
