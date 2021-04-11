#include <ESP32Servo.h>

bool state = false;
bool state2 = false;

unsigned long currentMillis;
unsigned long startMillis;
Servo servo;

void setup() {
	Serial.begin(9600);
	servo.attach(32);
	startMillis = millis();
	servo.write(0);
}

void loop() {
	// put your main code here, to run repeatedly:
	currentMillis = millis();
	Serial.println(currentMillis);
	if ((currentMillis-startMillis >= 6000) && (state == false)){
		servo.write(90);
		Serial.println("Hello");
		state = true;
	}
	if (currentMillis-startMillis >= 20000 && (state2 == false)){
		Serial.println("World");
		servo.write(0);
		state2 = true;
	}
}
