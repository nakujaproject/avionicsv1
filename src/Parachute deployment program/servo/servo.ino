#include <Servo.h>

unsigned long currentMillis;
unsigned long startMillis;
Servo servo;
void setup() {
	Serial.begin(115200);
	servo.attach(PA0);
	startMillis = millis();
	servo.write(0);
}

void loop() {
	// put your main code here, to run repeatedly:
	currentMillis = millis();
	if (currentMillis-startMillis == 6000){
		Serial.write("Turning");
		servo.write(90);
	}

	if (currentMillis-startMillis == 20000){
		servo.write(0);
	}
}
