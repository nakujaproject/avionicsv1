int relay = 13;
volatile byte relayState = LOW;
boolean launched = false;
void setup() {
  // put your setup code here, to run once:
pinMode(3, INPUT_PULLUP);
pinMode(13, OUTPUT);
Serial.begin(9600);
//start
}

void loop() {
  // put your main code here, to run repeatedly:
//delay(5000);
int button = digitalRead(3);
if (launched == 0 && button ==1){
  digitalWrite(relay, HIGH);
  launched = true;
  Serial.print("HIGH");
  }
  else{
    Serial.println("done");
    }
}
