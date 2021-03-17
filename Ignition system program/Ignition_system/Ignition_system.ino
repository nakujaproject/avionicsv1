int led = 13;
boolean launched = false;
void setup() {
  // put your setup code here, to run once:
pinMode(12, INPUT_PULLUP);
pinMode(13, OUTPUT);
Serial.begin(9600);
//start
}

void loop() {
  // put your main code here, to run repeatedly:
delay(5000);
int button = digitalRead(12);
if (launched == 0 && button ==1){
  digitalWrite(13, HIGH);
  launched = true;
  Serial.print("HIGH");
  }
  else{
    Serial.println("done");
    }
}
