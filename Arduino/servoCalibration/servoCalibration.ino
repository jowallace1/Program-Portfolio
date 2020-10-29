#include <Servo.h>

Servo myservo;

int potPin = A7;
int servoPin = 3;
int value;
int pos;

void setup() {
  // put your setup code here, to run once:
  pinMode(potPin, INPUT);
  Serial.begin(9600);
  myservo.attach(servoPin);
}

void loop() {
  // put your main code here, to run repeatedly:
  value = analogRead(potPin);
  pos = map(value, 0, 1023, 0, 360);
  myservo.write(pos);
  Serial.println(pos);
}
