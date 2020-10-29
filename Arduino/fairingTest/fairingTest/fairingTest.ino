#include <Servo.h>

Servo myservo;

int pos0 = 90;
int pos1 = 0;

void setup() {
  myservo.attach(3);
}

void loop() {
  myservo.write(pos0);
  delay(6000);
  myservo.write(pos1);
  delay(6000);
}
