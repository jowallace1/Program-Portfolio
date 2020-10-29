#include <Servo.h>

float rad2deg = 360/(2*PI);
float xLim = 2.87; //redesign mount to increase this
float xRatio = 0.33; //degrees theta1 per 1 degree theta0
float xZero = 56;
float yLim = 6.40;
float yRatio = 0.87; //degrees theta1 per 1 degree theta0
float yZero = 49;
float x;
float y;
float pos;
float i = 0;

Servo xServo;
Servo yServo;

void setup() {
  // put your setup code here, to run once:
  xServo.attach(3);
  yServo.attach(4);

  xServo.write(pos);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (i = -yLim; i <= yLim; i += 0.1) {
    pos = i/yRatio+yZero;
    yServo.write(pos);
    delay(15);
    Serial.println(i);
  }
  for (i = yLim; i >= -yLim; i -= 0.1) {
    pos = i/yRatio+yZero;
    yServo.write(pos);
    delay(15);
    Serial.println(i);
  }
}
