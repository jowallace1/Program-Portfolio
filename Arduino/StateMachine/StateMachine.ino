#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <utility/imumaths.h>
#include <Orientation.h>
#include <SD.h>
#include <SPI.h>
#include <SPIMemory.h>
#include <Servo.h>

#define BNO055_SAMPLERATE_DELAY_MS (10)
#define BMP280_SAMPLERATE_DELAY_MS (10)

#if defined (SIMBLEE)
#define RANDPIN 1
#else
#if defined(ARCH_STM32)
#define RANDPIN PA0
#else
#define RANDPIN A0
#endif
#endif

#define TRUE 1
#define FALSE 0

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
Adafruit_BMP280 bmp;

Sd2Card card;
SdVolume volume;
SdFile root;
File file;
SPIFlash flash(9);
Servo xServo;
Servo yServo;

//Constant Variables
const int8_t SPIPins[4] = { -1, -1, -1, 33};
const int SDCS = 10; // SD card reader chip select pin
const int FLASHCS = 9; // Flash chip select pin
const int gLedPin = 1;
const int rLedPin = 2;
const int buttonPin = 0;
const double divisionRatio = 11;
const double analogRatio = 0.00284;

//Random
int count = 0;
float gains[] = { -4.1, -0.79, -7.2};
double x, y, z;
float rad2deg = 360 / (2 * PI);
float xLim = 7.04; //redesign mount to increase this
float xRatio = 0.33; //get these values
int xZero = 84;
float yLim = 6.40;
float yRatio = 0.87; //get these values
int yZero = 49;
double lastAlt = 0;
uint64_t thisMicros = 0;
uint64_t lastMicros = 0;
double dt;
uint32_t addr = 0;
int buttonState = 0;

//Data
String serialOutput;
String flashOutput;

EulerAngles oriInitial;
double pressure0 = 0;

double alt0 = 0;

struct rawData {
  double accel[3];
  double alt, pressure, voltage, veloY;
  Orientation ori;
  EulerAngles euler;
  unsigned int state;
};

rawData raw;

EulerAngles gyro0;
EulerAngles gyro1;
EulerAngles gyroF;

void setup() {
  // put your setup code here, to run once:
  pinMode(rLedPin, OUTPUT);
  pinMode(gLedPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  flash.begin();
  SD.begin(SDCS);
  bmp.begin();
  bno.begin();
  bno.setExtCrystalUse(true);

  Serial.begin(115200);

  thisMicros = lastMicros = micros(); // Set starting time after init/calibration

  raw.state = 0;
  oriInitial.roll = 0;
  gyro0.yaw = gyro0.pitch = gyro0.roll = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  rawData raw;

  thisMicros = micros();
  dt = (double)(thisMicros - lastMicros) / 1000000.;

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyro1.yaw = gyro.z() * DEG_TO_RAD;
  gyro1.pitch = gyro.x() * DEG_TO_RAD;
  gyro1.roll = gyro.y() * DEG_TO_RAD;
  gyroF = filter(gyro0, gyro1);
  gyro0 = gyroF;

  raw.ori.update(gyroF, dt);
  raw.euler = raw.ori.toEuler();
  raw.euler = 

  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  raw.pressure = bmp.readPressure()
  if (!(pressure0 == 0)) {
    raw.alt = readAltitude(pressure0);
  }

  lastMicros = thisMicros; // We have updated, set the new timestamp

  raw.veloY = (raw.alt - alt0) / dt;
  alt0 = raw.alt;

  flash.writeAnything(addr, raw);
  addr = addr + sizeof(raw);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void stateMachine() {
  switch (raw.state) {
    case 0: {
        buttonState = digitalRead(buttonPin);
        digitalWrite(gLedPin, LOW);
        digitalWrite(rLedPin, LOW);

        count++;

        if (buttonState == HIGH & count >= 100) {
          raw.state = 1;
        }
      }
      break;

    case 1: {
        digitalWrite(gLedPin, LOW);
        digitalWrite(rLedPin, HIGH);

        double yaw = 0;
        double pitch = 0;
        
        if (state_prev = 0) {
          for (int i = 0; i < 1000; i++) {

            pitch += atan2(grav.y(), grav.z()) + PI / 2;
            yaw += -atan2(grav.y(), grav.x()) - PI / 2;

            pressure0 += raw.pressure;

            delay(BNO055_SAMPLERATE_DELAY_MS);
          }

          pitch /= 1000;
          yaw /= 1000;

          oriInitial.yaw = yaw;
          oriInitial.pitch = pitch;

          pressure0 /= 1000;

          digitalWrite(gLedPin, HIGH);
          digitalWrite(rLedPin, LOW);

        }
        //transition if acc.y > some nonzero number
      }
      break;

    case 2: {
        digitalWrite(gLedPin, LOW);
        digitalWrite(rLedPin, LOW);

        

        
      }
      break;

    case 3: {
      }
      break;

    case 4: {

      }
      break;

    case 5: {

      }
      break;
  }
  unsigned int state_prev = raw.state;
}

EulerAngles filter(EulerAngles gyro0, EulerAngles gyro1) {
  EulerAngles gyroF;

  gyroF.yaw = p1 * gyro0.yaw + p2 * gyro1.yaw;
  gyroF.pitch = p1 * gyro0.pitch + p2 * gyro1.pitch;
  gyroF.roll = p1 * gyro0.roll + p2 * gyro1.roll;

  return gyroF;
}
