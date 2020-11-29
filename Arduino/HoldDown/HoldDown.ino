#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Orientation.h>
#include <SD.h>
#include <SPI.h>
#include <SPIMemory.h>
#include <Servo.h>

#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

Sd2Card card;
SdVolume volume;
SdFile root;
File file;
SPIFlash flash(9);
Servo xServo;
Servo yServo;

float rad2deg = 360 / (2 * PI);

float gains[] = { -4.1, -0.79, -7.2};
float xLim = 7.04;
float xRatio = 0.33; //get these values
float xZero = 83;
float yLim = 6.40;
float yRatio = 0.82; //get these values
float yZero = 99;

const int8_t SPIPins[4] = { -1, -1, -1, 33};
const int SDCS = 10; // SD card reader chip select pin
const int FLASHCS = 9; // Flash chip select pin
const int gLedPin = 1;
const int rLedPin = 2;
const int buttonPin = 0;
const int xServoPin = 3;
const int yServoPin = 4;
const float p1 = 0.9;
const float p2 = 0.1;

unsigned int count = 0;
unsigned int count0 = 0;
unsigned int calib = 0;
unsigned int n;

uint64_t thisMicros = 0;
uint64_t lastMicros = 0;
double dt;
uint32_t addr = 0;
int buttonState = 0;

//PID Stuff
float cumYawE = 0;
float cumPitchE = 0;
EulerAngles lastError;

struct mount {
  float x, y;
};

EulerAngles gyro0;
EulerAngles gyro1;
EulerAngles gyroF;
EulerAngles oriInitial;
EulerAngles dataOut;

Quaternion zero = Quaternion(1,0,0);
Orientation ori;
Quaternion lastOri;
EulerAngles oriMeasure;
Quaternion setPoint = Quaternion(1, 0, 0);

mount tvcAngles;

void setup() {
  // put your setup code here, to run once:
  pinMode(rLedPin, OUTPUT);
  pinMode(gLedPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  flash.begin();
  bno.begin();
  bno.setExtCrystalUse(true);

  xServo.attach(xServoPin);
  yServo.attach(yServoPin);

  Serial.begin(115200);

  thisMicros = lastMicros = micros();

  oriInitial.roll = 0;
  gyro0.yaw = gyro0.pitch = gyro0.roll = 0;

  flash.eraseChip();
}

void loop() {
  buttonState = digitalRead(buttonPin);
  thisMicros = micros();

  if ((buttonState == HIGH) & (calib == 0)) {
    calib = 1;
    count0 = count;
  }

  float dt = (float)(thisMicros - lastMicros) / 1000000.; // Finds elapsed microseconds since last update, converts to float, and converts to seconds
  lastMicros = thisMicros; // We have updated, set the new timestamp

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  gyro1.yaw = gyro.z() * DEG_TO_RAD;
  gyro1.pitch = gyro.x() * DEG_TO_RAD;
  gyro1.roll = gyro.y() * DEG_TO_RAD;

  gyroF = filter(gyro0, gyro1);
  gyro0 = gyroF;

  ori.update(gyroF.yaw, gyroF.pitch, gyroF.roll, dt); // '* DEG_TO_RAD' after all gyro functions if they return degrees/sec
  oriMeasure = ori.toEuler();

  if ((calib == 1) & ((count - count0) < 100)) {
    n = count - count0;

    double yaw = -atan2(grav.y(), grav.x()) - PI / 2;
    double pitch = atan2(grav.y(), grav.z()) + PI / 2;

    oriInitial.yaw += (1 / (n + 1)) * (yaw - oriInitial.yaw);
    oriInitial.pitch += (1 / (n + 1)) * (pitch - oriInitial.pitch);

    ori.orientation = ori.orientation.from_euler_rotation(oriInitial.yaw, oriInitial.pitch, oriInitial.roll);
  }

  flash.writeAnything(addr, oriMeasure);

  if (count > 100000) {
    SD.begin(SDCS);
    for (int i = 0; i <= addr; i += sizeof(oriMeasure)) {
    file = SD.open("test.txt", FILE_WRITE);
      flash.readAnything(i, dataOut);
      if (file) { // if file object is open, write sensor readings to SD object
        file.println(String(dataOut.yaw, 16) + "," + String(dataOut.pitch, 16) + "," + String(dataOut.roll, 16));
        file.close();
      }
    }
    flash.eraseChip();
    while (1);
  }

  tvcAngles = pid(ori, lastOri, setPoint, dt);
  commandTVC(tvcAngles);

  //Serial.print(oriMeasure.yaw);
  //Serial.print(",");
  //Serial.println(oriMeasure.pitch);

  addr += sizeof(oriMeasure);
  count++;
  delay(BNO055_SAMPLERATE_DELAY_MS);

  lastOri = ori.orientation;
}

EulerAngles filter(EulerAngles gyro0, EulerAngles gyro1) {
  EulerAngles gyroF;

  gyroF.yaw = p1 * gyro0.yaw + p2 * gyro1.yaw;
  gyroF.pitch = p1 * gyro0.pitch + p2 * gyro1.pitch;
  gyroF.roll = p1 * gyro0.roll + p2 * gyro1.roll;

  return gyroF;
}

void commandTVC(mount angles) {
  float x = angles.x;
  float y = angles.y;

  if (x > xLim) {
    x = xLim;
  }
  if (x < -xLim) {
    x = -xLim;
  }
  if (y > yLim) {
    y = yLim;
  }
  if (y < -yLim) {
    y = -yLim;
  }

  Serial.print(x);
  Serial.print(",");
  Serial.println(y);

  x = x / xRatio + xZero;
  y = y / yRatio + yZero;

  xServo.write(x);
  yServo.write(y);
}

mount pid(Orientation ori, Quaternion lastOri, Quaternion setPoint, double dt) {
  Quaternion pos = ori.orientation.rotate(zero);
  Quaternion errorQuat = pos.rotation_between_vectors(setPoint);
  EulerAngles errorEuler = ori.quaternionToEuler(errorQuat);
  mount out;

  float yawE = errorEuler.yaw * rad2deg;
  float pitchE = errorEuler.pitch * rad2deg;
  float yawE0 = lastError.yaw * rad2deg;
  float pitchE0 = lastError.pitch * rad2deg;

  float rateYawE = (yawE - yawE0) / dt;
  float ratePitchE = (pitchE - pitchE0) / dt;

  cumYawE += yawE * dt;
  cumPitchE += pitchE * dt;

  lastError = errorEuler;

  out.x = gains[0]*yawE + gains[1]*cumYawE + gains[2]*rateYawE;
  out.y = gains[0]*pitchE + gains[1]*cumPitchE + gains[2]*ratePitchE;

  return out;
}
