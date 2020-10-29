#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>
#include <SPIMemory.h>
#include <Servo.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BMP280_SAMPLERATE_DELAY_MS (100)

// flash chip stuff; some of this is likely unnecessary but I'm including it just in case
//#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
// Required for Serial on Zero based boards
//#define Serial SERIAL_PORT_USBVIRTUAL
//#endif

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

int8_t SPIPins[4] = {-1, -1, -1, 33};
uint32_t addr = 0;
double voltageReading;
const int SDCS = 10; // SD card reader chip select pin
const int FLASHCS = 9; // Flash chip select pin
double divisionRatio = 11;
double analogRatio = 0.00284;
String serialOutput;
String flashOutput;
int count = 0;
float euler[3];
float gains[] = {-1,0,0};
float x,y,z;
float rad2deg = 360/(2*PI);
float xLim = 2.87; //redesign mount to increase this
float xRatio = 0.33; //get these values
int xZero = 56;
float yLim = 6.40;
float yRatio = 0.87; //get these values
int yZero = 49;

struct rawData {
  double accX;
  double accY;
  double accZ;
  double gyrW;
  double gyrX;
  double gyrY;
  double gyrZ;
  double alt;
  double voltageCorrected;
};

struct rawDataOut {
  double accXO;
  double accYO;
  double accZO;
  double gyrWO;
  double gyrXO;
  double gyrYO;
  double gyrZO;
  double altO;
  double voltageCorrectedO;
};

void setup() {
  delay(2000);
  //Serial.begin(115200);
  bno.begin();
  bmp.begin();
  SD.begin(SDCS);
  flash.begin();
  delay(1000);
  
  //if (!bno.begin()) {
  //  Serial.print("No BNO055 detected");
  //  while (1);
  //}
  //if (!bmp.begin()) {
  //  Serial.print("No BMP280 detected");
  //  while (1);
  //}
  // if (!SD.begin(SDCS)) {
  //  Serial.print("No SD card detected");
  //  while (1);
  //}
  //if (!flash.begin()) {
  //  Serial.print("No flash chip detected");
  //  while (1);
  //}
  int8_t bnoTemp = bno.getTemp();
  int8_t bmpTemp = bmp.readTemperature();
  bno.setExtCrystalUse(false);
  //Serial.print(F("IMU Temperature: "));
  //Serial.println(bnoTemp);
  //Serial.print(F("Baro Temperature: "));
  //Serial.println(bmpTemp);
  flash.eraseChip();

  xServo.attach(3);
  yServo.attach(4);
}

void loop() {
  //calibration
  uint8_t system, gyro, accel, mg = 0;
  bno.getCalibration(&system,&gyro,&accel,&mg);
  //Serial.print(gyro);
  //Serial.print(",");
  //Serial.print(accel);
  //Serial.print(",");
  //Serial.print(mg);
  //Serial.print(",");
  //Serial.println(system);
  
  //voltage divider reading
  voltageReading = analogRead(A3);
  rawData current;
  rawDataOut currentOut;
  current.voltageCorrected = voltageReading * analogRatio * divisionRatio;
  
  //i2c readings
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Quaternion gyr = bno.getQuat();
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  current.alt = bmp.readAltitude(1013.55);

  current.accX = acc.x();
  current.accY = acc.y();
  current.accZ = acc.z();
  current.gyrW = gyr.w();
  current.gyrX = gyr.x();
  current.gyrY = gyr.y();
  current.gyrZ = gyr.z();

  //serialOutput = String(current.accX,16) + "," + String(current.accY,16) + "," + String(current.accZ,16) + "," + String(current.gyrW,16) + "," + String(current.gyrX,16) + "," + String(current.gyrY,16) + "," + String(current.gyrZ,16) + "," + String(current.alt,16) + "," + String(current.voltageCorrected,16);
  //Serial.println(serialOutput);
  //serialOutput = "";

  euler[1] = atan2(2*(gyr.w()*gyr.x()+gyr.y()*gyr.z()),1-2*(gyr.x()*gyr.x()+gyr.y()*gyr.y()))+PI/2;
  euler[0] = asin(2*(gyr.w()*gyr.y()-gyr.z()*gyr.x()));
  euler[2] = atan2(2*(gyr.w()*gyr.z()+gyr.x()*gyr.y()),1-2*(gyr.y()*gyr.y()+gyr.z()*gyr.z()))+2.55;

  x = -euler[0]*rad2deg; //rocket yaw
  y = -euler[1]*rad2deg; //rocket pitch
  z = -euler[2]*rad2deg;
  
  if (x > yLim) { 
    x = yLim;
  }
  if (x < -yLim) {
    x = -yLim;
  }
  if (y > xLim) {
    y = xLim;
  }
  if (y < -xLim) {
    y = -xLim;
  }

  x = x/yRatio+yZero;
  y = y/xRatio+xZero;

  serialOutput = String(x)+","+String(y)+","+String(z);
  //serialOutput = String(euler[0]) + "," + String(euler[1]) + "," + String(euler[2]);
  //Serial.println(serialOutput);
  serialOutput = "";

  xServo.write(y); //rocket pitch
  yServo.write(x); //rocket yaw
  flash.writeAnything(addr, current); // Write it to flash
  //Serial.println(flash.readAnything(addr, currentOut));
  
  if (count > 10000) {
    for (int i = 0; i <= addr; i = i + sizeof(currentOut)) {
      file = SD.open("test.txt", FILE_WRITE); //open file object
      flash.readAnything(i, currentOut);
      if (file) { // if file object is open, write sensor readings to SD object
        file.println(String(currentOut.gyrWO,16) + "," + String(currentOut.gyrXO,16) + "," + String(currentOut.gyrYO,16) + "," + String(currentOut.gyrZO,16));
        file.close();
      }
    }
    flash.eraseChip();
    while(1);
  }
  
  addr = addr + sizeof(rawData);
  count++;
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
