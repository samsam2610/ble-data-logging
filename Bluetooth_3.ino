
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

//BNO055 sensor objects
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x29);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, 0x28);
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
unsigned long time;
uint32_t logtime;

void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  while (1);
}

void setup(void)
{
  //while (!Serial);
  delay(500);

  Serial.begin(115200);
  Serial.println("Twin BNO055 Orientation Sensor Test");
  Serial.println("");

  /* Initialise the first sensor */
  if (!bno1.begin())
  {
    /* There was a problem detecting the BNO055 at 0x28 ... check your connections */
    Serial.print("Ooops, no BNO055 detected at 0x28 ... Check your wiring or I2C ADDR!");
    while (1);
  }

  /* Initialise the second sensor */
  if (!bno2.begin())
  {
    /* There was a problem detecting the BNO055 at 0x29 ... check your connections */
    Serial.print("Ooops, no BNO055 detected at 0x29 ... Check your wiring or I2C ADDR!");
    while (1);
  }

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  Serial.println( F("OK!") );
  ble.echo(false);
  ble.info();
  ble.verbose(false);
  while (! ble.isConnected()) {
    delay(500);
  }
  delay(1000);

  bno1.setExtCrystalUse(true);
  bno2.setExtCrystalUse(true);
}

void loop(void)
{
  /* Get a new sensor event */
  time = millis();
  uint8_t system, gyro, accel, mag = 0;
  bno1.getCalibration(&system, &gyro, &accel, &mag);
  uint8_t sys1 = system;
  bno2.getCalibration(&system, &gyro, &accel, &mag);
  uint8_t sys2 = system;

  logtime = time;
  imu::Quaternion quat1 = bno1.getQuat();
  imu::Vector<3> gyro1 = bno1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> acce1 = bno1.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  imu::Quaternion quat2 = bno2.getQuat();
  imu::Vector<3> gyro2 = bno2.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> acce2 = bno2.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  float w1 = quat1.w();
  float x1 = quat1.x();
  float y1 = quat1.y();
  float z1 = quat1.z();
  float w2 = quat2.w();
  float x2 = quat2.x();
  float y2 = quat2.y();
  float z2 = quat2.z();
  int sign = 1;

  float norm1 = sqrt(pow(w1, 2) + pow(x1, 2) + pow(y1, 2) + pow(z1, 2));
  float norm2 = sqrt(pow(w2, 2) + pow(x2, 2) + pow(y2, 2) + pow(z2, 2));

  float product = ((w1 / norm1) * (w2 / norm2)) - ((x1 / norm1) * (x2 / norm2)) - ((y1 / norm1) * (y2 / norm2)) - ((z1 / norm1) * (z2 / norm2));
  float  angle = (acos((product * 2 - 1) * sign) * 57.2958);
  if (isnan(angle)) {
    sign = -1;
    angle = (acos(product * 2 - sign) * 57.2958) * sign;
  }

  Serial.print(" Data: ");
  Serial.print(",");
  Serial.print(angle);
  Serial.print(",");
  Serial.print(" Gyro 1: ");
  Serial.print(",");
  Serial.print(gyro1.x(), 0);
  Serial.print(",");
  Serial.print(gyro1.y(), 0);
  Serial.print(",");
  Serial.print(gyro1.z(), 0);
  Serial.print(",");
  Serial.print(" Acce 1: ");
  Serial.print(",");
  Serial.print(acce1.x(), 0);
  Serial.print(",");
  Serial.print(acce1.y(), 0);
  Serial.print(",");
  Serial.print(acce1.z(), 0);
  Serial.print(",");
  Serial.print(" Gyro 2: ");
  Serial.print(",");
  Serial.print(gyro2.x(), 0);
  Serial.print(",");
  Serial.print(gyro2.y(), 0);
  Serial.print(",");
  Serial.print(gyro2.z(), 0);
  Serial.print(",");
  Serial.print(" Acce 2: ");
  Serial.print(",");
  Serial.print(acce2.x(), 0);
  Serial.print(",");
  Serial.print(acce2.y(), 0);
  Serial.print(",");
  Serial.print(acce2.z(), 0);
  Serial.print(",");
  Serial.print(acos(abs(acce2.y()) / 10) * 57.2958, 0);
  Serial.println("");

  if (ble.isConnected()) {

    /* Display the floating point data */
    ble.print("AT+BLEUARTTX=");
    ble.print(angle, 2);
    ble.print(",");
    ble.print(gyro1.x(), 2);
    ble.print(",");
    ble.print(gyro1.y(), 2);
    ble.print(",");
    ble.print(gyro1.z(), 2);
    ble.print(",");
    ble.print(acce1.x(), 2);
    ble.print(",");
    ble.print(acce1.y(), 2);
    ble.print(",");
    ble.print(acce1.z(), 2);
    ble.print(",");
    ble.print(gyro2.x(), 2);
    ble.print(",");
    ble.print(gyro2.y(), 2);
    ble.print(",");
    ble.print(gyro2.z(), 2);
    ble.print(",");
    ble.print(acce2.x(), 2);
    ble.print(",");
    ble.print(acce2.y(), 2);
    ble.print(",");
    ble.print(acce2.z(), 2);
    ble.print("");
    ble.println("\\r\\n");
  }
}
