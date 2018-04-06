
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"

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
unsigned long old_time;
unsigned long new_time;
unsigned interval_time;
uint32_t logtime;

double acce_1[3][2] = {{0,0}, {0,0}, {0,0}};
double vel_1[3][2] = {{0,0}, {0,0}, {0,0}};
double pos_1[3][2] = {{0,0}, {0,0}, {0,0}};

double acce_2[3][2] = {{0,0}, {0,0}, {0,0}};
double vel_2[3][2] = {{0,0}, {0,0}, {0,0}};
double pos_2[3][2] = {{0,0}, {0,0}, {0,0}};

void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  while (1);
}

String addString(String value, String stringOriginal, String separator)
{
  value += separator;
  stringOriginal += value;
  return stringOriginal;
}

String addStringPlus( imu::Vector<3> dataIMU, String stringOriginal, int countDecimal, String seperator)
{
  String value = String( dataIMU.x(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  value = String( dataIMU.y(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  value = String( dataIMU.z(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  return stringOriginal;
}

String addStringArray(double array[], int size, String stringOriginal,int countDecimal, String seperator)
{
  for (int j = 0; j < size + 1; j++)
  {
    String value = String(array[j], countDecimal);
    stringOriginal = addString(value, stringOriginal, seperator);
  }
  return stringOriginal;
}

String CalibrationStatus(Adafruit_BNO055 bno, int code)
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  bno.getCalibration(&system, &gyro, &accel, &mag);

  String value = " ";
  value += String(code);
  value += String(system, DEC);
  value += String(gyro, DEC);
  value += String(accel, DEC);
  value += String(mag, DEC);

  return value;

}

void BLEsetup()
{
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  Serial.println( F("OK!") );
  ble.echo(false);
  ble.info();
  ble.verbose(false);
  //  while (! ble.isConnected()) {
  //    delay(500);
  //  }
  // ble.setMode(BLUEFRUIT_MODE_DATA);
  delay(3000);
}

void Sensorsetup(void)
{
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

  bno1.setExtCrystalUse(true);
  delay(1000);
  bno2.setExtCrystalUse(true);
  delay(1000);
}



void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData, int code)
{
  Serial.print(code);
  Serial.print(" ");
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" ");
  Serial.print(calibData.accel_offset_y); Serial.print(" ");
  Serial.print(calibData.accel_offset_z); Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.println(calibData.mag_radius);
}

adafruit_bno055_offsets_t CalibrationParam(
                                            int16_t accel_x,
                                            int16_t accel_y,
                                            int16_t accel_z,
                                            int16_t gyro_x,
                                            int16_t gyro_y,
                                            int16_t gyro_z,
                                            int16_t mag_x,
                                            int16_t mag_y,
                                            int16_t mag_z,
                                            int16_t accel_radius,
                                            int16_t mag_radius
                                          )
{
  adafruit_bno055_offsets_t CalibrationParam;
  CalibrationParam.accel_offset_x = accel_x;
  CalibrationParam.accel_offset_y = accel_y;
  CalibrationParam.accel_offset_z = accel_z;
  CalibrationParam.gyro_offset_x = gyro_x;
  CalibrationParam.gyro_offset_y = gyro_y;
  CalibrationParam.gyro_offset_z = gyro_z;
  CalibrationParam.mag_offset_x = mag_x;
  CalibrationParam.mag_offset_y = mag_y;
  CalibrationParam.mag_offset_z = mag_z;
  CalibrationParam.accel_radius = accel_radius;
  CalibrationParam.mag_radius = mag_radius;
  return CalibrationParam;
}

void Calibrationsetup(Adafruit_BNO055 bno1, Adafruit_BNO055 bno2)
{
  adafruit_bno055_offsets_t calibrationData;
  calibrationData = CalibrationParam(-7,	36,	11,	-1,	0,	0,	213,	-9,	-123,	1000,	779);
  bno1.setSensorOffsets(calibrationData);
  delay(1000);
  bno1.getSensorOffsets(calibrationData);
  displaySensorOffsets(calibrationData, 1);
  String statusCal_bno1 = CalibrationStatus(bno1, 1);
  Serial.print("BNO1: "); Serial.println(statusCal_bno1);

  calibrationData = CalibrationParam(13,	-23,	1,	-1,	1,	-1,	-715,	-923,	-866,	1000,	760);
  bno2.setSensorOffsets(calibrationData);
  delay(1000);
  bno2.getSensorOffsets(calibrationData);
  displaySensorOffsets(calibrationData, 2);
  String statusCal_bno2 = CalibrationStatus(bno2, 2);
  Serial.print("BNO2: "); Serial.println(statusCal_bno2);
  delay(1000);

  // bool all_Calibrated = false;
  // while (!all_Calibrated)
  // {
  //   statusCal_bno1 = CalibrationStatus(bno1, 1);
  //   statusCal_bno2 = CalibrationStatus(bno2, 2);
  //   Serial.print("BNO1: "); Serial.print(statusCal_bno1);
  //   Serial.print(" BNO2: "); Serial.println(statusCal_bno2);
  //   if (bno1.isFullyCalibrated() && bno2.isFullyCalibrated())
  //   {
  //     all_Calibrated = true;
  //   }
  //   delay(100);
  // }
  bno1.getSensorOffsets(calibrationData);
  displaySensorOffsets(calibrationData, 1);

  bno2.getSensorOffsets(calibrationData);
  displaySensorOffsets(calibrationData, 2);

}

void setup(void)
{
  while (!Serial);
  delay(500);

  Serial.begin(9600);
  BLEsetup();
  Sensorsetup();
  Calibrationsetup(bno1, bno2);


}


//
void loop(void)
{
  new_time = millis();

  String status_bno1 = CalibrationStatus(bno1, 1);
  String status_bno2 = CalibrationStatus(bno2, 2);

  imu::Quaternion quat1 = bno1.getQuat();
  imu::Vector<3> gyro1 = bno1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> acce1 = bno1.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  imu::Quaternion quat2 = bno2.getQuat();
  imu::Vector<3> gyro2 = bno2.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> acce2 = bno2.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  quat1 = quat1.normalize();
  quat2 = quat2.normalize();

  int sign = 1;

  acce_1[0][0] = acce1.x();
  acce_1[0][1] = acce1.y();
  acce_1[0][2] = acce1.z();

  acce_2[0][0] = acce2.x();
  acce_2[0][1] = acce2.y();
  acce_2[0][2] = acce2.z();

  for (int j = 0; j < 3; j++)
  {
    // get the velocity
    vel_1[j][1] = vel_1[j][0] + acce_1[j][0] * interval_time * 0.001;
    vel_2[j][1] = vel_2[j][0] + acce_2[j][0] * interval_time * 0.001;
    // get the distance
    pos_1[j][1] = pos_1[j][0] + vel_1[j][0] * interval_time * 0.001;
    pos_2[j][1] = pos_2[j][0] + vel_2[j][0] * interval_time * 0.001;

    // get ready for the next set of readings
    acce_1[j][0] = acce_1[j][1];
    vel_1[j][0] = vel_1[j][1];
    pos_1[j][0] = pos_1[j][1];

    acce_2[j][0] = acce_2[j][1];
    vel_2[j][0] = vel_2[j][1];
    pos_2[j][0] = pos_2[j][1];
  }




  float product = (quat1.w() * quat2.w() - quat1.x() * quat2.x() - quat1.y() * quat2.y() - quat1.z() * quat2.z());
  float  angle = (acos((product * 2 - 1) * sign) * 57.2958);
  if (isnan(angle)) {
    sign = -1;
    angle = (acos(product * 2 - sign) * 57.2958) * sign;
  }


  int countDecimal = 2;
  String stringTwo = "";
  String seperator = " ";
  stringTwo = addStringPlus(gyro1, stringTwo, countDecimal, seperator);
  stringTwo = addStringPlus(acce1, stringTwo, countDecimal, seperator);
  stringTwo = addStringPlus(gyro2, stringTwo, countDecimal, seperator);
  stringTwo = addStringPlus(acce2, stringTwo, countDecimal, seperator);
  stringTwo = addString(status_bno1, stringTwo, seperator);
  stringTwo = addString(status_bno2, stringTwo, seperator);

  if (ble.isConnected()) {
    ble.print("AT+BLEUARTTX=");
    ble.print(stringTwo);
    ble.println("\\r\\n");
  }

  String pos_Data = "";
  seperator = ",";
  pos_Data = addStringArray(vel_1[0], 2, pos_Data, countDecimal, seperator);
  pos_Data = addStringArray(vel_2[0], 2, pos_Data, countDecimal, seperator);
  Serial.println(pos_Data);

  interval_time = new_time - old_time;
  old_time = new_time;
  // ble.println("AT+BLEUARTFIFO=TX");
  // ble.readline();
  // Serial.print("TX FIFO: ");
  // Serial.println(ble.buffer);


}
