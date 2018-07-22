
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <string.h>
#include "Adafruit_BNO055.h"
#include "imumaths.h"

#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

#include "Butterworth_Filter.h"
#include "Velocity_Position.h"
#include "variables.h"
#include "stringOperation.h"
#include "calibration.h"
#include "setup.h"


#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

imu::Quaternion angle_calculate(imu::Quaternion product)
{
  imu::Quaternion output;
  output.w() = 2 * acos(product.w()) * 57.2958;
  output.x() = 2 * acos(product.x()) * 57.2958;
  output.y() = 2 * acos(product.y()) * 57.2958;
  output.z() = 2 * acos(product.z()) * 57.2958;
  return output;
}

double angle_calculat(double raw_data)
{
  double angle;
  angle = (2 * acos(raw_data) * 57.2958);
  return angle;
}

imu::Vector<3> rotate_data(imu::Vector<3> vec, imu::Quaternion quat)
{
  imu::Quaternion vec_quat;
  imu::Quaternion temp_quat(0, vec.x(), vec.y(), vec.z());
  vec_quat = quat * temp_quat;
  vec_quat = vec_quat * quat.conjugate();
  imu::Vector<3> out_vec(vec_quat.x(), vec_quat.y(), vec_quat.z());
  return out_vec;
}

double absolute_angle(imu::Vector<3> vec)
{
  double abs_max = 0;
  double x = double(abs(vec.x()));
  double y = double(abs(vec.y()));
  double z = double(abs(vec.z()));

  if (abs_max > x)
  {
    abs_max = x;
  }

  if (abs_max > y)
  {
    abs_max = y;
  }

  if (abs_max > z)
  {
    abs_max = z;
  }

  //double abs_angle = 9.81/abs_max;
  return abs_max;
}


void setup(void)
{
  delay(500);
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  Sensorsetup();
  Calibrationsetup(bno1, bno2);
  BLEsetup();
  CalibrationStatus(bno1, 1);
}

//
void loop(void)
{
  new_time = millis();

  Value1 = analogRead(analogIn1);
  Value3 = analogRead(analogIn3);

  imu::Quaternion quat1 = bno1.getQuat();
  imu::Vector<3> gyro1 = bno1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> acce1 = bno1.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> grav1 = bno1.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  imu::Quaternion quat2 = bno2.getQuat();
  imu::Vector<3> gyro2 = bno2.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> acce2 = bno2.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> grav2 = bno2.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  sensors_event_t bno1Event;
  sensors_event_t bno2Event;

  bno1.getEvent(&bno1Event);
  bno2.getEvent(&bno2Event);

  if (CalibrationStatus(bno1, 1) == CalibrationStatus(bno2, 1) && CalibrationStatus(bno1, 1) == "13333")
  {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  quat1 = quat1.normalize();
  quat2 = quat2.normalize();

  acce1 = rotate_data(acce1, quat1);
  acce2 = rotate_data(acce2, quat2);

  grav_1_max = absolute_angle(grav1);
  grav_2_max = absolute_angle(grav2);

  acce_1_mag = sqrt(pow(acce1.x(), 2) + pow(acce1.y(), 2) + pow(acce1.z(), 2));
  acce_2_mag = sqrt(pow(acce2.x(), 2) + pow(acce2.y(), 2) + pow(acce2.z(), 2));

  imu::Quaternion product = quat2 * quat1.conjugate();
  imu::Quaternion angle = angle_calculate(product);

  String seperator = ",";

  seperator = ",";
  multiplier = 1;

  String angle_data = "";
  angle_data = addString("s", angle_data, seperator);
  angle_data = addString(String(angle.w(), 0), angle_data, seperator);
  angle_data = addString(String(angle.x(), 0), angle_data, seperator);
  angle_data = addString(String(angle.y(), 0), angle_data, seperator);
  angle_data = addString(String(angle.z(), 0), angle_data, seperator);
  angle_data = addString(String(acce1.x(), 2), angle_data, seperator);
  angle_data = addString(String(acce1.y(), 2), angle_data, seperator);
  angle_data = addString(String(acce1.z(), 2), angle_data, seperator);
  angle_data = addString(String(acce2.x(), 2), angle_data, seperator);
  angle_data = addString(String(acce2.y(), 2), angle_data, seperator);
  angle_data = addString(String(acce2.z(), 2), angle_data, seperator);
  angle_data = addString(String(gyro1.x(), 2), angle_data, seperator);
  angle_data = addString(String(gyro1.y(), 2), angle_data, seperator);
  angle_data = addString(String(gyro1.z(), 2), angle_data, seperator);
  angle_data = addString(String(gyro2.x(), 2), angle_data, seperator);
  angle_data = addString(String(gyro2.y(), 2), angle_data, seperator);
  angle_data = addString(String(gyro2.z(), 2), angle_data, seperator);
  angle_data = addString(String(Value1), angle_data, seperator);
  angle_data = addString(String(Value3), angle_data, seperator);
  angle_data = addString(String(interval_time), angle_data, seperator);
  angle_data = addString("e", angle_data, "");


  if (ble.isConnected())
  {
    ble.println(angle_data);
  }

  Serial.println(angle_data);

  interval_time = new_time - old_time;
  old_time = new_time;
}
