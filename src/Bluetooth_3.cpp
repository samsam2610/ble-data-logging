
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

#include "variables.h"
#include "stringOperation.h"
#include "calibration.h"
#include "setup.h"

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"


double butter_filter(double raw_new, double raw_old, double prev, double a[], double b[])
{
  double output;
  output = b[0]*raw_new + b[1]*raw_old - a[1]*prev;
  return output;
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

  acce1 = rotate_data(acce1, quat1);
  acce2 = rotate_data(acce2, quat2);
  acce_1_new[0] = double(acce1.x());
  acce_1_new[1] = double(acce1.y());
  acce_1_new[2] = double(acce1.z());

  acce_2_new[0] = double(acce2.x());
  acce_2_new[1] = double(acce2.y());
  acce_2_new[2] = double(acce2.z());

  for (int j = 0; j < 3; j++)
  {
    //high-pass filter acceleration data
    a[0] = 1;
    a[1] = -0.9999;
    b[0] = 0.9999;
    b[1] = -0.9999;
    acce_1_new_HP_filtered[j] = butter_filter(acce_1_new[j], acce_1_old[j], acce_1_old_HP_filtered[j], a, b);
    acce_2_new_HP_filtered[j] = butter_filter(acce_2_new[j], acce_2_old[j], acce_2_old_HP_filtered[j], a, b);

    //low-pass filter acceeleration data
    a[0] = 1;
    a[1] = -0.5095;
    b[0] = 0.2452;
    b[1] = -0.2452;
    acce_1_new_LP_filtered[j] = butter_filter(acce_1_new_HP_filtered[j], acce_1_old_HP_filtered[j], acce_1_old_LP_filtered[j], a, b);
    acce_2_new_LP_filtered[j] = butter_filter(acce_2_new_HP_filtered[j], acce_2_old_HP_filtered[j], acce_2_old_LP_filtered[j], a, b);

    acce_1_mag = sqrt(pow(acce_1_new_LP_filtered[0], 2) + pow(acce_1_new_LP_filtered[1], 2) + pow(acce_1_new_LP_filtered[2], 2));
    acce_2_mag = sqrt(pow(acce_2_new_LP_filtered[0], 2) + pow(acce_2_new_LP_filtered[1], 2) + pow(acce_2_new_LP_filtered[2], 2));
  }

  for (int j = 0; j < 3; j++)
  {
    // get the velocity
    vel_1_new[j] = vel_1_old[j] + acce_1_new[j] * interval_time * 0.001;
    vel_2_new[j] = vel_1_old[j] + acce_2_new[j] * interval_time * 0.001;
    if (acce_1_mag < 0.05)
    {
      vel_1_new[j] = 0;
    }

    if (acce_2_mag < 0.05)
    {
      vel_2_new[j] = 0;
    }

    // get the distance
    pos_1_new[j] = pos_1_old[j] + vel_1_new[j] * interval_time * 0.001;
    pos_2_new[j] = pos_2_old[j] + vel_2_new[j] * interval_time * 0.001;

    // get ready for the next set of readings
    acce_1_old[j] = acce_1_new[j];
    acce_1_old_HP_filtered[j] = acce_1_new_HP_filtered[j];
    acce_1_old_LP_filtered[j] = acce_1_new_LP_filtered[j];
    vel_1_old[j] = vel_1_new[j];
    pos_1_old[j] = pos_1_new[j];

    acce_2_old[j] = acce_2_new[j];
    acce_2_old_HP_filtered[j] = acce_2_new_HP_filtered[j];
    acce_2_old_LP_filtered[j] = acce_2_new_LP_filtered[j];
    vel_2_old[j] = vel_2_new[j];
    pos_2_old[j] = pos_2_new[j];
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
  seperator = ", ";
  multiplier = 100;
  pos_Data = addStringArray(pos_1_new, pos_Data, countDecimal, multiplier, seperator);
  pos_Data = addStringArray(pos_2_new, pos_Data, countDecimal, multiplier, seperator);
  Serial.println(pos_Data);

  interval_time = new_time - old_time;
  old_time = new_time;
  // ble.println("AT+BLEUARTFIFO=TX");
  // ble.readline();
  // Serial.print("TX FIFO: ");
  // Serial.println(ble.buffer);


}
