
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


double butter_filter(double raw_new, double raw_old, double prev, double a[], double b[])
{
  double output;
  output = b[0]*raw_new + b[1]*raw_old - a[1]*prev;
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

void setup(void)
{
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

  sensors_event_t bno1Event;
  sensors_event_t bno2Event;

  bno1.getEvent(&bno1Event);
  bno2.getEvent(&bno2Event);

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

  acce_1.set_data(acce1);
  acce_2.set_data(acce2);
  a_hp[0] = 1;
  a_hp[1] = -0.9999;
  b_hp[0] = 0.9999;
  b_hp[1] = -0.9999;

  a_lp[0] = 1;
  a_lp[1] = -0.5095;
  b_lp[0] = 0.2452;
  b_lp[1] = -0.2452;
  acce_1.bandpass_filter(a_lp, b_lp, a_hp, b_hp);
  acce_2.bandpass_filter(a_lp, b_lp, a_hp, b_hp);

  imu::Vector<3> acce_1_filtered = acce_1.get_data_BP_filtered();
  imu::Vector<3> acce_2_filtered = acce_2.get_data_BP_filtered();

  acce_1.move_variables();
  acce_2.move_variables();


  acce_1_mag = sqrt(pow(acce_1_filtered.x(), 2) + pow(acce_1_filtered.y(), 2) + pow(acce_1_filtered.z(), 2));
  acce_2_mag = sqrt(pow(acce_2_filtered.x(), 2) + pow(acce_2_filtered.y(), 2) + pow(acce_2_filtered.z(), 2));


  pos_1.set_data(acce1, interval_time, acce_1_mag, a_hp, b_hp);
  pos_2.set_data(acce2, interval_time, acce_2_mag, a_hp, b_hp);
  a_hp[0] = 1;
  a_hp[1] = -0.9752;
  b_hp[0] = 0.9876;
  b_hp[1] = -0.9876;
  pos_1.calculate_velocity();
  pos_1.calculate_position();
  pos_1.calculate_position_mag();
  imu::Vector<3> pos_1_filtered = pos_1.get_Position_HP_filtered();
  double pos_1_magg = pos_1.get_Position_mag();
  pos_1.move_variables();

  pos_2.calculate_velocity();
  pos_2.calculate_position();
  pos_2.calculate_position_mag();
  imu::Vector<3> pos_2_filtered = pos_2.get_Position_HP_filtered();
  double pos_2_magg = pos_1.get_Position_mag();
  pos_2.move_variables();

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

    a[0] = 1;
    a[1] = -0.9752;
    b[0] = 0.9876;
    b[1] = -0.9876;
    pos_1_new_HP_filtered[j] = butter_filter(pos_1_new[j], pos_1_old[j], pos_1_old_HP_filtered[j], a, b);
    pos_2_new_HP_filtered[j] = butter_filter(pos_2_new[j], pos_2_old[j], pos_2_old_HP_filtered[j], a, b);

    // get ready for the next set of readings
    vel_1_old[j] = vel_1_new[j];
    pos_1_old[j] = pos_1_new[j];
    pos_1_mag = pos_1_mag + pow(pos_1_new_HP_filtered[j], 2);
    pos_1_old_HP_filtered[j] = pos_1_new_HP_filtered[j];

    vel_2_old[j] = vel_2_new[j];
    pos_2_old[j] = pos_2_new[j];
    pos_2_mag = pos_2_mag + pow(pos_2_new_HP_filtered[j], 2);
    pos_2_old_HP_filtered[j] = pos_2_new_HP_filtered[j];
  }

  pos_1_mag = sqrt(pos_1_mag);
  pos_2_mag = sqrt(pos_2_mag);

  imu::Quaternion product = quat1 * quat2 * quat1.conjugate();
  double angle_w = angle_calculat(product.w());
  double angle_x = angle_calculat(product.x());
  double angle_y = angle_calculat(product.y());
  double angle_z = angle_calculat(product.z());

  int countDecimal = 2;
  String stringTwo = "";
  String seperator = ",";

  // stringTwo = addString_Vector(gyro1, stringTwo, countDecimal, seperator);
  // stringTwo = addString_Vector(acce1, stringTwo, countDecimal, seperator);
  // stringTwo = addString_Vector(gyro2, stringTwo, countDecimal, seperator);
  // stringTwo = addString_Vector(acce2, stringTwo, countDecimal, seperator);
  // stringTwo = addString(status_bno1, stringTwo, seperator);
  // stringTwo = addString(status_bno2, stringTwo, seperator);

  stringTwo = addString_Vector(pos_1_filtered, stringTwo, countDecimal, seperator);
  stringTwo = addString_Vector(pos_2_filtered, stringTwo, countDecimal, seperator);

  String pos_Data = "";
  seperator = ",";
  multiplier = 1;
  pos_Data = addString_Array(pos_1_new_HP_filtered, pos_Data, countDecimal, multiplier, seperator);
  pos_Data = addString_Array(pos_2_new_HP_filtered, pos_Data, countDecimal, multiplier, seperator);
  Serial.print("New: ");
  Serial.print(stringTwo);
  Serial.print("Old: ");
  Serial.println(pos_Data);

  String angle_data = "";
  angle_data = addString(String(angle_w, 0), angle_data, seperator);
  angle_data = addString(String(angle_x, 0), angle_data, seperator);
  angle_data = addString(String(angle_y, 0), angle_data, seperator);
  angle_data = addString(String(angle_z, 0), angle_data, seperator);
  angle_data = addString(String(angle_z, 0), angle_data, seperator);
  angle_data = addString(String(pos_1_mag*multiplier, 0), angle_data, seperator);
  angle_data = addString(String(bno2Event.orientation.y, 0), angle_data, seperator);
  angle_data = addString(String(interval_time), angle_data, seperator);




  if (ble.isConnected()) {
    ble.print("AT+BLEUARTTX=");
    ble.print("s,");
    ble.print(angle_data);
    ble.print("e");
    ble.println("\\r\\n");
  }
  interval_time = new_time - old_time;
  delay(100);
  old_time = new_time;
}
