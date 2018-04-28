
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
  imu::Vector<3> grav1 = bno1.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  imu::Quaternion quat2 = bno2.getQuat();
  imu::Vector<3> gyro2 = bno2.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> acce2 = bno2.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> grav2 = bno2.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  sensors_event_t bno1Event;
  sensors_event_t bno2Event;

  bno1.getEvent(&bno1Event);
  bno2.getEvent(&bno2Event);

  quat1 = quat1.normalize();
  quat2 = quat2.normalize();

  int sign = 1;

  acce1 = rotate_data(acce1, quat1);
  acce2 = rotate_data(acce2, quat2);

  // acce_1.set_data(acce1);
  // acce_2.set_data(acce2);
  // a_hp[0] = 1;
  // a_hp[1] = -0.9997;
  // b_hp[0] = 0.9999;
  // b_hp[1] = -0.9999;
  //
  //
  // a_lp[0] = 1;
  // a_lp[1] = -0.1584;
  // b_lp[0] = 0.5792;
  // b_lp[1] = -0.5792;
  // acce_1.bandpass_filter(a_lp, b_lp, a_hp, b_hp);
  // acce_2.bandpass_filter(a_lp, b_lp, a_hp, b_hp);
  //
  // imu::Vector<3> acce_1_filtered = acce_1.get_data_BP_filtered();
  // imu::Vector<3> acce_2_filtered = acce_2.get_data_BP_filtered();
  //
  // acce_1.move_variables();
  // acce_2.move_variables();


  acce_1_mag = sqrt(pow(acce1.x(), 2) + pow(acce1.y(), 2) + pow(acce1.z(), 2));
  acce_2_mag = sqrt(pow(acce2.x(), 2) + pow(acce2.y(), 2) + pow(acce2.z(), 2));

  // a_hp[0] = 1;
  // a_hp[1] = -0.9752;
  // b_hp[0] = 0.9876;
  // b_hp[1] = -0.9876;

  // pos_1.set_data(acce1, interval_time, acce_1_mag);
  // pos_1.set_filter_coeff(a_hp, b_hp);
  // pos_1.set_multiplier(1);
  // pos_1.calculate_velocity();
  // pos_1.calculate_position();
  // pos_1.calculate_position_mag();
  // pos_1_filtered = pos_1.get_Position_HP_filtered();
  // pos_1_mag = pos_1.get_Position_mag();
  // pos_1.move_variables();
  //
  // pos_2.set_data(acce2, interval_time, acce_2_mag);
  // pos_2.set_filter_coeff(a_hp, b_hp);
  // pos_2.set_multiplier(1);
  // pos_2.calculate_velocity();
  // pos_2.calculate_position();
  // pos_2.calculate_position_mag();
  // pos_2_filtered = pos_2.get_Position_HP_filtered();
  // pos_2_mag = pos_1.get_Position_mag();
  // pos_2.move_variables();


  imu::Quaternion product = quat2 * quat1.conjugate();
  imu::Quaternion angle = angle_calculate(product);


  if (acce_1_mag < 0.5 && acce_2_mag < 0.5)
  {
    wait_time = wait_time + interval_time;
    if (wait_time*0.001 > 5)
    {
      offset.w() = angle.w();
      offset.x() = angle.x();
      offset.y() = angle.y();
      offset.z() = angle.z();
    }
  } else {
    wait_time = 0;
  }


  imu::Quaternion angle_w_offset;
  angle_w_offset.w() = angle.w() - offset.w();
  angle_w_offset.x() = angle.x() - offset.x();
  angle_w_offset.y() = angle.y() - offset.y();
  angle_w_offset.z() = angle.z() - offset.z();

  // int countDecimal = 2;
  // String stringTwo = "";
  String seperator = ",";
  //
  // // stringTwo = addString_Vector(gyro1, stringTwo, countDecimal, seperator);
  // // stringTwo = addString_Vector(acce1, stringTwo, countDecimal, seperator);
  // // stringTwo = addString_Vector(gyro2, stringTwo, countDecimal, seperator);
  // // stringTwo = addString_Vector(acce2, stringTwo, countDecimal, seperator);
  // // stringTwo = addString(status_bno1, stringTwo, seperator);
  // // stringTwo = addString(status_bno2, stringTwo, seperator);
  //
  // stringTwo = addString_Vector(grav1, stringTwo, countDecimal, seperator);
  // stringTwo = addString_Vector(grav2, stringTwo, countDecimal, seperator);
  //
  // String pos_Data = "";
  seperator = ",";
  multiplier = 1;
  // pos_Data = addString_Array(pos_1_new_HP_filtered, pos_Data, countDecimal, multiplier, seperator);
  // pos_Data = addString_Array(pos_2_new_HP_filtered, pos_Data, countDecimal, multiplier, seperator);

  String angle_data = "";
  angle_data = addString(String(angle_w_offset.w(), 0), angle_data, seperator);
  angle_data = addString(String(angle_w_offset.x(), 0), angle_data, seperator);
  angle_data = addString(String(angle_w_offset.y(), 0), angle_data, seperator);
  angle_data = addString(String(angle_w_offset.z(), 0), angle_data, seperator);
  //angle_data = addString(String(pos_1_mag*multiplier, 0), angle_data, seperator);
//  angle_data = addString(String(bno2Event.orientation.y, 0), angle_data, seperator);
  //angle_data = addString(String(interval_time), angle_data, seperator);


  if (interval_time < 200)
  {
    if (ble.isConnected()) {
      ble.print("AT+BLEUARTTX=");
      ble.print(angle_data);
      ble.println("\\r\\n");
    }
  }

  interval_time = new_time - old_time;
  old_time = new_time;
  delay(150);
}
