#ifndef CALIBRATION_H
#define CALIBRATION_H

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

  bno1.getSensorOffsets(calibrationData);
  displaySensorOffsets(calibrationData, 1);

  bno2.getSensorOffsets(calibrationData);
  displaySensorOffsets(calibrationData, 2);

}

#endif
