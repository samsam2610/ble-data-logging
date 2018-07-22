#ifndef VARIABLES_H
#define VARIABLES_H

//BNO055 sensor objects
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, 0x29);
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

const int analogIn1 = A0;
const int analogIn3 = A1;

//Filtered objects
Butterworth_Filter acce_1 = Butterworth_Filter();
Butterworth_Filter acce_2 = Butterworth_Filter();

Velocity_Position pos_1 = Velocity_Position();
Velocity_Position pos_2 = Velocity_Position();

int Value1 = 0;        // value read from the pot
int Value2 =1000;
int Value3 = 0;        // value read from the pot
int Value4 =0;

unsigned long old_time;
unsigned long new_time;
unsigned interval_time;
unsigned wait_time;
int motion = 0;
imu::Quaternion offset;

uint32_t logtime;

// First IMU unit
imu::Vector<3> acce_1_filtered;
imu::Vector<3> pos_1_filtered;
double acce_1_mag;
double pos_1_mag;
double grav_1_max;

// Second IMU unit
imu::Vector<3> acce_2_filtered;
imu::Vector<3> pos_2_filtered;
double acce_2_mag;
double pos_2_mag;
double grav_2_max;

double a[2] = {0, 0};
double b[2] = {0, 0};

double a_lp[2] = {0, 0};
double b_lp[2] = {0, 0};

double a_hp[2] = {0, 0};
double b_hp[2] = {0, 0};

int multiplier = 1;

#endif
