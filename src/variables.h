#ifndef VARIABLES_H
#define VARIABLES_H

//BNO055 sensor objects
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x29);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, 0x28);
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

//Filtered objects
Butterworth_Filter acce_1_filtered = Butterworth_Filter();
Butterworth_Filter acce_2_filtered = Butterworth_Filter();
unsigned long old_time;
unsigned long new_time;
unsigned interval_time;
uint32_t logtime;

// First IMU unit
double acce_1_mat[4] = {0, 0, 0, 0};
double acce_1_mag = 0;

double acce_1_new[3] = {0, 0, 0};
double acce_1_new_LP_filtered[3] = {0, 0, 0};
double acce_1_new_HP_filtered[3] = {0, 0, 0};

double vel_1_new[3] = {0, 0, 0};
double pos_1_new[3] = {0, 0, 0};
double pos_1_new_HP_filtered[3] = {0, 0, 0};
double pos_1_mag = 0;

double acce_1_old[3] = {0, 0, 0};
double acce_1_old_LP_filtered[3] = {0, 0, 0};
double acce_1_old_HP_filtered[3] = {0, 0, 0};

double vel_1_old[3] = {0, 0, 0};

double pos_1_old[3] = {0, 0, 0};
double pos_1_old_HP_filtered[3] = {0, 0, 0};

// Second IMU unit
double acce_2_mat[4] = {0, 0, 0, 0};
double acce_2_mag = 0;
double acce_2_new[3] = {0, 0, 0};
double acce_2_new_LP_filtered[3] = {0, 0, 0};
double acce_2_new_HP_filtered[3] = {0, 0, 0};

double vel_2_new[3] = {0, 0, 0};
double pos_2_new[3] = {0, 0, 0};
double pos_2_new_HP_filtered[3] = {0, 0, 0};

double acce_2_old[3] = {0, 0, 0};
double acce_2_old_LP_filtered[3] = {0, 0, 0};
double acce_2_old_HP_filtered[3] = {0, 0, 0};

double vel_2_old[3] = {0, 0, 0};
double pos_2_old[3] = {0, 0, 0};
double pos_2_old_HP_filtered[3] = {0, 0, 0};
double pos_2_mag = 0;

double a[2] = {0, 0};
double b[2] = {0, 0};

int multiplier = 1;

#endif
