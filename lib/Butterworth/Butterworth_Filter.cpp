#include <math.h>
#include <imumaths.h>

#include "Butterworth_Filter.h"

Butterworth_Filter::Butterworth_Filter()
{
  for (int j = 0; j < 3; j++)
  {
    data_new_HP_filtered[j] = 0;
    data_new_LP_filtered[j] = 0;
    data_1st[j] = 0;
    data_1st_HP_filtered[j] = 0;
    data_1st_LP_filtered[j] = 0;
  }
}

// Setter
void Butterworth_Filter::set_data(imu::Vector<3> data_new)
{
  for (int j = 0; j < 3; j++)
  {
    data_new[j] = data_new[j];
  }
}

//Getter
imu::Vector<3> Butterworth_Filter::get_data_HP_filtered()
{
  return data_new_HP_filtered;
}

imu::Vector<3> Butterworth_Filter::get_data_LP_filtered()
{
  return data_new_LP_filtered;
}


//Filter functions
imu::Vector<3> Butterworth_Filter::butter_filter(imu::Vector<3> raw_new, imu::Vector<3> raw_old, imu::Vector<3> prev, double a[], double b[])
{
  imu::Vector<3> output;
  for (int j = 0; j < 3; j++)
  {
    output[j] = b[0]*raw_new[j] + b[1]*raw_old[j] - a[1]*prev[j];
  }
  return output;
}

void Butterworth_Filter::bandpass_filter(double a_lp[], double b_lp[], double a_hp[], double b_hp[])
{
  data_new_HP_filtered = butter_filter(data_new, data_1st, data_1st_HP_filtered, a_hp, b_hp);
  data_new_LP_filtered = butter_filter(data_new_HP_filtered, data_1st_HP_filtered, data_1st_LP_filtered, a_lp, b_lp);
}

void Butterworth_Filter::highpass_filter(double a_hp[], double b_hp[])
{
  data_new_HP_filtered = butter_filter(data_new, data_1st, data_1st_HP_filtered, a_hp, b_hp);
}

void Butterworth_Filter::lowpass_filter(double a_lp[], double b_lp[])
{
  data_new_LP_filtered = butter_filter(data_new, data_1st, data_1st_LP_filtered, a_lp, b_lp);
}

void Butterworth_Filter::move_variables(imu::Vector<3> data_new, imu::Vector<3> data_new_HP_filtered, imu::Vector<3> data_new_LP_filtered)
{
  for (int j = 0; j < 3; j++)
  {
    data_1st[j] = data_new[j];
    data_1st_LP_filtered[j] = data_new_LP_filtered[j];
    data_1st_HP_filtered[j] = data_new_HP_filtered[j];
  }
}
