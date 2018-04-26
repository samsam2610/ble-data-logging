#ifndef BUTTERWORTH_FILTER
#define BUTTERWORTH_FILTER

#include <imumaths.h>

class Butterworth_Filter
{
public:


  double a_lp[3];
  double b_lp[3];

  double a_hp[3];
  double b_hp[3];

  Butterworth_Filter();
  imu::Vector<3> butter_filter  ( imu::Vector<3> raw_new, imu::Vector<3> raw_old, imu::Vector<3> prev, double a[], double b[]  );
  void bandpass_filter  ( double a_lp[], double b_lp[], double a_hp[], double b_hp[] );
  void highpass_filter  ( double a_hp[], double b_hp[]  );
  void lowpass_filter ( double a_lp[], double b_lp[]  );
  void move_variables (  );
  void set_data(imu::Vector<3> data_new);
  imu::Vector<3> get_data_HP_filtered();
  imu::Vector<3> get_data_LP_filtered();
  imu::Vector<3> get_data_BP_filtered();

private:

  imu::Vector<3> data_new;
  imu::Vector<3> data_new_HP_filtered;
  imu::Vector<3> data_new_LP_filtered;

  imu::Vector<3> data_1st;
  imu::Vector<3> data_1st_HP_filtered;
  imu::Vector<3> data_1st_LP_filtered;
};

#endif
