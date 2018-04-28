#ifndef VELOCITY_POSITION
#define VELOCITY_POSITION

#include <imumaths.h>
#include <Butterworth_Filter.h>

class Velocity_Position
{
public:
  double a_lp[3];
  double b_lp[3];

  double a_hp[3];
  double b_hp[3];

  Velocity_Position();
  void set_data ( imu::Vector<3> _accel, unsigned _interval_time, double _accel_mag );
  void set_filter_coeff ( double _a_hp[], double _b_hp[] );
  void set_multiplier ( int _multiplier);
  imu::Vector<3> get_Position_HP_filtered();
  double get_Position_mag ();
  void calculate_velocity  ();
  void calculate_position ();
  void calculate_position_mag ();
  void move_variables();


private:
  Butterworth_Filter Position;
  Butterworth_Filter Velocity;

  imu::Vector<3> accel;
  double accel_mag;

  imu::Vector<3> Velocity_new;
  imu::Vector<3> Velocity_new_filtered;
  imu::Vector<3> Velocity_1st;
  imu::Vector<3> Velocity_1st_filtered;

  imu::Vector<3> Position_new;
  imu::Vector<3> Position_new_filtered;
  imu::Vector<3> Position_1st;
  imu::Vector<3> Position_1st_filtered;

  imu::Vector<1> Position_mag_temp;
  double Position_mag;
  unsigned interval_time;
  int multiplier;


};

#endif
