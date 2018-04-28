#include <math.h>
#include <imumaths.h>

#include "Butterworth_Filter.h"
#include "Velocity_Position.h"

Velocity_Position::Velocity_Position()
{
  for (int j = 0; j < 3; j++)
  {
    Velocity_1st[j] = 0;
    Position_new[j] = 0;
    Position_1st[j] = 0;
    Position_1st_filtered[j] = 0;
  }
  Butterworth_Filter Position = Butterworth_Filter();
}

// Setter;
void Velocity_Position::set_data(imu::Vector<3> _accel, unsigned _interval_time, double _accel_mag, double _a_hp[], double _b_hp[])
{
  interval_time = _interval_time;
  accel_mag = _accel_mag;
  for (int j = 0; j < 3; j++)
  {
    accel[j] = _accel[j];
    a_hp[j] = _a_hp[j];
    b_hp[j] = _b_hp[j];
  }
}
// Getter;
imu::Vector<3> Velocity_Position::get_Position_HP_filtered()
{
  return Position_new_filtered;
}

double Velocity_Position::get_Position_mag()
{
  return Position_mag;
}

void Velocity_Position::calculate_velocity()
{
  for (int j = 0; j < 3; j++)
  {
    Velocity_new[j] = Velocity_1st[j] + accel[j] * interval_time * 0.001;
    if (accel_mag < 0.05)
    {
      Velocity_new[j] = 0;
    }
  }
}

void Velocity_Position::calculate_position()
{
  for (int j = 0; j < 3; j++)
  {
    Position_new[j] = Position_1st[j] + Velocity_new[j] * interval_time * 0.001;
  }
  Position.set_data(Position_new);
  Position.highpass_filter(a_hp, b_hp);
  Position_new_filtered = Position.get_data_HP_filtered();
}

void Velocity_Position::calculate_position_mag()
{

  Position_mag_temp[0] = 0;
  for (int j = 0; j < 3; j++)
  {
    Position_mag_temp[0] = Position_mag_temp[0] + Position_new[j]*Position_new[j];
  }
  Position_mag = double(Position_mag_temp[0]);
}

void Velocity_Position::move_variables()
{
  for (int j = 0; j < 3; j++)
  {
    Velocity_1st[j] = Velocity_new[j];
    Position_1st[j] = Position_new[j];
    Position_1st_filtered[j] = Position_new_filtered[j];
  }
}
