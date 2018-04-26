#ifndef STRINGOPERATION_H
#define STRINGOPERATION_H

#include <string.h>

String addString(String value, String stringOriginal, String seperator)
{
  value += seperator;
  stringOriginal += value;
  return stringOriginal;
}

String addString_Vector( imu::Vector<3> dataIMU, String stringOriginal, int countDecimal, String seperator)
{
  String value = String( dataIMU.x(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  value = String( dataIMU.y(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  value = String( dataIMU.z(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  return stringOriginal;
}

String addString_Array(double array[], String stringOriginal, int countDecimal, int multipier, String seperator)
{
  for (int j = 0; j < 3; j++)
  {
    String value = String(array[j] * multiplier, countDecimal);
    stringOriginal = addString(value, stringOriginal, seperator);
  }
  return stringOriginal;
}

String addString_Quat( imu::Quaternion dataIMU, String stringOriginal, int countDecimal, String seperator)
{
  String value = String( dataIMU.w(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  value = String( dataIMU.x(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  value = String( dataIMU.y(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  value = String( dataIMU.z(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  return stringOriginal;
}

#endif
