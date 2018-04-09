#ifndef STRINGOPERATION_H
#define STRINGOPERATION_H

#include <string.h>

String addString(String value, String stringOriginal, String separator)
{
  value += separator;
  stringOriginal += value;
  return stringOriginal;
}

String addStringPlus( imu::Vector<3> dataIMU, String stringOriginal, int countDecimal, String seperator)
{
  String value = String( dataIMU.x(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  value = String( dataIMU.y(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  value = String( dataIMU.z(), countDecimal);
  stringOriginal = addString(value, stringOriginal, seperator);
  return stringOriginal;
}

String addStringArray(double array[], String stringOriginal, int countDecimal, int multipier, String seperator)
{
  for (int j = 0; j < 3; j++)
  {
    String value = String(array[j] * multiplier, countDecimal);
    stringOriginal = addString(value, stringOriginal, seperator);
  }
  return stringOriginal;
}

#endif
