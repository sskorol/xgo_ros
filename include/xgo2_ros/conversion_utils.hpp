#ifndef __CONVERSION_UTILS_HPP
#define __CONVERSION_UTILS_HPP

#include <cstdint>

float limit(float data, float min_limit, float max_limit)
{
  return data < min_limit ? min_limit : data > max_limit ? max_limit : data;
}

float uint82float(uint8_t data, float min_limit, float max_limit)
{
  return (float)data / 255.0 * (max_limit - min_limit) + min_limit;
}

uint8_t float2uint8(float data, float min_limit, float max_limit)
{
  return (uint8_t)((data - min_limit) / (max_limit - min_limit) * 255);
}

float bytes2float(uint8_t* databytes)
{
  return *(float*)(databytes);
}

float toRad(float value)
{
  return value / 57.3;
}

#endif
