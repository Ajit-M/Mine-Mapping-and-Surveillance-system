#ifndef _ROS_r2d2_custom_h
#define _ROS_r2d2_custom_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2d2
{

  class custom : public ros::Msg
  {
    public:
      typedef int16_t _humidity_type;
      _humidity_type humidity;
      typedef int16_t _temprature_type;
      _temprature_type temprature;
      typedef float _distance_1_type;
      _distance_1_type distance_1;
      typedef float _distance_2_type;
      _distance_2_type distance_2;

    custom():
      humidity(0),
      temprature(0),
      distance_1(0),
      distance_2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_humidity;
      u_humidity.real = this->humidity;
      *(outbuffer + offset + 0) = (u_humidity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_humidity.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->humidity);
      union {
        int16_t real;
        uint16_t base;
      } u_temprature;
      u_temprature.real = this->temprature;
      *(outbuffer + offset + 0) = (u_temprature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temprature.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->temprature);
      union {
        float real;
        uint32_t base;
      } u_distance_1;
      u_distance_1.real = this->distance_1;
      *(outbuffer + offset + 0) = (u_distance_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance_1);
      union {
        float real;
        uint32_t base;
      } u_distance_2;
      u_distance_2.real = this->distance_2;
      *(outbuffer + offset + 0) = (u_distance_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance_2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_humidity;
      u_humidity.base = 0;
      u_humidity.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_humidity.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->humidity = u_humidity.real;
      offset += sizeof(this->humidity);
      union {
        int16_t real;
        uint16_t base;
      } u_temprature;
      u_temprature.base = 0;
      u_temprature.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temprature.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->temprature = u_temprature.real;
      offset += sizeof(this->temprature);
      union {
        float real;
        uint32_t base;
      } u_distance_1;
      u_distance_1.base = 0;
      u_distance_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance_1 = u_distance_1.real;
      offset += sizeof(this->distance_1);
      union {
        float real;
        uint32_t base;
      } u_distance_2;
      u_distance_2.base = 0;
      u_distance_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance_2 = u_distance_2.real;
      offset += sizeof(this->distance_2);
     return offset;
    }

    const char * getType(){ return "r2d2/custom"; };
    const char * getMD5(){ return "1c3d1ac516fb624064e660eea5fbaecc"; };

  };

}
#endif