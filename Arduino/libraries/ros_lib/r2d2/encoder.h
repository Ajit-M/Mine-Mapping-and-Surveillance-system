#ifndef _ROS_r2d2_encoder_h
#define _ROS_r2d2_encoder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2d2
{

  class encoder : public ros::Msg
  {
    public:
      typedef float _distance_LF_type;
      _distance_LF_type distance_LF;
      typedef float _distance_LR_type;
      _distance_LR_type distance_LR;
      typedef float _distance_RF_type;
      _distance_RF_type distance_RF;
      typedef float _distance_RR_type;
      _distance_RR_type distance_RR;

    encoder():
      distance_LF(0),
      distance_LR(0),
      distance_RF(0),
      distance_RR(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_distance_LF;
      u_distance_LF.real = this->distance_LF;
      *(outbuffer + offset + 0) = (u_distance_LF.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance_LF.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance_LF.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance_LF.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance_LF);
      union {
        float real;
        uint32_t base;
      } u_distance_LR;
      u_distance_LR.real = this->distance_LR;
      *(outbuffer + offset + 0) = (u_distance_LR.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance_LR.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance_LR.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance_LR.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance_LR);
      union {
        float real;
        uint32_t base;
      } u_distance_RF;
      u_distance_RF.real = this->distance_RF;
      *(outbuffer + offset + 0) = (u_distance_RF.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance_RF.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance_RF.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance_RF.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance_RF);
      union {
        float real;
        uint32_t base;
      } u_distance_RR;
      u_distance_RR.real = this->distance_RR;
      *(outbuffer + offset + 0) = (u_distance_RR.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance_RR.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance_RR.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance_RR.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance_RR);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_distance_LF;
      u_distance_LF.base = 0;
      u_distance_LF.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance_LF.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance_LF.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance_LF.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance_LF = u_distance_LF.real;
      offset += sizeof(this->distance_LF);
      union {
        float real;
        uint32_t base;
      } u_distance_LR;
      u_distance_LR.base = 0;
      u_distance_LR.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance_LR.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance_LR.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance_LR.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance_LR = u_distance_LR.real;
      offset += sizeof(this->distance_LR);
      union {
        float real;
        uint32_t base;
      } u_distance_RF;
      u_distance_RF.base = 0;
      u_distance_RF.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance_RF.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance_RF.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance_RF.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance_RF = u_distance_RF.real;
      offset += sizeof(this->distance_RF);
      union {
        float real;
        uint32_t base;
      } u_distance_RR;
      u_distance_RR.base = 0;
      u_distance_RR.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance_RR.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance_RR.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance_RR.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance_RR = u_distance_RR.real;
      offset += sizeof(this->distance_RR);
     return offset;
    }

    const char * getType(){ return "r2d2/encoder"; };
    const char * getMD5(){ return "8daba27cc7ea9c30ca6c6a6a0e1c4cd5"; };

  };

}
#endif