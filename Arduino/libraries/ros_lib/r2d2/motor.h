#ifndef _ROS_r2d2_motor_h
#define _ROS_r2d2_motor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2d2
{

  class motor : public ros::Msg
  {
    public:
      typedef int16_t _forward_type;
      _forward_type forward;
      typedef int16_t _backward_type;
      _backward_type backward;
      typedef int16_t _left_type;
      _left_type left;
      typedef int16_t _right_type;
      _right_type right;

    motor():
      forward(0),
      backward(0),
      left(0),
      right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_forward;
      u_forward.real = this->forward;
      *(outbuffer + offset + 0) = (u_forward.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_forward.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->forward);
      union {
        int16_t real;
        uint16_t base;
      } u_backward;
      u_backward.real = this->backward;
      *(outbuffer + offset + 0) = (u_backward.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_backward.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->backward);
      union {
        int16_t real;
        uint16_t base;
      } u_left;
      u_left.real = this->left;
      *(outbuffer + offset + 0) = (u_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->left);
      union {
        int16_t real;
        uint16_t base;
      } u_right;
      u_right.real = this->right;
      *(outbuffer + offset + 0) = (u_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_forward;
      u_forward.base = 0;
      u_forward.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_forward.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->forward = u_forward.real;
      offset += sizeof(this->forward);
      union {
        int16_t real;
        uint16_t base;
      } u_backward;
      u_backward.base = 0;
      u_backward.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_backward.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->backward = u_backward.real;
      offset += sizeof(this->backward);
      union {
        int16_t real;
        uint16_t base;
      } u_left;
      u_left.base = 0;
      u_left.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->left = u_left.real;
      offset += sizeof(this->left);
      union {
        int16_t real;
        uint16_t base;
      } u_right;
      u_right.base = 0;
      u_right.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->right = u_right.real;
      offset += sizeof(this->right);
     return offset;
    }

    const char * getType(){ return "r2d2/motor"; };
    const char * getMD5(){ return "87db3c7725f33767861f8c2a54c4b9f6"; };

  };

}
#endif