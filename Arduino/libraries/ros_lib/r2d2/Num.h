#ifndef _ROS_r2d2_Num_h
#define _ROS_r2d2_Num_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace r2d2
{

  class Num : public ros::Msg
  {
    public:
      typedef float _num_type;
      _num_type num;

    Num():
      num(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->num);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->num));
     return offset;
    }

    const char * getType(){ return "r2d2/Num"; };
    const char * getMD5(){ return "f942a3e0cbd340847b2a0e5b31a783fe"; };

  };

}
#endif