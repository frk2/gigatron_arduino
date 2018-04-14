#ifndef _ROS_gigatron_Drive_h
#define _ROS_gigatron_Drive_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gigatron
{

  class Drive : public ros::Msg
  {
    public:
      typedef float _angle_type;
      _angle_type angle;
      typedef float _vel_left_type;
      _vel_left_type vel_left;
      typedef float _vel_right_type;
      _vel_right_type vel_right;

    Drive():
      angle(0),
      vel_left(0),
      vel_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->vel_left);
      offset += serializeAvrFloat64(outbuffer + offset, this->vel_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vel_left));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vel_right));
     return offset;
    }

    const char * getType(){ return "gigatron/Drive"; };
    const char * getMD5(){ return "a9e040342a3774f24754cba3bba91082"; };

  };

}
#endif