#ifndef _ROS_gigatron_msgs_Radio_h
#define _ROS_gigatron_msgs_Radio_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace gigatron_msgs
{

  class Radio : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _speed_left_type;
      _speed_left_type speed_left;
      typedef uint8_t _speed_right_type;
      _speed_right_type speed_right;
      typedef uint8_t _angle_type;
      _angle_type angle;
      typedef uint8_t _kill_type;
      _kill_type kill;

    Radio():
      header(),
      speed_left(0),
      speed_right(0),
      angle(0),
      kill(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->speed_left >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed_left);
      *(outbuffer + offset + 0) = (this->speed_right >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed_right);
      *(outbuffer + offset + 0) = (this->angle >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angle);
      *(outbuffer + offset + 0) = (this->kill >> (8 * 0)) & 0xFF;
      offset += sizeof(this->kill);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->speed_left =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->speed_left);
      this->speed_right =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->speed_right);
      this->angle =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->angle);
      this->kill =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->kill);
     return offset;
    }

    const char * getType(){ return "gigatron_msgs/Radio"; };
    const char * getMD5(){ return "179f0a81e231566cfe9870e01763e531"; };

  };

}
#endif
