#ifndef _ROS_gigatron_msgs_Steering_h
#define _ROS_gigatron_msgs_Steering_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace gigatron_msgs
{

  class Steering : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _angle_command_type;
      _angle_command_type angle_command;
      typedef uint8_t _angle_type;
      _angle_type angle;

    Steering():
      header(),
      angle_command(0),
      angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->angle_command >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angle_command);
      *(outbuffer + offset + 0) = (this->angle >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->angle_command =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->angle_command);
      this->angle =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->angle);
     return offset;
    }

    const char * getType(){ return "gigatron_msgs/Steering"; };
    const char * getMD5(){ return "d462d94c1b7e973a58a35ad5ef9c54b0"; };

  };

}
#endif
