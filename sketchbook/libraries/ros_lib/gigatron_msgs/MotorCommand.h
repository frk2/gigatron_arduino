#ifndef _ROS_gigatron_msgs_MotorCommand_h
#define _ROS_gigatron_msgs_MotorCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace gigatron_msgs
{

  class MotorCommand : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _angle_command_type;
      _angle_command_type angle_command;
      typedef int16_t _rpm_left_type;
      _rpm_left_type rpm_left;
      typedef int16_t _rpm_right_type;
      _rpm_right_type rpm_right;

    MotorCommand():
      header(),
      angle_command(0),
      rpm_left(0),
      rpm_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->angle_command >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angle_command);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_left;
      u_rpm_left.real = this->rpm_left;
      *(outbuffer + offset + 0) = (u_rpm_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_left.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rpm_left);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_right;
      u_rpm_right.real = this->rpm_right;
      *(outbuffer + offset + 0) = (u_rpm_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_right.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rpm_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->angle_command =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->angle_command);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_left;
      u_rpm_left.base = 0;
      u_rpm_left.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_left.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rpm_left = u_rpm_left.real;
      offset += sizeof(this->rpm_left);
      union {
        int16_t real;
        uint16_t base;
      } u_rpm_right;
      u_rpm_right.base = 0;
      u_rpm_right.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_right.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rpm_right = u_rpm_right.real;
      offset += sizeof(this->rpm_right);
     return offset;
    }

    const char * getType(){ return "gigatron_msgs/MotorCommand"; };
    const char * getMD5(){ return "5ac1e902a62229dcc56b81bef1a595fa"; };

  };

}
#endif
