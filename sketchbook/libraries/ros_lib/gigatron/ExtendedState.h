#ifndef _ROS_gigatron_ExtendedState_h
#define _ROS_gigatron_ExtendedState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "gigatron/Drive.h"

namespace gigatron
{

  class ExtendedState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _mode_type;
      _mode_type mode;
      typedef gigatron::Drive _drive_type;
      _drive_type drive;
      typedef bool _estop_type;
      _estop_type estop;

    ExtendedState():
      header(),
      mode(""),
      drive(),
      estop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_mode = strlen(this->mode);
      varToArr(outbuffer + offset, length_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->mode, length_mode);
      offset += length_mode;
      offset += this->drive.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_estop;
      u_estop.real = this->estop;
      *(outbuffer + offset + 0) = (u_estop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->estop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_mode;
      arrToVar(length_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode-1]=0;
      this->mode = (char *)(inbuffer + offset-1);
      offset += length_mode;
      offset += this->drive.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_estop;
      u_estop.base = 0;
      u_estop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->estop = u_estop.real;
      offset += sizeof(this->estop);
     return offset;
    }

    const char * getType(){ return "gigatron/ExtendedState"; };
    const char * getMD5(){ return "1e9b67e46b84efcafb03eeb36df8ca6a"; };

  };

}
#endif