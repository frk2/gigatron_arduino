#ifndef _ROS_gigatron_State_h
#define _ROS_gigatron_State_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "gigatron/Drive.h"

namespace gigatron
{

  class State : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _mode_type;
      _mode_type mode;
      typedef gigatron::Drive _drive_type;
      _drive_type drive;

    State():
      header(),
      mode(""),
      drive()
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
     return offset;
    }

    const char * getType(){ return "gigatron/State"; };
    const char * getMD5(){ return "5d8838928981c71d02635a3b70b7fd0f"; };

  };

}
#endif