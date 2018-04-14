#ifndef _ROS_gigatron_DriveStamped_h
#define _ROS_gigatron_DriveStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "gigatron/Drive.h"

namespace gigatron
{

  class DriveStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef gigatron::Drive _drive_type;
      _drive_type drive;

    DriveStamped():
      header(),
      drive()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->drive.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->drive.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "gigatron/DriveStamped"; };
    const char * getMD5(){ return "94cf5b9e1cd461c4a7431f611c9b0a9d"; };

  };

}
#endif