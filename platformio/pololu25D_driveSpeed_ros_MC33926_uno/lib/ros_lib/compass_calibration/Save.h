#ifndef _ROS_SERVICE_Save_h
#define _ROS_SERVICE_Save_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/String.h"

namespace compass_calibration
{

static const char SAVE[] = "compass_calibration/Save";

  class SaveRequest : public ros::Msg
  {
    public:
      typedef std_msgs::String _filepath_type;
      _filepath_type filepath;

    SaveRequest():
      filepath()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->filepath.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->filepath.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SAVE; };
    virtual const char * getMD5() override { return "39ed0cb62145dcb0087e41a1626d44d0"; };

  };

  class SaveResponse : public ros::Msg
  {
    public:

    SaveResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return SAVE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Save {
    public:
    typedef SaveRequest Request;
    typedef SaveResponse Response;
  };

}
#endif
