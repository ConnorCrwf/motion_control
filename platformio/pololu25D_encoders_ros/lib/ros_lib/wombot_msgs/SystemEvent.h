#ifndef _ROS_wombot_msgs_SystemEvent_h
#define _ROS_wombot_msgs_SystemEvent_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace wombot_msgs
{

  class SystemEvent : public ros::Msg
  {
    public:
      typedef const char* _msg_type;
      _msg_type msg;
      typedef const char* _code_type;
      _code_type code;

    SystemEvent():
      msg(""),
      code("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_msg = strlen(this->msg);
      varToArr(outbuffer + offset, length_msg);
      offset += 4;
      memcpy(outbuffer + offset, this->msg, length_msg);
      offset += length_msg;
      uint32_t length_code = strlen(this->code);
      varToArr(outbuffer + offset, length_code);
      offset += 4;
      memcpy(outbuffer + offset, this->code, length_code);
      offset += length_code;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_msg;
      arrToVar(length_msg, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_msg; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_msg-1]=0;
      this->msg = (char *)(inbuffer + offset-1);
      offset += length_msg;
      uint32_t length_code;
      arrToVar(length_code, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_code; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_code-1]=0;
      this->code = (char *)(inbuffer + offset-1);
      offset += length_code;
     return offset;
    }

    virtual const char * getType() override { return "wombot_msgs/SystemEvent"; };
    virtual const char * getMD5() override { return "59b7cc05341285597200d8c84094862e"; };

  };

}
#endif
