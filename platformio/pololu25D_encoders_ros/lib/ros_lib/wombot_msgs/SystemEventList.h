#ifndef _ROS_wombot_msgs_SystemEventList_h
#define _ROS_wombot_msgs_SystemEventList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "wombot_msgs/SystemEvent.h"

namespace wombot_msgs
{

  class SystemEventList : public ros::Msg
  {
    public:
      uint32_t events_length;
      typedef wombot_msgs::SystemEvent _events_type;
      _events_type st_events;
      _events_type * events;

    SystemEventList():
      events_length(0), st_events(), events(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->events_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->events_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->events_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->events_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->events_length);
      for( uint32_t i = 0; i < events_length; i++){
      offset += this->events[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t events_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      events_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      events_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      events_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->events_length);
      if(events_lengthT > events_length)
        this->events = (wombot_msgs::SystemEvent*)realloc(this->events, events_lengthT * sizeof(wombot_msgs::SystemEvent));
      events_length = events_lengthT;
      for( uint32_t i = 0; i < events_length; i++){
      offset += this->st_events.deserialize(inbuffer + offset);
        memcpy( &(this->events[i]), &(this->st_events), sizeof(wombot_msgs::SystemEvent));
      }
     return offset;
    }

    virtual const char * getType() override { return "wombot_msgs/SystemEventList"; };
    virtual const char * getMD5() override { return "cedca3b35b737f278fa2fa90cd2cb9ba"; };

  };

}
#endif
