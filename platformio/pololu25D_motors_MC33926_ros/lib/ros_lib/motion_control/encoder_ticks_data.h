#ifndef _ROS_motion_control_encoder_ticks_data_h
#define _ROS_motion_control_encoder_ticks_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace motion_control
{

  class encoder_ticks_data : public ros::Msg
  {
    public:
      int64_t data[2];

    encoder_ticks_data():
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 2; i++){
      union {
        int64_t real;
        uint64_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_datai.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_datai.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_datai.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_datai.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 2; i++){
      union {
        int64_t real;
        uint64_t base;
      } u_datai;
      u_datai.base = 0;
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->data[i] = u_datai.real;
      offset += sizeof(this->data[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "motion_control/encoder_ticks_data"; };
    virtual const char * getMD5() override { return "f2fcb6b78c66c7c2f2c91c68e2474290"; };

  };

}
#endif
