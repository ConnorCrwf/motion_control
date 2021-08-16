#ifndef _ROS_motion_control_encoder_rad_data_h
#define _ROS_motion_control_encoder_rad_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace motion_control
{

  class encoder_rad_data : public ros::Msg
  {
    public:
      float data[2];

    encoder_rad_data():
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 2; i++){
      union {
        float real;
        uint32_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 2; i++){
      union {
        float real;
        uint32_t base;
      } u_datai;
      u_datai.base = 0;
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data[i] = u_datai.real;
      offset += sizeof(this->data[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "motion_control/encoder_rad_data"; };
    virtual const char * getMD5() override { return "3a423cd43be5532c6a3def568afe4aec"; };

  };

}
#endif
