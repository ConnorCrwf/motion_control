#ifndef _ROS_wombot_msgs_GapData_h
#define _ROS_wombot_msgs_GapData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace wombot_msgs
{

  class GapData : public ros::Msg
  {
    public:
      typedef uint32_t _seq_type;
      _seq_type seq;
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef float _odom_dist_type;
      _odom_dist_type odom_dist;
      typedef float _robot_theta_type;
      _robot_theta_type robot_theta;
      typedef float _gap_size_type;
      _gap_size_type gap_size;

    GapData():
      seq(0),
      stamp(),
      odom_dist(0),
      robot_theta(0),
      gap_size(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->seq >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->seq >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->seq >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->seq >> (8 * 3)) & 0xFF;
      offset += sizeof(this->seq);
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      offset += serializeAvrFloat64(outbuffer + offset, this->odom_dist);
      offset += serializeAvrFloat64(outbuffer + offset, this->robot_theta);
      offset += serializeAvrFloat64(outbuffer + offset, this->gap_size);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->seq =  ((uint32_t) (*(inbuffer + offset)));
      this->seq |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->seq |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->seq |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->seq);
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->odom_dist));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->robot_theta));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gap_size));
     return offset;
    }

    virtual const char * getType() override { return "wombot_msgs/GapData"; };
    virtual const char * getMD5() override { return "b3db6342dfe6d969e498cd7d74807467"; };

  };

}
#endif
