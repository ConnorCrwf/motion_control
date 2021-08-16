#ifndef _ROS_wombot_msgs_SafetyData_h
#define _ROS_wombot_msgs_SafetyData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace wombot_msgs
{

  class SafetyData : public ros::Msg
  {
    public:
      typedef uint32_t _seq_type;
      _seq_type seq;
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef float _pressure_state_type;
      _pressure_state_type pressure_state;
      typedef float _positive_pressure_type;
      _positive_pressure_type positive_pressure;
      typedef float _pressure_setpoint_type;
      _pressure_setpoint_type pressure_setpoint;
      typedef float _control_effort_type;
      _control_effort_type control_effort;
      typedef int32_t _servo_load_type;
      _servo_load_type servo_load;
      typedef int32_t _servo_speed_state_type;
      _servo_speed_state_type servo_speed_state;
      typedef int32_t _servo_speed_cmd_type;
      _servo_speed_cmd_type servo_speed_cmd;
      typedef int32_t _servo_position_state_type;
      _servo_position_state_type servo_position_state;
      typedef int32_t _servo_position_cmd_type;
      _servo_position_cmd_type servo_position_cmd;

    SafetyData():
      seq(0),
      stamp(),
      pressure_state(0),
      positive_pressure(0),
      pressure_setpoint(0),
      control_effort(0),
      servo_load(0),
      servo_speed_state(0),
      servo_speed_cmd(0),
      servo_position_state(0),
      servo_position_cmd(0)
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
      offset += serializeAvrFloat64(outbuffer + offset, this->pressure_state);
      offset += serializeAvrFloat64(outbuffer + offset, this->positive_pressure);
      offset += serializeAvrFloat64(outbuffer + offset, this->pressure_setpoint);
      offset += serializeAvrFloat64(outbuffer + offset, this->control_effort);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_load;
      u_servo_load.real = this->servo_load;
      *(outbuffer + offset + 0) = (u_servo_load.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_load.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_load.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_load.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo_load);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_speed_state;
      u_servo_speed_state.real = this->servo_speed_state;
      *(outbuffer + offset + 0) = (u_servo_speed_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_speed_state.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_speed_state.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_speed_state.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo_speed_state);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_speed_cmd;
      u_servo_speed_cmd.real = this->servo_speed_cmd;
      *(outbuffer + offset + 0) = (u_servo_speed_cmd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_speed_cmd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_speed_cmd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_speed_cmd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo_speed_cmd);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_position_state;
      u_servo_position_state.real = this->servo_position_state;
      *(outbuffer + offset + 0) = (u_servo_position_state.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_position_state.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_position_state.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_position_state.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo_position_state);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_position_cmd;
      u_servo_position_cmd.real = this->servo_position_cmd;
      *(outbuffer + offset + 0) = (u_servo_position_cmd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_position_cmd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_position_cmd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_position_cmd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo_position_cmd);
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
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pressure_state));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->positive_pressure));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pressure_setpoint));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->control_effort));
      union {
        int32_t real;
        uint32_t base;
      } u_servo_load;
      u_servo_load.base = 0;
      u_servo_load.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_load.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_load.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_load.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo_load = u_servo_load.real;
      offset += sizeof(this->servo_load);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_speed_state;
      u_servo_speed_state.base = 0;
      u_servo_speed_state.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_speed_state.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_speed_state.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_speed_state.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo_speed_state = u_servo_speed_state.real;
      offset += sizeof(this->servo_speed_state);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_speed_cmd;
      u_servo_speed_cmd.base = 0;
      u_servo_speed_cmd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_speed_cmd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_speed_cmd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_speed_cmd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo_speed_cmd = u_servo_speed_cmd.real;
      offset += sizeof(this->servo_speed_cmd);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_position_state;
      u_servo_position_state.base = 0;
      u_servo_position_state.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_position_state.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_position_state.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_position_state.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo_position_state = u_servo_position_state.real;
      offset += sizeof(this->servo_position_state);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_position_cmd;
      u_servo_position_cmd.base = 0;
      u_servo_position_cmd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_position_cmd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_position_cmd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_position_cmd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo_position_cmd = u_servo_position_cmd.real;
      offset += sizeof(this->servo_position_cmd);
     return offset;
    }

    virtual const char * getType() override { return "wombot_msgs/SafetyData"; };
    virtual const char * getMD5() override { return "81cffd2855d2199e4fb9c2e6d106af1c"; };

  };

}
#endif
