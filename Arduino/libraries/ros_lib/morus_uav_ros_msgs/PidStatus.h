#ifndef _ROS_morus_uav_ros_msgs_PidStatus_h
#define _ROS_morus_uav_ros_msgs_PidStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace morus_uav_ros_msgs
{

  class PidStatus : public ros::Msg
  {
    public:
      typedef uint8_t _mode_type;
      _mode_type mode;
      typedef float _gain_proportional_type;
      _gain_proportional_type gain_proportional;
      typedef float _gain_integral_type;
      _gain_integral_type gain_integral;
      typedef float _gain_derivative_type;
      _gain_derivative_type gain_derivative;
      typedef float _sample_rate_type;
      _sample_rate_type sample_rate;
      typedef float _setpoint_type;
      _setpoint_type setpoint;
      typedef float _feedback_type;
      _feedback_type feedback;
      typedef float _output_type;
      _output_type output;
      typedef float _output_proportional_type;
      _output_proportional_type output_proportional;
      typedef float _output_integral_type;
      _output_integral_type output_integral;
      typedef float _output_derivative_type;
      _output_derivative_type output_derivative;

    PidStatus():
      mode(0),
      gain_proportional(0),
      gain_integral(0),
      gain_derivative(0),
      sample_rate(0),
      setpoint(0),
      feedback(0),
      output(0),
      output_proportional(0),
      output_integral(0),
      output_derivative(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      union {
        float real;
        uint32_t base;
      } u_gain_proportional;
      u_gain_proportional.real = this->gain_proportional;
      *(outbuffer + offset + 0) = (u_gain_proportional.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gain_proportional.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gain_proportional.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gain_proportional.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gain_proportional);
      union {
        float real;
        uint32_t base;
      } u_gain_integral;
      u_gain_integral.real = this->gain_integral;
      *(outbuffer + offset + 0) = (u_gain_integral.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gain_integral.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gain_integral.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gain_integral.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gain_integral);
      union {
        float real;
        uint32_t base;
      } u_gain_derivative;
      u_gain_derivative.real = this->gain_derivative;
      *(outbuffer + offset + 0) = (u_gain_derivative.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gain_derivative.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gain_derivative.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gain_derivative.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gain_derivative);
      union {
        float real;
        uint32_t base;
      } u_sample_rate;
      u_sample_rate.real = this->sample_rate;
      *(outbuffer + offset + 0) = (u_sample_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sample_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sample_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sample_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sample_rate);
      union {
        float real;
        uint32_t base;
      } u_setpoint;
      u_setpoint.real = this->setpoint;
      *(outbuffer + offset + 0) = (u_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->setpoint);
      union {
        float real;
        uint32_t base;
      } u_feedback;
      u_feedback.real = this->feedback;
      *(outbuffer + offset + 0) = (u_feedback.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_feedback.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_feedback.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_feedback.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->feedback);
      union {
        float real;
        uint32_t base;
      } u_output;
      u_output.real = this->output;
      *(outbuffer + offset + 0) = (u_output.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output);
      union {
        float real;
        uint32_t base;
      } u_output_proportional;
      u_output_proportional.real = this->output_proportional;
      *(outbuffer + offset + 0) = (u_output_proportional.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output_proportional.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output_proportional.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output_proportional.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_proportional);
      union {
        float real;
        uint32_t base;
      } u_output_integral;
      u_output_integral.real = this->output_integral;
      *(outbuffer + offset + 0) = (u_output_integral.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output_integral.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output_integral.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output_integral.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_integral);
      union {
        float real;
        uint32_t base;
      } u_output_derivative;
      u_output_derivative.real = this->output_derivative;
      *(outbuffer + offset + 0) = (u_output_derivative.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output_derivative.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output_derivative.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output_derivative.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_derivative);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
      union {
        float real;
        uint32_t base;
      } u_gain_proportional;
      u_gain_proportional.base = 0;
      u_gain_proportional.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gain_proportional.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gain_proportional.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gain_proportional.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gain_proportional = u_gain_proportional.real;
      offset += sizeof(this->gain_proportional);
      union {
        float real;
        uint32_t base;
      } u_gain_integral;
      u_gain_integral.base = 0;
      u_gain_integral.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gain_integral.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gain_integral.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gain_integral.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gain_integral = u_gain_integral.real;
      offset += sizeof(this->gain_integral);
      union {
        float real;
        uint32_t base;
      } u_gain_derivative;
      u_gain_derivative.base = 0;
      u_gain_derivative.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gain_derivative.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gain_derivative.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gain_derivative.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gain_derivative = u_gain_derivative.real;
      offset += sizeof(this->gain_derivative);
      union {
        float real;
        uint32_t base;
      } u_sample_rate;
      u_sample_rate.base = 0;
      u_sample_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sample_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sample_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sample_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sample_rate = u_sample_rate.real;
      offset += sizeof(this->sample_rate);
      union {
        float real;
        uint32_t base;
      } u_setpoint;
      u_setpoint.base = 0;
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->setpoint = u_setpoint.real;
      offset += sizeof(this->setpoint);
      union {
        float real;
        uint32_t base;
      } u_feedback;
      u_feedback.base = 0;
      u_feedback.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_feedback.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_feedback.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_feedback.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->feedback = u_feedback.real;
      offset += sizeof(this->feedback);
      union {
        float real;
        uint32_t base;
      } u_output;
      u_output.base = 0;
      u_output.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->output = u_output.real;
      offset += sizeof(this->output);
      union {
        float real;
        uint32_t base;
      } u_output_proportional;
      u_output_proportional.base = 0;
      u_output_proportional.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output_proportional.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output_proportional.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output_proportional.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->output_proportional = u_output_proportional.real;
      offset += sizeof(this->output_proportional);
      union {
        float real;
        uint32_t base;
      } u_output_integral;
      u_output_integral.base = 0;
      u_output_integral.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output_integral.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output_integral.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output_integral.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->output_integral = u_output_integral.real;
      offset += sizeof(this->output_integral);
      union {
        float real;
        uint32_t base;
      } u_output_derivative;
      u_output_derivative.base = 0;
      u_output_derivative.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output_derivative.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output_derivative.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output_derivative.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->output_derivative = u_output_derivative.real;
      offset += sizeof(this->output_derivative);
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/PidStatus"; };
    const char * getMD5(){ return "244bae0bd56e1d603818e627039a2458"; };

  };

}
#endif