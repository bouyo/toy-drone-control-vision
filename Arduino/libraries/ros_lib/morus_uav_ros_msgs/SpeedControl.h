#ifndef _ROS_SERVICE_SpeedControl_h
#define _ROS_SERVICE_SpeedControl_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace morus_uav_ros_msgs
{

static const char SPEEDCONTROL[] = "morus_uav_ros_msgs/SpeedControl";

  class SpeedControlRequest : public ros::Msg
  {
    public:
      typedef uint8_t _speed_control_type;
      _speed_control_type speed_control;
      typedef int8_t _motor_id_type;
      _motor_id_type motor_id;

    SpeedControlRequest():
      speed_control(0),
      motor_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->speed_control >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed_control);
      union {
        int8_t real;
        uint8_t base;
      } u_motor_id;
      u_motor_id.real = this->motor_id;
      *(outbuffer + offset + 0) = (u_motor_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motor_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->speed_control =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->speed_control);
      union {
        int8_t real;
        uint8_t base;
      } u_motor_id;
      u_motor_id.base = 0;
      u_motor_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motor_id = u_motor_id.real;
      offset += sizeof(this->motor_id);
     return offset;
    }

    const char * getType(){ return SPEEDCONTROL; };
    const char * getMD5(){ return "930871d8a1e52e5a93bc7c24b5e55a04"; };

  };

  class SpeedControlResponse : public ros::Msg
  {
    public:
      typedef bool _ok_type;
      _ok_type ok;

    SpeedControlResponse():
      ok(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.real = this->ok;
      *(outbuffer + offset + 0) = (u_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ok);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.base = 0;
      u_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ok = u_ok.real;
      offset += sizeof(this->ok);
     return offset;
    }

    const char * getType(){ return SPEEDCONTROL; };
    const char * getMD5(){ return "6f6da3883749771fac40d6deb24a8c02"; };

  };

  class SpeedControl {
    public:
    typedef SpeedControlRequest Request;
    typedef SpeedControlResponse Response;
  };

}
#endif
