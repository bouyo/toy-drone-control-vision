#ifndef _ROS_SERVICE_MmCalibration_h
#define _ROS_SERVICE_MmCalibration_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace morus_uav_ros_msgs
{

static const char MMCALIBRATION[] = "morus_uav_ros_msgs/MmCalibration";

  class MmCalibrationRequest : public ros::Msg
  {
    public:
      typedef int8_t _motor_id_type;
      _motor_id_type motor_id;

    MmCalibrationRequest():
      motor_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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

    const char * getType(){ return MMCALIBRATION; };
    const char * getMD5(){ return "66f99a84d0f0c5db8defdf884367519d"; };

  };

  class MmCalibrationResponse : public ros::Msg
  {
    public:
      typedef bool _ok_type;
      _ok_type ok;

    MmCalibrationResponse():
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

    const char * getType(){ return MMCALIBRATION; };
    const char * getMD5(){ return "6f6da3883749771fac40d6deb24a8c02"; };

  };

  class MmCalibration {
    public:
    typedef MmCalibrationRequest Request;
    typedef MmCalibrationResponse Response;
  };

}
#endif
