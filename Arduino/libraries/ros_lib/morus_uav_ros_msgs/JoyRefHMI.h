#ifndef _ROS_morus_uav_ros_msgs_JoyRefHMI_h
#define _ROS_morus_uav_ros_msgs_JoyRefHMI_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace morus_uav_ros_msgs
{

  class JoyRefHMI : public ros::Msg
  {
    public:
      typedef int8_t _ignition1_type;
      _ignition1_type ignition1;
      typedef int8_t _ignition2_type;
      _ignition2_type ignition2;
      typedef int8_t _ignition3_type;
      _ignition3_type ignition3;
      typedef int8_t _ignition4_type;
      _ignition4_type ignition4;
      typedef bool _masterIgnitionFlag_type;
      _masterIgnitionFlag_type masterIgnitionFlag;
      typedef int16_t _ppm1_type;
      _ppm1_type ppm1;
      typedef int16_t _ppm2_type;
      _ppm2_type ppm2;
      typedef int16_t _ppm3_type;
      _ppm3_type ppm3;
      typedef int16_t _ppm4_type;
      _ppm4_type ppm4;
      typedef bool _masterPpmFlag_type;
      _masterPpmFlag_type masterPpmFlag;
      typedef float _roll_type;
      _roll_type roll;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef float _z_type;
      _z_type z;

    JoyRefHMI():
      ignition1(0),
      ignition2(0),
      ignition3(0),
      ignition4(0),
      masterIgnitionFlag(0),
      ppm1(0),
      ppm2(0),
      ppm3(0),
      ppm4(0),
      masterPpmFlag(0),
      roll(0),
      pitch(0),
      yaw(0),
      z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_ignition1;
      u_ignition1.real = this->ignition1;
      *(outbuffer + offset + 0) = (u_ignition1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ignition1);
      union {
        int8_t real;
        uint8_t base;
      } u_ignition2;
      u_ignition2.real = this->ignition2;
      *(outbuffer + offset + 0) = (u_ignition2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ignition2);
      union {
        int8_t real;
        uint8_t base;
      } u_ignition3;
      u_ignition3.real = this->ignition3;
      *(outbuffer + offset + 0) = (u_ignition3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ignition3);
      union {
        int8_t real;
        uint8_t base;
      } u_ignition4;
      u_ignition4.real = this->ignition4;
      *(outbuffer + offset + 0) = (u_ignition4.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ignition4);
      union {
        bool real;
        uint8_t base;
      } u_masterIgnitionFlag;
      u_masterIgnitionFlag.real = this->masterIgnitionFlag;
      *(outbuffer + offset + 0) = (u_masterIgnitionFlag.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->masterIgnitionFlag);
      union {
        int16_t real;
        uint16_t base;
      } u_ppm1;
      u_ppm1.real = this->ppm1;
      *(outbuffer + offset + 0) = (u_ppm1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ppm1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ppm1);
      union {
        int16_t real;
        uint16_t base;
      } u_ppm2;
      u_ppm2.real = this->ppm2;
      *(outbuffer + offset + 0) = (u_ppm2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ppm2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ppm2);
      union {
        int16_t real;
        uint16_t base;
      } u_ppm3;
      u_ppm3.real = this->ppm3;
      *(outbuffer + offset + 0) = (u_ppm3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ppm3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ppm3);
      union {
        int16_t real;
        uint16_t base;
      } u_ppm4;
      u_ppm4.real = this->ppm4;
      *(outbuffer + offset + 0) = (u_ppm4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ppm4.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ppm4);
      union {
        bool real;
        uint8_t base;
      } u_masterPpmFlag;
      u_masterPpmFlag.real = this->masterPpmFlag;
      *(outbuffer + offset + 0) = (u_masterPpmFlag.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->masterPpmFlag);
      offset += serializeAvrFloat64(outbuffer + offset, this->roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_ignition1;
      u_ignition1.base = 0;
      u_ignition1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ignition1 = u_ignition1.real;
      offset += sizeof(this->ignition1);
      union {
        int8_t real;
        uint8_t base;
      } u_ignition2;
      u_ignition2.base = 0;
      u_ignition2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ignition2 = u_ignition2.real;
      offset += sizeof(this->ignition2);
      union {
        int8_t real;
        uint8_t base;
      } u_ignition3;
      u_ignition3.base = 0;
      u_ignition3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ignition3 = u_ignition3.real;
      offset += sizeof(this->ignition3);
      union {
        int8_t real;
        uint8_t base;
      } u_ignition4;
      u_ignition4.base = 0;
      u_ignition4.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ignition4 = u_ignition4.real;
      offset += sizeof(this->ignition4);
      union {
        bool real;
        uint8_t base;
      } u_masterIgnitionFlag;
      u_masterIgnitionFlag.base = 0;
      u_masterIgnitionFlag.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->masterIgnitionFlag = u_masterIgnitionFlag.real;
      offset += sizeof(this->masterIgnitionFlag);
      union {
        int16_t real;
        uint16_t base;
      } u_ppm1;
      u_ppm1.base = 0;
      u_ppm1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ppm1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ppm1 = u_ppm1.real;
      offset += sizeof(this->ppm1);
      union {
        int16_t real;
        uint16_t base;
      } u_ppm2;
      u_ppm2.base = 0;
      u_ppm2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ppm2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ppm2 = u_ppm2.real;
      offset += sizeof(this->ppm2);
      union {
        int16_t real;
        uint16_t base;
      } u_ppm3;
      u_ppm3.base = 0;
      u_ppm3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ppm3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ppm3 = u_ppm3.real;
      offset += sizeof(this->ppm3);
      union {
        int16_t real;
        uint16_t base;
      } u_ppm4;
      u_ppm4.base = 0;
      u_ppm4.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ppm4.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ppm4 = u_ppm4.real;
      offset += sizeof(this->ppm4);
      union {
        bool real;
        uint8_t base;
      } u_masterPpmFlag;
      u_masterPpmFlag.base = 0;
      u_masterPpmFlag.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->masterPpmFlag = u_masterPpmFlag.real;
      offset += sizeof(this->masterPpmFlag);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->z));
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/JoyRefHMI"; };
    const char * getMD5(){ return "266b56fc90d3c337bf4d065450b2ccb3"; };

  };

}
#endif