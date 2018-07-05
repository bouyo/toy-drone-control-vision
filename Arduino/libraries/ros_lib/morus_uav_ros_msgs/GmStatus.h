#ifndef _ROS_morus_uav_ros_msgs_GmStatus_h
#define _ROS_morus_uav_ros_msgs_GmStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace morus_uav_ros_msgs
{

  class GmStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _motor_id_type;
      _motor_id_type motor_id;
      typedef float _force_M_type;
      _force_M_type force_M;
      typedef float _speed_M_type;
      _speed_M_type speed_M;
      typedef float _temperatureR_M_type;
      _temperatureR_M_type temperatureR_M;
      typedef float _temperatureL_M_type;
      _temperatureL_M_type temperatureL_M;
      typedef float _fuel_level_M_type;
      _fuel_level_M_type fuel_level_M;
      typedef float _throttle_M_type;
      _throttle_M_type throttle_M;
      typedef int8_t _choke_M_type;
      _choke_M_type choke_M;
      typedef float _engine_hours_type;
      _engine_hours_type engine_hours;
      typedef int8_t _ignition_S_type;
      _ignition_S_type ignition_S;
      typedef int16_t _starter_ppm_S_type;
      _starter_ppm_S_type starter_ppm_S;
      typedef float _speed_S_type;
      _speed_S_type speed_S;
      typedef float _throttle_S_type;
      _throttle_S_type throttle_S;
      typedef int8_t _speed_ctl_S_type;
      _speed_ctl_S_type speed_ctl_S;
      typedef int8_t _choke_S_type;
      _choke_S_type choke_S;

    GmStatus():
      header(),
      motor_id(0),
      force_M(0),
      speed_M(0),
      temperatureR_M(0),
      temperatureL_M(0),
      fuel_level_M(0),
      throttle_M(0),
      choke_M(0),
      engine_hours(0),
      ignition_S(0),
      starter_ppm_S(0),
      speed_S(0),
      throttle_S(0),
      speed_ctl_S(0),
      choke_S(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_motor_id;
      u_motor_id.real = this->motor_id;
      *(outbuffer + offset + 0) = (u_motor_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motor_id);
      union {
        float real;
        uint32_t base;
      } u_force_M;
      u_force_M.real = this->force_M;
      *(outbuffer + offset + 0) = (u_force_M.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_force_M.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_force_M.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_force_M.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->force_M);
      union {
        float real;
        uint32_t base;
      } u_speed_M;
      u_speed_M.real = this->speed_M;
      *(outbuffer + offset + 0) = (u_speed_M.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_M.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_M.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_M.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_M);
      union {
        float real;
        uint32_t base;
      } u_temperatureR_M;
      u_temperatureR_M.real = this->temperatureR_M;
      *(outbuffer + offset + 0) = (u_temperatureR_M.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperatureR_M.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperatureR_M.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperatureR_M.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperatureR_M);
      union {
        float real;
        uint32_t base;
      } u_temperatureL_M;
      u_temperatureL_M.real = this->temperatureL_M;
      *(outbuffer + offset + 0) = (u_temperatureL_M.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperatureL_M.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperatureL_M.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperatureL_M.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperatureL_M);
      union {
        float real;
        uint32_t base;
      } u_fuel_level_M;
      u_fuel_level_M.real = this->fuel_level_M;
      *(outbuffer + offset + 0) = (u_fuel_level_M.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fuel_level_M.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fuel_level_M.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fuel_level_M.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fuel_level_M);
      union {
        float real;
        uint32_t base;
      } u_throttle_M;
      u_throttle_M.real = this->throttle_M;
      *(outbuffer + offset + 0) = (u_throttle_M.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_throttle_M.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_throttle_M.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_throttle_M.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->throttle_M);
      union {
        int8_t real;
        uint8_t base;
      } u_choke_M;
      u_choke_M.real = this->choke_M;
      *(outbuffer + offset + 0) = (u_choke_M.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->choke_M);
      union {
        float real;
        uint32_t base;
      } u_engine_hours;
      u_engine_hours.real = this->engine_hours;
      *(outbuffer + offset + 0) = (u_engine_hours.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_engine_hours.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_engine_hours.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_engine_hours.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->engine_hours);
      union {
        int8_t real;
        uint8_t base;
      } u_ignition_S;
      u_ignition_S.real = this->ignition_S;
      *(outbuffer + offset + 0) = (u_ignition_S.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ignition_S);
      union {
        int16_t real;
        uint16_t base;
      } u_starter_ppm_S;
      u_starter_ppm_S.real = this->starter_ppm_S;
      *(outbuffer + offset + 0) = (u_starter_ppm_S.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_starter_ppm_S.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->starter_ppm_S);
      union {
        float real;
        uint32_t base;
      } u_speed_S;
      u_speed_S.real = this->speed_S;
      *(outbuffer + offset + 0) = (u_speed_S.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_S.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_S.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_S.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_S);
      union {
        float real;
        uint32_t base;
      } u_throttle_S;
      u_throttle_S.real = this->throttle_S;
      *(outbuffer + offset + 0) = (u_throttle_S.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_throttle_S.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_throttle_S.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_throttle_S.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->throttle_S);
      union {
        int8_t real;
        uint8_t base;
      } u_speed_ctl_S;
      u_speed_ctl_S.real = this->speed_ctl_S;
      *(outbuffer + offset + 0) = (u_speed_ctl_S.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed_ctl_S);
      union {
        int8_t real;
        uint8_t base;
      } u_choke_S;
      u_choke_S.real = this->choke_S;
      *(outbuffer + offset + 0) = (u_choke_S.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->choke_S);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_motor_id;
      u_motor_id.base = 0;
      u_motor_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motor_id = u_motor_id.real;
      offset += sizeof(this->motor_id);
      union {
        float real;
        uint32_t base;
      } u_force_M;
      u_force_M.base = 0;
      u_force_M.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_force_M.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_force_M.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_force_M.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->force_M = u_force_M.real;
      offset += sizeof(this->force_M);
      union {
        float real;
        uint32_t base;
      } u_speed_M;
      u_speed_M.base = 0;
      u_speed_M.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_M.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_M.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_M.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_M = u_speed_M.real;
      offset += sizeof(this->speed_M);
      union {
        float real;
        uint32_t base;
      } u_temperatureR_M;
      u_temperatureR_M.base = 0;
      u_temperatureR_M.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperatureR_M.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperatureR_M.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperatureR_M.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperatureR_M = u_temperatureR_M.real;
      offset += sizeof(this->temperatureR_M);
      union {
        float real;
        uint32_t base;
      } u_temperatureL_M;
      u_temperatureL_M.base = 0;
      u_temperatureL_M.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperatureL_M.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperatureL_M.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperatureL_M.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperatureL_M = u_temperatureL_M.real;
      offset += sizeof(this->temperatureL_M);
      union {
        float real;
        uint32_t base;
      } u_fuel_level_M;
      u_fuel_level_M.base = 0;
      u_fuel_level_M.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fuel_level_M.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fuel_level_M.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fuel_level_M.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fuel_level_M = u_fuel_level_M.real;
      offset += sizeof(this->fuel_level_M);
      union {
        float real;
        uint32_t base;
      } u_throttle_M;
      u_throttle_M.base = 0;
      u_throttle_M.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_throttle_M.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_throttle_M.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_throttle_M.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->throttle_M = u_throttle_M.real;
      offset += sizeof(this->throttle_M);
      union {
        int8_t real;
        uint8_t base;
      } u_choke_M;
      u_choke_M.base = 0;
      u_choke_M.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->choke_M = u_choke_M.real;
      offset += sizeof(this->choke_M);
      union {
        float real;
        uint32_t base;
      } u_engine_hours;
      u_engine_hours.base = 0;
      u_engine_hours.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_engine_hours.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_engine_hours.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_engine_hours.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->engine_hours = u_engine_hours.real;
      offset += sizeof(this->engine_hours);
      union {
        int8_t real;
        uint8_t base;
      } u_ignition_S;
      u_ignition_S.base = 0;
      u_ignition_S.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ignition_S = u_ignition_S.real;
      offset += sizeof(this->ignition_S);
      union {
        int16_t real;
        uint16_t base;
      } u_starter_ppm_S;
      u_starter_ppm_S.base = 0;
      u_starter_ppm_S.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_starter_ppm_S.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->starter_ppm_S = u_starter_ppm_S.real;
      offset += sizeof(this->starter_ppm_S);
      union {
        float real;
        uint32_t base;
      } u_speed_S;
      u_speed_S.base = 0;
      u_speed_S.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_S.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_S.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_S.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_S = u_speed_S.real;
      offset += sizeof(this->speed_S);
      union {
        float real;
        uint32_t base;
      } u_throttle_S;
      u_throttle_S.base = 0;
      u_throttle_S.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_throttle_S.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_throttle_S.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_throttle_S.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->throttle_S = u_throttle_S.real;
      offset += sizeof(this->throttle_S);
      union {
        int8_t real;
        uint8_t base;
      } u_speed_ctl_S;
      u_speed_ctl_S.base = 0;
      u_speed_ctl_S.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->speed_ctl_S = u_speed_ctl_S.real;
      offset += sizeof(this->speed_ctl_S);
      union {
        int8_t real;
        uint8_t base;
      } u_choke_S;
      u_choke_S.base = 0;
      u_choke_S.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->choke_S = u_choke_S.real;
      offset += sizeof(this->choke_S);
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/GmStatus"; };
    const char * getMD5(){ return "98c46d712ad21d84384d3956ac2f2624"; };

  };

}
#endif