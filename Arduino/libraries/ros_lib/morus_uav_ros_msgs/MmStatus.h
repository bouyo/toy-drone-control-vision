#ifndef _ROS_morus_uav_ros_msgs_MmStatus_h
#define _ROS_morus_uav_ros_msgs_MmStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace morus_uav_ros_msgs
{

  class MmStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _actuator_id_type;
      _actuator_id_type actuator_id;
      typedef float _position_M_type;
      _position_M_type position_M;
      typedef int32_t _speed_M_type;
      _speed_M_type speed_M;
      typedef float _temperature_M_type;
      _temperature_M_type temperature_M;
      typedef float _position_S_type;
      _position_S_type position_S;

    MmStatus():
      header(),
      actuator_id(0),
      position_M(0),
      speed_M(0),
      temperature_M(0),
      position_S(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_actuator_id;
      u_actuator_id.real = this->actuator_id;
      *(outbuffer + offset + 0) = (u_actuator_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->actuator_id);
      union {
        float real;
        uint32_t base;
      } u_position_M;
      u_position_M.real = this->position_M;
      *(outbuffer + offset + 0) = (u_position_M.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_M.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_M.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_M.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_M);
      union {
        int32_t real;
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
      } u_temperature_M;
      u_temperature_M.real = this->temperature_M;
      *(outbuffer + offset + 0) = (u_temperature_M.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature_M.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature_M.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature_M.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature_M);
      union {
        float real;
        uint32_t base;
      } u_position_S;
      u_position_S.real = this->position_S;
      *(outbuffer + offset + 0) = (u_position_S.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_S.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_S.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_S.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_S);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_actuator_id;
      u_actuator_id.base = 0;
      u_actuator_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->actuator_id = u_actuator_id.real;
      offset += sizeof(this->actuator_id);
      union {
        float real;
        uint32_t base;
      } u_position_M;
      u_position_M.base = 0;
      u_position_M.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_M.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_M.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_M.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_M = u_position_M.real;
      offset += sizeof(this->position_M);
      union {
        int32_t real;
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
      } u_temperature_M;
      u_temperature_M.base = 0;
      u_temperature_M.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature_M.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature_M.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature_M.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature_M = u_temperature_M.real;
      offset += sizeof(this->temperature_M);
      union {
        float real;
        uint32_t base;
      } u_position_S;
      u_position_S.base = 0;
      u_position_S.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_S.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_S.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_S.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_S = u_position_S.real;
      offset += sizeof(this->position_S);
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/MmStatus"; };
    const char * getMD5(){ return "4a6c4d06f68e23de7ccee79f171d4b6f"; };

  };

}
#endif