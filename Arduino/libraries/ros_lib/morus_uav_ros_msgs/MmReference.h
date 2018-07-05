#ifndef _ROS_morus_uav_ros_msgs_MmReference_h
#define _ROS_morus_uav_ros_msgs_MmReference_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace morus_uav_ros_msgs
{

  class MmReference : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _actuator_id_type;
      _actuator_id_type actuator_id;
      typedef float _position_S_type;
      _position_S_type position_S;

    MmReference():
      header(),
      actuator_id(0),
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

    const char * getType(){ return "morus_uav_ros_msgs/MmReference"; };
    const char * getMD5(){ return "d30e2d5a7826165c4e43f6d9bfc1ddb5"; };

  };

}
#endif