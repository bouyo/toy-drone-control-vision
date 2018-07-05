#ifndef _ROS_morus_uav_ros_msgs_GmChoke_h
#define _ROS_morus_uav_ros_msgs_GmChoke_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace morus_uav_ros_msgs
{

  class GmChoke : public ros::Msg
  {
    public:
      typedef int8_t _motor_id_type;
      _motor_id_type motor_id;
      typedef int8_t _choke_type;
      _choke_type choke;

    GmChoke():
      motor_id(0),
      choke(0)
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
      union {
        int8_t real;
        uint8_t base;
      } u_choke;
      u_choke.real = this->choke;
      *(outbuffer + offset + 0) = (u_choke.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->choke);
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
      union {
        int8_t real;
        uint8_t base;
      } u_choke;
      u_choke.base = 0;
      u_choke.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->choke = u_choke.real;
      offset += sizeof(this->choke);
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/GmChoke"; };
    const char * getMD5(){ return "685ea4651c5238cad432b003ba431e5b"; };

  };

}
#endif