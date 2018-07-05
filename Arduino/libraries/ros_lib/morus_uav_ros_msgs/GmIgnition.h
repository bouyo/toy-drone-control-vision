#ifndef _ROS_morus_uav_ros_msgs_GmIgnition_h
#define _ROS_morus_uav_ros_msgs_GmIgnition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace morus_uav_ros_msgs
{

  class GmIgnition : public ros::Msg
  {
    public:
      typedef int8_t _motor_id_type;
      _motor_id_type motor_id;
      typedef int8_t _ignition_type;
      _ignition_type ignition;

    GmIgnition():
      motor_id(0),
      ignition(0)
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
      } u_ignition;
      u_ignition.real = this->ignition;
      *(outbuffer + offset + 0) = (u_ignition.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ignition);
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
      } u_ignition;
      u_ignition.base = 0;
      u_ignition.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ignition = u_ignition.real;
      offset += sizeof(this->ignition);
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/GmIgnition"; };
    const char * getMD5(){ return "b6c287123fb01b2d3746d3ba2af74760"; };

  };

}
#endif