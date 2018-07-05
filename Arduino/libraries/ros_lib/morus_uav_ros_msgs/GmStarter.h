#ifndef _ROS_morus_uav_ros_msgs_GmStarter_h
#define _ROS_morus_uav_ros_msgs_GmStarter_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace morus_uav_ros_msgs
{

  class GmStarter : public ros::Msg
  {
    public:
      typedef int8_t _motor_id_type;
      _motor_id_type motor_id;
      typedef int16_t _ppm_type;
      _ppm_type ppm;

    GmStarter():
      motor_id(0),
      ppm(0)
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
        int16_t real;
        uint16_t base;
      } u_ppm;
      u_ppm.real = this->ppm;
      *(outbuffer + offset + 0) = (u_ppm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ppm.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ppm);
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
        int16_t real;
        uint16_t base;
      } u_ppm;
      u_ppm.base = 0;
      u_ppm.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ppm.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ppm = u_ppm.real;
      offset += sizeof(this->ppm);
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/GmStarter"; };
    const char * getMD5(){ return "b30117960fcae6f6dcba4eaf775cc86a"; };

  };

}
#endif