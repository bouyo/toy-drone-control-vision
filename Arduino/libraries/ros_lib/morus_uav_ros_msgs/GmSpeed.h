#ifndef _ROS_morus_uav_ros_msgs_GmSpeed_h
#define _ROS_morus_uav_ros_msgs_GmSpeed_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace morus_uav_ros_msgs
{

  class GmSpeed : public ros::Msg
  {
    public:
      uint32_t speed_length;
      typedef float _speed_type;
      _speed_type st_speed;
      _speed_type * speed;

    GmSpeed():
      speed_length(0), speed(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->speed_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->speed_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->speed_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->speed_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_length);
      for( uint32_t i = 0; i < speed_length; i++){
      union {
        float real;
        uint32_t base;
      } u_speedi;
      u_speedi.real = this->speed[i];
      *(outbuffer + offset + 0) = (u_speedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speedi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t speed_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      speed_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      speed_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      speed_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->speed_length);
      if(speed_lengthT > speed_length)
        this->speed = (float*)realloc(this->speed, speed_lengthT * sizeof(float));
      speed_length = speed_lengthT;
      for( uint32_t i = 0; i < speed_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_speed;
      u_st_speed.base = 0;
      u_st_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_speed = u_st_speed.real;
      offset += sizeof(this->st_speed);
        memcpy( &(this->speed[i]), &(this->st_speed), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/GmSpeed"; };
    const char * getMD5(){ return "c2d600321673904df95b6b086e7cca22"; };

  };

}
#endif