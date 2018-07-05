#ifndef _ROS_morus_uav_ros_msgs_GmThrottle_h
#define _ROS_morus_uav_ros_msgs_GmThrottle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace morus_uav_ros_msgs
{

  class GmThrottle : public ros::Msg
  {
    public:
      uint32_t throttle_length;
      typedef float _throttle_type;
      _throttle_type st_throttle;
      _throttle_type * throttle;

    GmThrottle():
      throttle_length(0), throttle(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->throttle_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->throttle_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->throttle_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->throttle_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->throttle_length);
      for( uint32_t i = 0; i < throttle_length; i++){
      union {
        float real;
        uint32_t base;
      } u_throttlei;
      u_throttlei.real = this->throttle[i];
      *(outbuffer + offset + 0) = (u_throttlei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_throttlei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_throttlei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_throttlei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->throttle[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t throttle_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      throttle_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      throttle_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      throttle_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->throttle_length);
      if(throttle_lengthT > throttle_length)
        this->throttle = (float*)realloc(this->throttle, throttle_lengthT * sizeof(float));
      throttle_length = throttle_lengthT;
      for( uint32_t i = 0; i < throttle_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_throttle;
      u_st_throttle.base = 0;
      u_st_throttle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_throttle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_throttle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_throttle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_throttle = u_st_throttle.real;
      offset += sizeof(this->st_throttle);
        memcpy( &(this->throttle[i]), &(this->st_throttle), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/GmThrottle"; };
    const char * getMD5(){ return "793ad828b6f77610e3d3acad7d4ceee2"; };

  };

}
#endif