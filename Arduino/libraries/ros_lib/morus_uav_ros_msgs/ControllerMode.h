#ifndef _ROS_morus_uav_ros_msgs_ControllerMode_h
#define _ROS_morus_uav_ros_msgs_ControllerMode_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "morus_uav_ros_msgs/GmIgnition.h"

namespace morus_uav_ros_msgs
{

  class ControllerMode : public ros::Msg
  {
    public:
      typedef uint8_t _armed_type;
      _armed_type armed;
      typedef uint8_t _controller_x_type;
      _controller_x_type controller_x;
      typedef uint8_t _controller_y_type;
      _controller_y_type controller_y;
      typedef uint8_t _controller_z_type;
      _controller_z_type controller_z;
      typedef uint8_t _controller_velocity_x_type;
      _controller_velocity_x_type controller_velocity_x;
      typedef uint8_t _controller_velocity_y_type;
      _controller_velocity_y_type controller_velocity_y;
      typedef uint8_t _controller_velocity_z_type;
      _controller_velocity_z_type controller_velocity_z;
      typedef uint8_t _controller_yaw_type;
      _controller_yaw_type controller_yaw;
      typedef uint8_t _controller_yaw_rate_type;
      _controller_yaw_rate_type controller_yaw_rate;
      typedef morus_uav_ros_msgs::GmIgnition _ignition_type;
      _ignition_type ignition;

    ControllerMode():
      armed(0),
      controller_x(0),
      controller_y(0),
      controller_z(0),
      controller_velocity_x(0),
      controller_velocity_y(0),
      controller_velocity_z(0),
      controller_yaw(0),
      controller_yaw_rate(0),
      ignition()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->armed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->armed);
      *(outbuffer + offset + 0) = (this->controller_x >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_x);
      *(outbuffer + offset + 0) = (this->controller_y >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_y);
      *(outbuffer + offset + 0) = (this->controller_z >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_z);
      *(outbuffer + offset + 0) = (this->controller_velocity_x >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_velocity_x);
      *(outbuffer + offset + 0) = (this->controller_velocity_y >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_velocity_y);
      *(outbuffer + offset + 0) = (this->controller_velocity_z >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_velocity_z);
      *(outbuffer + offset + 0) = (this->controller_yaw >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_yaw);
      *(outbuffer + offset + 0) = (this->controller_yaw_rate >> (8 * 0)) & 0xFF;
      offset += sizeof(this->controller_yaw_rate);
      offset += this->ignition.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->armed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->armed);
      this->controller_x =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->controller_x);
      this->controller_y =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->controller_y);
      this->controller_z =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->controller_z);
      this->controller_velocity_x =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->controller_velocity_x);
      this->controller_velocity_y =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->controller_velocity_y);
      this->controller_velocity_z =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->controller_velocity_z);
      this->controller_yaw =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->controller_yaw);
      this->controller_yaw_rate =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->controller_yaw_rate);
      offset += this->ignition.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/ControllerMode"; };
    const char * getMD5(){ return "d56349f1bce5e0ad24ddd0926629e0a7"; };

  };

}
#endif