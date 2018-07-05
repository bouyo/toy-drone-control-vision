#ifndef _ROS_morus_uav_ros_msgs_PositionControlStatus_h
#define _ROS_morus_uav_ros_msgs_PositionControlStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "morus_uav_ros_msgs/PidStatus.h"

namespace morus_uav_ros_msgs
{

  class PositionControlStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef morus_uav_ros_msgs::PidStatus _x_type;
      _x_type x;
      typedef morus_uav_ros_msgs::PidStatus _vx_type;
      _vx_type vx;
      typedef morus_uav_ros_msgs::PidStatus _y_type;
      _y_type y;
      typedef morus_uav_ros_msgs::PidStatus _vy_type;
      _vy_type vy;
      typedef morus_uav_ros_msgs::PidStatus _z_type;
      _z_type z;
      typedef morus_uav_ros_msgs::PidStatus _vz_type;
      _vz_type vz;

    PositionControlStatus():
      header(),
      x(),
      vx(),
      y(),
      vy(),
      z(),
      vz()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->x.serialize(outbuffer + offset);
      offset += this->vx.serialize(outbuffer + offset);
      offset += this->y.serialize(outbuffer + offset);
      offset += this->vy.serialize(outbuffer + offset);
      offset += this->z.serialize(outbuffer + offset);
      offset += this->vz.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->x.deserialize(inbuffer + offset);
      offset += this->vx.deserialize(inbuffer + offset);
      offset += this->y.deserialize(inbuffer + offset);
      offset += this->vy.deserialize(inbuffer + offset);
      offset += this->z.deserialize(inbuffer + offset);
      offset += this->vz.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/PositionControlStatus"; };
    const char * getMD5(){ return "3be8a31764bcf598381362b14a7a4a4c"; };

  };

}
#endif