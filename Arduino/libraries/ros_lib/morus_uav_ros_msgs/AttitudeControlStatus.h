#ifndef _ROS_morus_uav_ros_msgs_AttitudeControlStatus_h
#define _ROS_morus_uav_ros_msgs_AttitudeControlStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "morus_uav_ros_msgs/PidStatus.h"

namespace morus_uav_ros_msgs
{

  class AttitudeControlStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef morus_uav_ros_msgs::PidStatus _roll_type;
      _roll_type roll;
      typedef morus_uav_ros_msgs::PidStatus _roll_rate_type;
      _roll_rate_type roll_rate;
      typedef morus_uav_ros_msgs::PidStatus _pitch_type;
      _pitch_type pitch;
      typedef morus_uav_ros_msgs::PidStatus _pitch_rate_type;
      _pitch_rate_type pitch_rate;
      typedef morus_uav_ros_msgs::PidStatus _yaw_type;
      _yaw_type yaw;
      typedef morus_uav_ros_msgs::PidStatus _yaw_rate_type;
      _yaw_rate_type yaw_rate;

    AttitudeControlStatus():
      header(),
      roll(),
      roll_rate(),
      pitch(),
      pitch_rate(),
      yaw(),
      yaw_rate()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->roll.serialize(outbuffer + offset);
      offset += this->roll_rate.serialize(outbuffer + offset);
      offset += this->pitch.serialize(outbuffer + offset);
      offset += this->pitch_rate.serialize(outbuffer + offset);
      offset += this->yaw.serialize(outbuffer + offset);
      offset += this->yaw_rate.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->roll.deserialize(inbuffer + offset);
      offset += this->roll_rate.deserialize(inbuffer + offset);
      offset += this->pitch.deserialize(inbuffer + offset);
      offset += this->pitch_rate.deserialize(inbuffer + offset);
      offset += this->yaw.deserialize(inbuffer + offset);
      offset += this->yaw_rate.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/AttitudeControlStatus"; };
    const char * getMD5(){ return "c64d8da35950ea2f76a19738d3a09225"; };

  };

}
#endif