#ifndef _ROS_morus_uav_ros_msgs_GmFuelStatus_h
#define _ROS_morus_uav_ros_msgs_GmFuelStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace morus_uav_ros_msgs
{

  class GmFuelStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _sensor_id_type;
      _sensor_id_type sensor_id;
      typedef int8_t _fuel_level_type;
      _fuel_level_type fuel_level;

    GmFuelStatus():
      header(),
      sensor_id(0),
      fuel_level(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_sensor_id;
      u_sensor_id.real = this->sensor_id;
      *(outbuffer + offset + 0) = (u_sensor_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sensor_id);
      union {
        int8_t real;
        uint8_t base;
      } u_fuel_level;
      u_fuel_level.real = this->fuel_level;
      *(outbuffer + offset + 0) = (u_fuel_level.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fuel_level);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_sensor_id;
      u_sensor_id.base = 0;
      u_sensor_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sensor_id = u_sensor_id.real;
      offset += sizeof(this->sensor_id);
      union {
        int8_t real;
        uint8_t base;
      } u_fuel_level;
      u_fuel_level.base = 0;
      u_fuel_level.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fuel_level = u_fuel_level.real;
      offset += sizeof(this->fuel_level);
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/GmFuelStatus"; };
    const char * getMD5(){ return "87dc7c49e32dc2aea7dc80e31eb41e09"; };

  };

}
#endif