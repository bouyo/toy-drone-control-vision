#ifndef _ROS_morus_uav_ros_msgs_PositionControllerReference_h
#define _ROS_morus_uav_ros_msgs_PositionControllerReference_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace morus_uav_ros_msgs
{

  class PositionControllerReference : public ros::Msg
  {
    public:
      typedef float _position_x_type;
      _position_x_type position_x;
      typedef float _position_y_type;
      _position_y_type position_y;
      typedef float _position_z_type;
      _position_z_type position_z;
      typedef float _velocity_x_type;
      _velocity_x_type velocity_x;
      typedef float _velocity_y_type;
      _velocity_y_type velocity_y;
      typedef float _velocity_z_type;
      _velocity_z_type velocity_z;
      typedef float _acceleration_x_type;
      _acceleration_x_type acceleration_x;
      typedef float _acceleration_y_type;
      _acceleration_y_type acceleration_y;
      typedef float _acceleration_z_type;
      _acceleration_z_type acceleration_z;
      typedef float _roll_type;
      _roll_type roll;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef float _yaw_rate_type;
      _yaw_rate_type yaw_rate;
      typedef float _yaw_manual_type;
      _yaw_manual_type yaw_manual;
      typedef float _throttle_type;
      _throttle_type throttle;

    PositionControllerReference():
      position_x(0),
      position_y(0),
      position_z(0),
      velocity_x(0),
      velocity_y(0),
      velocity_z(0),
      acceleration_x(0),
      acceleration_y(0),
      acceleration_z(0),
      roll(0),
      pitch(0),
      yaw(0),
      yaw_rate(0),
      yaw_manual(0),
      throttle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_position_x;
      u_position_x.real = this->position_x;
      *(outbuffer + offset + 0) = (u_position_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_x);
      union {
        float real;
        uint32_t base;
      } u_position_y;
      u_position_y.real = this->position_y;
      *(outbuffer + offset + 0) = (u_position_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_y);
      union {
        float real;
        uint32_t base;
      } u_position_z;
      u_position_z.real = this->position_z;
      *(outbuffer + offset + 0) = (u_position_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_z);
      union {
        float real;
        uint32_t base;
      } u_velocity_x;
      u_velocity_x.real = this->velocity_x;
      *(outbuffer + offset + 0) = (u_velocity_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_x);
      union {
        float real;
        uint32_t base;
      } u_velocity_y;
      u_velocity_y.real = this->velocity_y;
      *(outbuffer + offset + 0) = (u_velocity_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_y);
      union {
        float real;
        uint32_t base;
      } u_velocity_z;
      u_velocity_z.real = this->velocity_z;
      *(outbuffer + offset + 0) = (u_velocity_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_z);
      union {
        float real;
        uint32_t base;
      } u_acceleration_x;
      u_acceleration_x.real = this->acceleration_x;
      *(outbuffer + offset + 0) = (u_acceleration_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acceleration_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acceleration_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acceleration_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acceleration_x);
      union {
        float real;
        uint32_t base;
      } u_acceleration_y;
      u_acceleration_y.real = this->acceleration_y;
      *(outbuffer + offset + 0) = (u_acceleration_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acceleration_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acceleration_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acceleration_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acceleration_y);
      union {
        float real;
        uint32_t base;
      } u_acceleration_z;
      u_acceleration_z.real = this->acceleration_z;
      *(outbuffer + offset + 0) = (u_acceleration_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acceleration_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acceleration_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acceleration_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acceleration_z);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.real = this->roll;
      *(outbuffer + offset + 0) = (u_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.real = this->pitch;
      *(outbuffer + offset + 0) = (u_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.real = this->yaw;
      *(outbuffer + offset + 0) = (u_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_yaw_rate;
      u_yaw_rate.real = this->yaw_rate;
      *(outbuffer + offset + 0) = (u_yaw_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_rate);
      union {
        float real;
        uint32_t base;
      } u_yaw_manual;
      u_yaw_manual.real = this->yaw_manual;
      *(outbuffer + offset + 0) = (u_yaw_manual.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_manual.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_manual.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_manual.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_manual);
      union {
        float real;
        uint32_t base;
      } u_throttle;
      u_throttle.real = this->throttle;
      *(outbuffer + offset + 0) = (u_throttle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_throttle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_throttle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_throttle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->throttle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_position_x;
      u_position_x.base = 0;
      u_position_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_x = u_position_x.real;
      offset += sizeof(this->position_x);
      union {
        float real;
        uint32_t base;
      } u_position_y;
      u_position_y.base = 0;
      u_position_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_y = u_position_y.real;
      offset += sizeof(this->position_y);
      union {
        float real;
        uint32_t base;
      } u_position_z;
      u_position_z.base = 0;
      u_position_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_z = u_position_z.real;
      offset += sizeof(this->position_z);
      union {
        float real;
        uint32_t base;
      } u_velocity_x;
      u_velocity_x.base = 0;
      u_velocity_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_x = u_velocity_x.real;
      offset += sizeof(this->velocity_x);
      union {
        float real;
        uint32_t base;
      } u_velocity_y;
      u_velocity_y.base = 0;
      u_velocity_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_y = u_velocity_y.real;
      offset += sizeof(this->velocity_y);
      union {
        float real;
        uint32_t base;
      } u_velocity_z;
      u_velocity_z.base = 0;
      u_velocity_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity_z = u_velocity_z.real;
      offset += sizeof(this->velocity_z);
      union {
        float real;
        uint32_t base;
      } u_acceleration_x;
      u_acceleration_x.base = 0;
      u_acceleration_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acceleration_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acceleration_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acceleration_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acceleration_x = u_acceleration_x.real;
      offset += sizeof(this->acceleration_x);
      union {
        float real;
        uint32_t base;
      } u_acceleration_y;
      u_acceleration_y.base = 0;
      u_acceleration_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acceleration_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acceleration_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acceleration_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acceleration_y = u_acceleration_y.real;
      offset += sizeof(this->acceleration_y);
      union {
        float real;
        uint32_t base;
      } u_acceleration_z;
      u_acceleration_z.base = 0;
      u_acceleration_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acceleration_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acceleration_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acceleration_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acceleration_z = u_acceleration_z.real;
      offset += sizeof(this->acceleration_z);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.base = 0;
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll = u_roll.real;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.base = 0;
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch = u_pitch.real;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.base = 0;
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw = u_yaw.real;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_yaw_rate;
      u_yaw_rate.base = 0;
      u_yaw_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw_rate = u_yaw_rate.real;
      offset += sizeof(this->yaw_rate);
      union {
        float real;
        uint32_t base;
      } u_yaw_manual;
      u_yaw_manual.base = 0;
      u_yaw_manual.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_manual.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_manual.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_manual.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw_manual = u_yaw_manual.real;
      offset += sizeof(this->yaw_manual);
      union {
        float real;
        uint32_t base;
      } u_throttle;
      u_throttle.base = 0;
      u_throttle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_throttle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_throttle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_throttle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->throttle = u_throttle.real;
      offset += sizeof(this->throttle);
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/PositionControllerReference"; };
    const char * getMD5(){ return "8ca9a6f7be69f793ea19c5e9b86d9348"; };

  };

}
#endif