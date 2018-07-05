#ifndef _ROS_SERVICE_LevelHorizon_h
#define _ROS_SERVICE_LevelHorizon_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace morus_uav_ros_msgs
{

static const char LEVELHORIZON[] = "morus_uav_ros_msgs/LevelHorizon";

  class LevelHorizonRequest : public ros::Msg
  {
    public:
      typedef int8_t _node_id_type;
      _node_id_type node_id;
      typedef int8_t _n_type;
      _n_type n;

    LevelHorizonRequest():
      node_id(0),
      n(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_node_id;
      u_node_id.real = this->node_id;
      *(outbuffer + offset + 0) = (u_node_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->node_id);
      union {
        int8_t real;
        uint8_t base;
      } u_n;
      u_n.real = this->n;
      *(outbuffer + offset + 0) = (u_n.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->n);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_node_id;
      u_node_id.base = 0;
      u_node_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->node_id = u_node_id.real;
      offset += sizeof(this->node_id);
      union {
        int8_t real;
        uint8_t base;
      } u_n;
      u_n.base = 0;
      u_n.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->n = u_n.real;
      offset += sizeof(this->n);
     return offset;
    }

    const char * getType(){ return LEVELHORIZON; };
    const char * getMD5(){ return "c72df6678e53f12dfd0de096bac6cd53"; };

  };

  class LevelHorizonResponse : public ros::Msg
  {
    public:
      typedef bool _ok_type;
      _ok_type ok;

    LevelHorizonResponse():
      ok(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.real = this->ok;
      *(outbuffer + offset + 0) = (u_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ok);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.base = 0;
      u_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ok = u_ok.real;
      offset += sizeof(this->ok);
     return offset;
    }

    const char * getType(){ return LEVELHORIZON; };
    const char * getMD5(){ return "6f6da3883749771fac40d6deb24a8c02"; };

  };

  class LevelHorizon {
    public:
    typedef LevelHorizonRequest Request;
    typedef LevelHorizonResponse Response;
  };

}
#endif
