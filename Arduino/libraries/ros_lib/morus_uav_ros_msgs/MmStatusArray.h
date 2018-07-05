#ifndef _ROS_morus_uav_ros_msgs_MmStatusArray_h
#define _ROS_morus_uav_ros_msgs_MmStatusArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "morus_uav_ros_msgs/MmStatus.h"

namespace morus_uav_ros_msgs
{

  class MmStatusArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t status_length;
      typedef morus_uav_ros_msgs::MmStatus _status_type;
      _status_type st_status;
      _status_type * status;

    MmStatusArray():
      header(),
      status_length(0), status(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->status_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->status_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->status_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->status_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status_length);
      for( uint32_t i = 0; i < status_length; i++){
      offset += this->status[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t status_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      status_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      status_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      status_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->status_length);
      if(status_lengthT > status_length)
        this->status = (morus_uav_ros_msgs::MmStatus*)realloc(this->status, status_lengthT * sizeof(morus_uav_ros_msgs::MmStatus));
      status_length = status_lengthT;
      for( uint32_t i = 0; i < status_length; i++){
      offset += this->st_status.deserialize(inbuffer + offset);
        memcpy( &(this->status[i]), &(this->st_status), sizeof(morus_uav_ros_msgs::MmStatus));
      }
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/MmStatusArray"; };
    const char * getMD5(){ return "c1bd88388e211d6c623ea629437918fa"; };

  };

}
#endif