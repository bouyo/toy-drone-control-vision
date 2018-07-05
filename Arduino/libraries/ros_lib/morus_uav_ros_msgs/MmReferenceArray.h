#ifndef _ROS_morus_uav_ros_msgs_MmReferenceArray_h
#define _ROS_morus_uav_ros_msgs_MmReferenceArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "morus_uav_ros_msgs/MmReference.h"

namespace morus_uav_ros_msgs
{

  class MmReferenceArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t reference_length;
      typedef morus_uav_ros_msgs::MmReference _reference_type;
      _reference_type st_reference;
      _reference_type * reference;

    MmReferenceArray():
      header(),
      reference_length(0), reference(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->reference_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->reference_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->reference_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->reference_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reference_length);
      for( uint32_t i = 0; i < reference_length; i++){
      offset += this->reference[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t reference_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      reference_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      reference_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      reference_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->reference_length);
      if(reference_lengthT > reference_length)
        this->reference = (morus_uav_ros_msgs::MmReference*)realloc(this->reference, reference_lengthT * sizeof(morus_uav_ros_msgs::MmReference));
      reference_length = reference_lengthT;
      for( uint32_t i = 0; i < reference_length; i++){
      offset += this->st_reference.deserialize(inbuffer + offset);
        memcpy( &(this->reference[i]), &(this->st_reference), sizeof(morus_uav_ros_msgs::MmReference));
      }
     return offset;
    }

    const char * getType(){ return "morus_uav_ros_msgs/MmReferenceArray"; };
    const char * getMD5(){ return "af1800c924b65623773dd443f44e1dc6"; };

  };

}
#endif