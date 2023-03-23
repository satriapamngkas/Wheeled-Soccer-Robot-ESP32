#ifndef _ROS_SERVICE_ObstaclesService_h
#define _ROS_SERVICE_ObstaclesService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "fukuro_common/Obstacle.h"

namespace fukuro_common
{

static const char OBSTACLESSERVICE[] = "fukuro_common/ObstaclesService";

  class ObstaclesServiceRequest : public ros::Msg
  {
    public:
      uint32_t obstacle_length;
      typedef fukuro_common::Obstacle _obstacle_type;
      _obstacle_type st_obstacle;
      _obstacle_type * obstacle;

    ObstaclesServiceRequest():
      obstacle_length(0), st_obstacle(), obstacle(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->obstacle_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->obstacle_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->obstacle_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->obstacle_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->obstacle_length);
      for( uint32_t i = 0; i < obstacle_length; i++){
      offset += this->obstacle[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t obstacle_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      obstacle_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      obstacle_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      obstacle_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->obstacle_length);
      if(obstacle_lengthT > obstacle_length)
        this->obstacle = (fukuro_common::Obstacle*)realloc(this->obstacle, obstacle_lengthT * sizeof(fukuro_common::Obstacle));
      obstacle_length = obstacle_lengthT;
      for( uint32_t i = 0; i < obstacle_length; i++){
      offset += this->st_obstacle.deserialize(inbuffer + offset);
        memcpy( &(this->obstacle[i]), &(this->st_obstacle), sizeof(fukuro_common::Obstacle));
      }
     return offset;
    }

    virtual const char * getType() override { return OBSTACLESSERVICE; };
    virtual const char * getMD5() override { return "0c04b02beeb901211c5a0f08490a07a3"; };

  };

  class ObstaclesServiceResponse : public ros::Msg
  {
    public:
      typedef uint8_t _ok_type;
      _ok_type ok;

    ObstaclesServiceResponse():
      ok(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->ok >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ok);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->ok =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ok);
     return offset;
    }

    virtual const char * getType() override { return OBSTACLESSERVICE; };
    virtual const char * getMD5() override { return "ebb43f57b7820ff999dc120ba80eb7d8"; };

  };

  class ObstaclesService {
    public:
    typedef ObstaclesServiceRequest Request;
    typedef ObstaclesServiceResponse Response;
  };

}
#endif
