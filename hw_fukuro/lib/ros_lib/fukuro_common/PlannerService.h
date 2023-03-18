#ifndef _ROS_SERVICE_PlannerService_h
#define _ROS_SERVICE_PlannerService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fukuro_common
{

static const char PLANNERSERVICE[] = "fukuro_common/PlannerService";

  class PlannerServiceRequest : public ros::Msg
  {
    public:
      typedef float _goal_x_type;
      _goal_x_type goal_x;
      typedef float _goal_y_type;
      _goal_y_type goal_y;

    PlannerServiceRequest():
      goal_x(0),
      goal_y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->goal_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->goal_y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->goal_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->goal_y));
     return offset;
    }

    virtual const char * getType() override { return PLANNERSERVICE; };
    virtual const char * getMD5() override { return "13b965790a69b58a06825eaf607cdbe7"; };

  };

  class PlannerServiceResponse : public ros::Msg
  {
    public:
      typedef uint8_t _ok_type;
      _ok_type ok;

    PlannerServiceResponse():
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

    virtual const char * getType() override { return PLANNERSERVICE; };
    virtual const char * getMD5() override { return "ebb43f57b7820ff999dc120ba80eb7d8"; };

  };

  class PlannerService {
    public:
    typedef PlannerServiceRequest Request;
    typedef PlannerServiceResponse Response;
  };

}
#endif
