#ifndef _ROS_fukuro_common_RobotControlInfo_h
#define _ROS_fukuro_common_RobotControlInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"

namespace fukuro_common
{

  class RobotControlInfo : public ros::Msg
  {
    public:
      typedef float _error_radius_type;
      _error_radius_type error_radius;
      typedef float _error_angle_type;
      _error_angle_type error_angle;
      typedef geometry_msgs::Pose2D _setpoint_type;
      _setpoint_type setpoint;
      typedef bool _plan_type;
      _plan_type plan;

    RobotControlInfo():
      error_radius(0),
      error_angle(0),
      setpoint(),
      plan(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->error_radius);
      offset += serializeAvrFloat64(outbuffer + offset, this->error_angle);
      offset += this->setpoint.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_plan;
      u_plan.real = this->plan;
      *(outbuffer + offset + 0) = (u_plan.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->plan);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error_radius));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error_angle));
      offset += this->setpoint.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_plan;
      u_plan.base = 0;
      u_plan.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->plan = u_plan.real;
      offset += sizeof(this->plan);
     return offset;
    }

    virtual const char * getType() override { return "fukuro_common/RobotControlInfo"; };
    virtual const char * getMD5() override { return "3d219701cbb4b4f70bacc052c397c3a7"; };

  };

}
#endif
