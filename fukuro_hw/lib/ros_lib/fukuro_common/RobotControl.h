#ifndef _ROS_fukuro_common_RobotControl_h
#define _ROS_fukuro_common_RobotControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"

namespace fukuro_common
{

  class RobotControl : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose2D _target_pose_type;
      _target_pose_type target_pose;
      typedef std_msgs::String _option_type;
      _option_type option;
      typedef std_msgs::String _control_type;
      _control_type control;
      typedef int32_t _dribbling_mode_type;
      _dribbling_mode_type dribbling_mode;
      typedef bool _plan_type;
      _plan_type plan;
      typedef bool _approach_ball_type;
      _approach_ball_type approach_ball;
      typedef bool _motor_brake_type;
      _motor_brake_type motor_brake;
      typedef bool _absolute_move_type;
      _absolute_move_type absolute_move;

    RobotControl():
      target_pose(),
      option(),
      control(),
      dribbling_mode(0),
      plan(0),
      approach_ball(0),
      motor_brake(0),
      absolute_move(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->target_pose.serialize(outbuffer + offset);
      offset += this->option.serialize(outbuffer + offset);
      offset += this->control.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_dribbling_mode;
      u_dribbling_mode.real = this->dribbling_mode;
      *(outbuffer + offset + 0) = (u_dribbling_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbling_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbling_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbling_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbling_mode);
      union {
        bool real;
        uint8_t base;
      } u_plan;
      u_plan.real = this->plan;
      *(outbuffer + offset + 0) = (u_plan.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->plan);
      union {
        bool real;
        uint8_t base;
      } u_approach_ball;
      u_approach_ball.real = this->approach_ball;
      *(outbuffer + offset + 0) = (u_approach_ball.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->approach_ball);
      union {
        bool real;
        uint8_t base;
      } u_motor_brake;
      u_motor_brake.real = this->motor_brake;
      *(outbuffer + offset + 0) = (u_motor_brake.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motor_brake);
      union {
        bool real;
        uint8_t base;
      } u_absolute_move;
      u_absolute_move.real = this->absolute_move;
      *(outbuffer + offset + 0) = (u_absolute_move.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->absolute_move);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->target_pose.deserialize(inbuffer + offset);
      offset += this->option.deserialize(inbuffer + offset);
      offset += this->control.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_dribbling_mode;
      u_dribbling_mode.base = 0;
      u_dribbling_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbling_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbling_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbling_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbling_mode = u_dribbling_mode.real;
      offset += sizeof(this->dribbling_mode);
      union {
        bool real;
        uint8_t base;
      } u_plan;
      u_plan.base = 0;
      u_plan.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->plan = u_plan.real;
      offset += sizeof(this->plan);
      union {
        bool real;
        uint8_t base;
      } u_approach_ball;
      u_approach_ball.base = 0;
      u_approach_ball.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->approach_ball = u_approach_ball.real;
      offset += sizeof(this->approach_ball);
      union {
        bool real;
        uint8_t base;
      } u_motor_brake;
      u_motor_brake.base = 0;
      u_motor_brake.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motor_brake = u_motor_brake.real;
      offset += sizeof(this->motor_brake);
      union {
        bool real;
        uint8_t base;
      } u_absolute_move;
      u_absolute_move.base = 0;
      u_absolute_move.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->absolute_move = u_absolute_move.real;
      offset += sizeof(this->absolute_move);
     return offset;
    }

    virtual const char * getType() override { return "fukuro_common/RobotControl"; };
    virtual const char * getMD5() override { return "443a92a508b155bf2adcee0c839453e2"; };

  };

}
#endif
