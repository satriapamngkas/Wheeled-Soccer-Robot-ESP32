#ifndef _ROS_fukuro_common_STMData_h
#define _ROS_fukuro_common_STMData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "fukuro_common/MotorVel.h"
#include "fukuro_common/RobotVel.h"

namespace fukuro_common
{

  class STMData : public ros::Msg
  {
    public:
      typedef fukuro_common::MotorVel _encoder_type;
      _encoder_type encoder;
      typedef fukuro_common::RobotVel _freeenc_type;
      _freeenc_type freeenc;
      typedef bool _ready_kick_type;
      _ready_kick_type ready_kick;
      typedef bool _ir_type;
      _ir_type ir;
      typedef bool _garis1_type;
      _garis1_type garis1;
      typedef bool _garis2_type;
      _garis2_type garis2;
      typedef bool _garis3_type;
      _garis3_type garis3;
      typedef bool _garis4_type;
      _garis4_type garis4;

    STMData():
      encoder(),
      freeenc(),
      ready_kick(0),
      ir(0),
      garis1(0),
      garis2(0),
      garis3(0),
      garis4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->encoder.serialize(outbuffer + offset);
      offset += this->freeenc.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_ready_kick;
      u_ready_kick.real = this->ready_kick;
      *(outbuffer + offset + 0) = (u_ready_kick.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ready_kick);
      union {
        bool real;
        uint8_t base;
      } u_ir;
      u_ir.real = this->ir;
      *(outbuffer + offset + 0) = (u_ir.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ir);
      union {
        bool real;
        uint8_t base;
      } u_garis1;
      u_garis1.real = this->garis1;
      *(outbuffer + offset + 0) = (u_garis1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->garis1);
      union {
        bool real;
        uint8_t base;
      } u_garis2;
      u_garis2.real = this->garis2;
      *(outbuffer + offset + 0) = (u_garis2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->garis2);
      union {
        bool real;
        uint8_t base;
      } u_garis3;
      u_garis3.real = this->garis3;
      *(outbuffer + offset + 0) = (u_garis3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->garis3);
      union {
        bool real;
        uint8_t base;
      } u_garis4;
      u_garis4.real = this->garis4;
      *(outbuffer + offset + 0) = (u_garis4.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->garis4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->encoder.deserialize(inbuffer + offset);
      offset += this->freeenc.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_ready_kick;
      u_ready_kick.base = 0;
      u_ready_kick.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ready_kick = u_ready_kick.real;
      offset += sizeof(this->ready_kick);
      union {
        bool real;
        uint8_t base;
      } u_ir;
      u_ir.base = 0;
      u_ir.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ir = u_ir.real;
      offset += sizeof(this->ir);
      union {
        bool real;
        uint8_t base;
      } u_garis1;
      u_garis1.base = 0;
      u_garis1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->garis1 = u_garis1.real;
      offset += sizeof(this->garis1);
      union {
        bool real;
        uint8_t base;
      } u_garis2;
      u_garis2.base = 0;
      u_garis2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->garis2 = u_garis2.real;
      offset += sizeof(this->garis2);
      union {
        bool real;
        uint8_t base;
      } u_garis3;
      u_garis3.base = 0;
      u_garis3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->garis3 = u_garis3.real;
      offset += sizeof(this->garis3);
      union {
        bool real;
        uint8_t base;
      } u_garis4;
      u_garis4.base = 0;
      u_garis4.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->garis4 = u_garis4.real;
      offset += sizeof(this->garis4);
     return offset;
    }

    virtual const char * getType() override { return "fukuro_common/STMData"; };
    virtual const char * getMD5() override { return "1f74160987d11b971db5bf3af08f3ce7"; };

  };

}
#endif
