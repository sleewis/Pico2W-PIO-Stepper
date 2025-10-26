#pragma once
#include <math.h>
#include "config.h"

static inline float clampf(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }

static inline float angle_between_vec(float ax,float ay,float az, float bx,float by,float bz){
  float la = sqrtf(ax*ax+ay*ay+az*az), lb = sqrtf(bx*bx+by*by+bz*bz);
  if(la<1e-6f || lb<1e-6f) return 0.0f;
  float dot = clampf((ax*bx+ay*by+az*bz)/(la*lb), -1.0f, 1.0f);
  return acosf(dot);
}

static inline float corner_speed_limit(float theta){
  float s = sinf(0.5f*theta); if(s<1e-6f) return 1e9f;
  return sqrtf((A_MAX_STEPS_S2 * BLEND_LEN_STEPS) / (2.0f*s));
}

static inline uint32_t period_us_from_speed(float v_steps_s){
  if(v_steps_s < 1.0f) v_steps_s = 1.0f;
  return (uint32_t)lrintf(1e6f / v_steps_s);
}
