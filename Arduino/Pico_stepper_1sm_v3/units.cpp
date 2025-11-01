#include "units.h"
#include "config.h"

volatile UnitMode g_units = UNITS_STEPS;

static inline float clampf_u(float v, float lo, float hi){
  return (v<lo)?lo:((v>hi)?hi:v);
}

static float clamp_units_value(float v, const UnitLimit& L, bool wrap=false){
  if(!L.enabled) return v;
  if(wrap){
    float t = fmodf(v, 360.0f); if(t<0) t += 360.0f; v = t;
  }
  return clampf_u(v, L.min_v, L.max_v);
}

// steps -> units
float steps_to_units_X(int32_t s){
  switch(g_units){ case UNITS_MM: return s/X_STEPS_PER_MM;
    case UNITS_DEG: return 360.0f*s/(float)X_STEPS_PER_REV; default: return (float)s; }
}
float steps_to_units_Y(int32_t s){
  switch(g_units){ case UNITS_MM: return s/Y_STEPS_PER_MM;
    case UNITS_DEG: return 360.0f*s/(float)Y_STEPS_PER_REV; default: return (float)s; }
}
float steps_to_units_Z(int32_t s){
  switch(g_units){ case UNITS_MM: return s/Z_STEPS_PER_MM;
    case UNITS_DEG: return 360.0f*s/(float)Z_STEPS_PER_REV; default: return (float)s; }
}

// units -> steps
int32_t units_to_steps_X(float u){
  switch(g_units){
    case UNITS_MM: return (int32_t)lrintf(u*X_STEPS_PER_MM);
    case UNITS_DEG: return (int32_t)lrintf((u/360.0f)*X_STEPS_PER_REV);
    default: return (int32_t)lrintf(u);
  }
}

int32_t units_to_steps_Y(float u){
  switch(g_units){
    case UNITS_MM: return (int32_t)lrintf(u*Y_STEPS_PER_MM);
    case UNITS_DEG: return (int32_t)lrintf((u/360.0f)*Y_STEPS_PER_REV);
    default: return (int32_t)lrintf(u);
  }
}

int32_t units_to_steps_Z(float u){
  switch(g_units){
    case UNITS_MM: return (int32_t)lrintf(u*Z_STEPS_PER_MM);
    case UNITS_DEG: return (int32_t)lrintf((u/360.0f)*Z_STEPS_PER_REV);
    default: return (int32_t)lrintf(u);
  }
}

// feed helpers
float steps_per_unit_X(){
  switch(g_units){
    case UNITS_MM: return X_STEPS_PER_MM;
    case UNITS_DEG: return (float)X_STEPS_PER_REV/360.0f;
    default: return 1.0f;
  }
}

float steps_per_unit_Y(){
  switch(g_units){
    case UNITS_MM: return Y_STEPS_PER_MM;
    case UNITS_DEG: return (float)Y_STEPS_PER_REV/360.0f;
    default: return 1.0f;
  }
}

float steps_per_unit_Z(){
  switch(g_units){
    case UNITS_MM: return Z_STEPS_PER_MM;
    case UNITS_DEG: return (float)Z_STEPS_PER_REV/360.0f;
    default: return 1.0f;
  }
}

// caps
float apply_caps_X(float v){
  switch(g_units){
    case UNITS_STEPS: return clamp_units_value(v,LIMIT_X.steps);
    case UNITS_MM: return clamp_units_value(v,LIMIT_X.mm);
    case UNITS_DEG: return clamp_units_value(v,LIMIT_X.deg);
  } return v;
}

float apply_caps_Y(float v){
  switch(g_units){
    case UNITS_STEPS: return clamp_units_value(v,LIMIT_Y.steps);
    case UNITS_MM: return clamp_units_value(v,LIMIT_Y.mm);
    case UNITS_DEG: return clamp_units_value(v,LIMIT_Y.deg);
  } return v;
}

float apply_caps_Z(float v){
  switch(g_units){
    case UNITS_STEPS: return clamp_units_value(v,LIMIT_Z.steps);
    case UNITS_MM: return clamp_units_value(v,LIMIT_Z.mm);
    case UNITS_DEG: return clamp_units_value(v,LIMIT_Z.deg);
  } return v;
}

// UI helpers
const char* units_label(){ return (g_units==UNITS_STEPS)?"steps":(g_units==UNITS_MM)?"mm":"deg"; }
String toStringUnits(float v){
  if(g_units==UNITS_MM){ char b[24]; dtostrf(v,0,2,b); return String(b); }
  long vi = lroundf(v); char b[24]; ltoa(vi,b,10); return String(b);
}

void serial_print_units(float v){
  if(g_units==UNITS_MM) SD.print(v,2);
  else SD.print((int32_t)lrintf(v));
}
