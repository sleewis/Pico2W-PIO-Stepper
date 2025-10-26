#include "config.h"

AxisLimits LIMIT_X = {
  /*steps*/ { false, 0, 0 },
  /*mm   */ { false, 0, 0 },
  /*deg  */ { true , 0.0f, 360.0f },
  /*wrap */ true
};

AxisLimits LIMIT_Y = {
  /*steps*/ { false, 0, 0 },
  /*mm   */ { false, 0, 0 },
  /*deg  */ { true , 0.0f, 90.0f },
  /*wrap */ false
};

AxisLimits LIMIT_Z = {
  /*steps*/ { false, 0, 0 },
  /*mm   */ { false, 0, 0 },
  /*deg  */ { false, 0.0f, 0.0f },
  /*wrap */ false
};
