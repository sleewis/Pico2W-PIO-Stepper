#pragma once
#include <Arduino.h>
#include "types.h"

// ---- Debug ----
#ifndef DEBUG
#define DEBUG 1
#endif

#define BUFFER_SIZE 512u

// ---- Pins ----
constexpr uint HOME_X_PIN = 20;
constexpr uint HOME_Y_PIN = 21;

// ---- PIO/DIR invert mask ----
#define DIR_INV_MASK (0u) // bv: (1u<<1) om Y te inverteren

// ---- Steps per unit ----
constexpr float X_STEPS_PER_MM = 10.0f;
constexpr float Y_STEPS_PER_MM = 10.0f;
constexpr float Z_STEPS_PER_MM = 10.0f;

constexpr int32_t X_STEPS_PER_REV = 200 * 2 * 30;
constexpr int32_t Y_STEPS_PER_REV = 200 * 2 * 30;
constexpr int32_t Z_STEPS_PER_REV = 200 * 2 * 30;

// ---- Planner / blending ----
constexpr float A_MAX_STEPS_S2   = 15000.0f;
constexpr float BLEND_LEN_STEPS  = 50.0f;
constexpr float MIN_CORNER_SPEED = 0.0f;
constexpr float MAX_FEED_STEPS_S = 12000.0f;



// ---- Per-axis limits (runtime variabelen, gedefinieerd in config.cpp) ----
extern AxisLimits LIMIT_X;
extern AxisLimits LIMIT_Y;
extern AxisLimits LIMIT_Z;
