#pragma once

#if defined(TARGET_AERIAL)
#define NO_CHASSIS
#define SHOOTER
#define NO_HOPPER
#define GIMBAL
#define FEEDER
#include "robots/aerial/aerial_constants.hpp"

#elif defined(TARGET_ENGINEER)
#define CHASSIS
#define MECANUM
#define NO_SHOOTER
#define NO_HOPPER
#define NO_GIMBAL
#define NO_FEEDER
#include "robots/engineer/engineer_constants.hpp"

#elif defined(TARGET_SWERVE_ENGINEER)
#define CHASSIS
#define SWERVE
#define NO_SHOOTER
#define NO_HOPPER
#define NO_GIMBAL
#define NO_FEEDER
#include "robots/engineer-swerve/swerve_engineer_constants.hpp"

#elif defined(TARGET_HERO)
#define CHASSIS
#define OMNI
#define SHOOTER
#define HOPPER
#define GIMBAL
#define FEEDER
#include "robots/hero/hero_constants.hpp"

#elif defined(TARGET_SENTRY)
#define CHASSIS
#define OMNI
#define SHOOTER
#define HOPPER
#define GIMBAL
#define FEEDER
#include "robots/sentry/sentry_constants.hpp"

#elif defined(TARGET_STANDARD)
#define CHASSIS
#define MECANUM
#define SHOOTER
#define HOPPER
#define GIMBAL
#define FEEDER
#include "robots/standard/standard_constants.hpp"
#include "robots/standard/standard_control_interface.hpp"

#elif defined(TARGET_SWERVE_STANDARD)
#define CHASSIS
#define SWERVE
#define SHOOTER
#define HOPPER
#define GIMBAL
#define FEEDER
#include "robots/standard/swerve_standard_constants.hpp"

#elif defined(TARGET_DART)
#define NO_CHASSIS
#define SHOOTER
#define NO_HOPPER
#define GIMBAL
#define FEEDER

// #else
// #include "robots/standard/standard_constants.hpp"
// #define TARGET_UNDEFINED

#endif