#pragma once

#if defined(TARGET_AERIAL)
#include "robots/aerial/aerial_constants.hpp"
#include "robots/aerial/aerial_control_interface.hpp"

#elif defined(TARGET_ENGINEER)
#include "robots/engineer/engineer_constants.hpp"

#elif defined(TARGET_SWERVE_ENGINEER)
#include "robots/engineer/swerve_engineer_constants.hpp"

#elif defined(TARGET_HERO)
#include "robots/hero/hero_constants.hpp"

#elif defined(TARGET_SENTRY)
#include "robots/sentry/sentry_constants.hpp"
#include "robots/sentry/sentry_control_interface.hpp"

#elif defined(TARGET_STANDARD)
#include "robots/standard/standard_constants.hpp"
#include "robots/standard/standard_control_interface.hpp"

#elif defined(TARGET_SWERVE_STANDARD)
#include "robots/standard/swerve_standard_constants.hpp"

#endif