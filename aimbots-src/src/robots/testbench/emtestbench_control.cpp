#ifdef TARGET_EMTESTBENCH

#include "utils/common_types.hpp"

#include "drivers.hpp"
#include "drivers_singleton.hpp"

//
#include "informants/transformers/robot_frames.hpp"
#include "utils/ballistics_solver.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
//
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//
#include "subsystems/vel_tester/run_velocity_command.hpp"
#include "subsystems/vel_tester/vel_tester.hpp"
using namespace src::vel_tester;
// TODO: using namespace insertSubsystemNamespaceHere

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;
using namespace tap::communication::serial;

namespace EMTestbenchControl {

BarrelID currentBarrel = BarrelID::TURRET_17MM_1;

src::Utils::RefereeHelperTurreted refHelper(drivers(), currentBarrel, 10);

// Define subsystems here ------------------------------------------------
Velocity_Control vel_tester(drivers());

// Ballistics Solver -------------------------------------------------------
src::Utils::Ballistics::BallisticsSolver ballisticsSolver(drivers(), BARREL_POSITION_FROM_GIMBAL_ORIGIN);

// Configs -----------------------------------------------------------------

// Define commands here ---------------------------------------------------
// TODO: Commands
Velocity_Control_Command Velocity_Control_Command(drivers(), &vel_tester);

// Define command mappings here -------------------------------------------

// HoldCommandMapping rightClickMouse(
//     drivers(),
//     {&},
//     RemoteMapState(RemoteMapState::MouseButton::LEFT));

// TODO: Create Command Mappings
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&Velocity_Control_Command},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    // TODO: Register Subsystem
    drivers->commandScheduler.registerSubsystem(&vel_tester);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    // TODO: Initialize Subsystem
    vel_tester.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {}

// Set commands scheduled on startup
void startupCommands(src::Drivers *) {
    // no startup commands should be set
    // yet...
    // TODO: Possibly add some sort of hardware test command
    //       that will move all the parts so we
    //       can make sure they're fully operational.
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    // TODO: Register Command Mappings
    drivers->commandMapper.addMap(&leftSwitchUp);
}

}  // namespace EMTestbenchControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    EMTestbenchControl::initializeSubsystems();
    EMTestbenchControl::registerSubsystems(drivers);
    EMTestbenchControl::setDefaultCommands(drivers);
    EMTestbenchControl::startupCommands(drivers);
    EMTestbenchControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif  // TARGET_STANDARD
