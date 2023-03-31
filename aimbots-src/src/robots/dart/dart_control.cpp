#ifdef TARGET_DART

#include "utils/common_types.hpp"

#include "drivers.hpp"
#include "drivers_singleton.hpp"
//
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//
#include "subsystems/feeder/feeder.hpp"
#include "subsystems/feeder/full_auto_feeder_command.hpp"
#include "subsystems/feeder/stop_feeder_command.hpp"
//
#include "subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp"
// #include "subsystems/gimbal/controllers/gimbal_field_relative_controller.hpp"
#include "subsystems/gimbal/gimbal.hpp"
// #include "subsystems/gimbal/gimbal_chase_command.hpp"
#include "subsystems/gimbal/gimbal_control_command.hpp"
// #include "subsystems/gimbal/gimbal_field_relative_control_command.hpp"
//
#include "subsystems/shooter/brake_shooter_command.hpp"
#include "subsystems/shooter/run_shooter_command.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/stop_shooter_command.hpp"
#include "subsystems/shooter/stop_shooter_comprised_command.hpp"

using namespace src::Feeder;
using namespace src::Gimbal;
using namespace src::Shooter;
// using namespace src::Hopper;
// using namespace src::GUI;

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

namespace DartControl {

// Define subsystems here ------------------------------------------------
FeederSubsystem feeder(drivers());
GimbalSubsystem gimbal(drivers());
ShooterSubsystem shooter(drivers());

// Robot Specific Controllers ------------------------------------------------
GimbalChassisRelativeController gimbalChassisRelativeController(&gimbal);

// Define commands here ---------------------------------------------------
GimbalControlCommand gimbalControlCommand(drivers(), &gimbal, &gimbalChassisRelativeController);

FullAutoFeederCommand runFeederCommand(drivers(), &feeder, FEEDER_DEFAULT_RPM, 0.80f);
FullAutoFeederCommand runFeederCommandFromMouse(drivers(), &feeder, FEEDER_DEFAULT_RPM, 0.80f);
StopFeederCommand stopFeederCommand(drivers(), &feeder);

RunShooterCommand runShooterCommand(drivers(), &shooter);
RunShooterCommand runShooterWithFeederCommand(drivers(), &shooter);
StopShooterComprisedCommand stopShooterComprisedCommand(drivers(), &shooter);

// Define command mappings here -------------------------------------------

// Runs shooter only and closes hopper
HoldCommandMapping rightSwitchMid(drivers(), {&runShooterCommand}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Runs shooter with feeder and closes hopper
HoldCommandMapping rightSwitchUp(
    drivers(),
    {&runFeederCommand, &runShooterWithFeederCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&feeder);
    drivers->commandScheduler.registerSubsystem(&gimbal);
    drivers->commandScheduler.registerSubsystem(&shooter);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    feeder.initialize();
    gimbal.initialize();
    shooter.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    feeder.setDefaultCommand(&stopFeederCommand);
    shooter.setDefaultCommand(&stopShooterComprisedCommand);
}

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
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchMid);
}

}  // namespace DartControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    DartControl::initializeSubsystems();
    DartControl::registerSubsystems(drivers);
    DartControl::setDefaultCommands(drivers);
    DartControl::startupCommands(drivers);
    DartControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif  // TARGET_DART