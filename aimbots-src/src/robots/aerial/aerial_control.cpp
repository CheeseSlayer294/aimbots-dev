#ifdef TARGET_AERIAL

#include "drivers.hpp"
#include "drivers_singleton.hpp"
#include "utils/common_types.hpp"
//
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//
#include "subsystems/feeder/burst_feeder_command.hpp"
#include "subsystems/feeder/feeder.hpp"
#include "subsystems/feeder/run_feeder_command.hpp"
#include "subsystems/feeder/stop_feeder_command.hpp"
#include "subsystems/feeder/unjam_feeder_command.hpp"
//
#include "subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "subsystems/gimbal/gimbal_chase_command.hpp"
#include "subsystems/gimbal/gimbal_control_command.hpp"
//
#include "subsystems/shooter/brake_shooter_command.hpp"
#include "subsystems/shooter/run_shooter_command.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/stop_shooter_command.hpp"
#include "subsystems/shooter/stop_shooter_comprised_command.hpp"
//
using namespace src::Feeder;
using namespace src::Gimbal;
using namespace src::Shooter;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;

namespace AerialControl {

// Define subsystems here ------------------------------------------------
FeederSubsystem feeder(drivers());
GimbalSubsystem gimbal(drivers());
ShooterSubsystem shooter(drivers());

// Robot Specific Controllers ------------------------------------------------
GimbalChassisRelativeController gimbalController(&gimbal);

// Define commands here ---------------------------------------------------
GimbalControlCommand gimbalControlCommand(drivers(), &gimbal, &gimbalController, USER_JOYSTICK_YAW_SCALAR, USER_JOYSTICK_PITCH_SCALAR);
GimbalControlCommand gimbalControlCommandTwo(drivers(), &gimbal, &gimbalController, USER_JOYSTICK_YAW_SCALAR, USER_JOYSTICK_PITCH_SCALAR);
GimbalChaseCommand gimbalChaseCommand(drivers(), &gimbal, &gimbalController);

RunFeederCommand runFeederCommand(drivers(), &feeder);
StopFeederCommand stopFeederCommand(drivers(), &feeder);
BurstFeederCommand burstFeederCommand(drivers(), &feeder);
UnjamFeederCommand unjamFeederCommand(drivers(), &feeder);

RunShooterCommand runShooterCommand(drivers(), &shooter);
RunShooterCommand runShooterWithFeederCommand(drivers(), &shooter);
StopShooterComprisedCommand stopShooterComprisedCommand(drivers(), &shooter);

// Define command mappings here -------------------------------------------
// Enables both chassis and gimbal manual control
HoldCommandMapping leftSwitchMid(
    drivers(),
    {&gimbalControlCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

HoldCommandMapping leftSwitchUp(
    drivers(),
    {&unjamFeederCommand,&gimbalControlCommandTwo},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// Runs shooter only
HoldCommandMapping rightSwitchMid(
    drivers(),
    {&runShooterCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Runs shooter with feeder
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
    drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchMid);
}

}  // namespace AerialControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    AerialControl::initializeSubsystems();
    AerialControl::registerSubsystems(drivers);
    AerialControl::setDefaultCommands(drivers);
    AerialControl::startupCommands(drivers);
    AerialControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif  // TARGET_AERIAL