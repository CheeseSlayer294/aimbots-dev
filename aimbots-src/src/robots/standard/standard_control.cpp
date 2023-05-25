#ifdef TARGET_STANDARD

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
#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_manual_drive_command.hpp"
#include "subsystems/chassis/chassis_toggle_drive_command.hpp"
#include "subsystems/chassis/chassis_tokyo_command.hpp"
//
#include "subsystems/feeder/feeder.hpp"
#include "subsystems/feeder/full_auto_feeder_command.hpp"
#include "subsystems/feeder/stop_feeder_command.hpp"
//
#include "subsystems/gimbal/controllers/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/controllers/gimbal_field_relative_controller.hpp"
#include "subsystems/gimbal/gimbal.hpp"
#include "subsystems/gimbal/gimbal_chase_command.hpp"
#include "subsystems/gimbal/gimbal_control_command.hpp"
#include "subsystems/gimbal/gimbal_field_relative_control_command.hpp"
//
#include "subsystems/shooter/brake_shooter_command.hpp"
#include "subsystems/shooter/run_shooter_command.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/stop_shooter_command.hpp"
#include "subsystems/shooter/stop_shooter_comprised_command.hpp"
//
#include "subsystems/hopper/close_hopper_command.hpp"
#include "subsystems/hopper/hopper.hpp"
#include "subsystems/hopper/open_hopper_command.hpp"
#include "subsystems/hopper/toggle_hopper_command.hpp"
//
#include "informants/communication/communication_response_handler.hpp"
#include "informants/communication/communication_response_subsytem.hpp"
//
#include "utils/display/client_display_command.hpp"
#include "utils/display/client_display_subsystem.hpp"
//

using namespace src::Chassis;
using namespace src::Feeder;
using namespace src::Gimbal;
using namespace src::Shooter;
using namespace src::Hopper;
using namespace src::Communication;
using namespace src::RobotStates;
using namespace src::utils::display;

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

namespace StandardControl {

// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());
FeederSubsystem feeder(drivers());
GimbalSubsystem gimbal(drivers());
ShooterSubsystem shooter(drivers());
HopperSubsystem hopper(drivers());
CommunicationResponseSubsytem response(*drivers());
ClientDisplaySubsystem clientDisplay(*drivers());

// Robot Specific Controllers ------------------------------------------------
GimbalChassisRelativeController gimbalChassisRelativeController(&gimbal);
GimbalFieldRelativeController gimbalFieldRelativeController(drivers(), &gimbal);

// Define commands here ---------------------------------------------------
ChassisManualDriveCommand chassisManualDriveCommand(drivers(), &chassis);
ChassisToggleDriveCommand chassisToggleDriveCommand(drivers(), &chassis, &gimbal);
ChassisTokyoCommand chassisTokyoCommand(drivers(), &chassis, &gimbal);

GimbalControlCommand gimbalControlCommand(drivers(), &gimbal, &gimbalChassisRelativeController);
GimbalFieldRelativeControlCommand gimbalFieldRelativeControlCommand(drivers(), &gimbal, &gimbalFieldRelativeController);
GimbalFieldRelativeControlCommand gimbalFieldRelativeControlCommand2(drivers(), &gimbal, &gimbalFieldRelativeController);
GimbalChaseCommand gimbalChaseCommand(drivers(), &gimbal, &gimbalFieldRelativeController);
GimbalChaseCommand gimbalChaseCommand2(drivers(), &gimbal, &gimbalFieldRelativeController);

FullAutoFeederCommand runFeederCommand(drivers(), &feeder, FEEDER_DEFAULT_RPM, 0.80f);
FullAutoFeederCommand runFeederCommandFromMouse(drivers(), &feeder, FEEDER_DEFAULT_RPM, 0.80f);
StopFeederCommand stopFeederCommand(drivers(), &feeder);

RunShooterCommand runShooterCommand(drivers(), &shooter);
RunShooterCommand runShooterWithFeederCommand(drivers(), &shooter);
StopShooterComprisedCommand stopShooterComprisedCommand(drivers(), &shooter);

OpenHopperCommand openHopperCommand(drivers(), &hopper);
OpenHopperCommand openHopperCommand2(drivers(), &hopper);
CloseHopperCommand closeHopperCommand(drivers(), &hopper);
CloseHopperCommand closeHopperCommand2(drivers(), &hopper);
ToggleHopperCommand toggleHopperCommand(drivers(), &hopper);

CommunicationResponseHandler responseHandler(*drivers());

// client display
ClientDisplayCommand clientDisplayCommand(*drivers(), drivers()->commandScheduler, clientDisplay);

// Define command mappings here -------------------------------------------
HoldCommandMapping leftSwitchMid(
    drivers(),
    {&chassisToggleDriveCommand, &gimbalFieldRelativeControlCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::MID));

// Enables both chassis and gimbal control and closes hopper
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&chassisTokyoCommand, &gimbalFieldRelativeControlCommand2},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

// opens hopper
HoldCommandMapping rightSwitchDown(drivers(), {&openHopperCommand}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::DOWN));

// Runs shooter only and closes hopper
HoldCommandMapping rightSwitchMid(
    drivers(),
    {&runShooterCommand, &closeHopperCommand},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));

// Runs shooter with feeder and closes hopper
HoldRepeatCommandMapping rightSwitchUp(
    drivers(),
    {&runFeederCommand, &runShooterWithFeederCommand, &closeHopperCommand2},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP),
    true);

HoldCommandMapping leftClickMouse(drivers(), {&runFeederCommandFromMouse}, RemoteMapState(RemoteMapState::MouseButton::LEFT));

// The user can press b+ctrl when the remote right switch is in the down position to restart the
// client display command. This is necessary since we don't know when the robot is connected to the
// server and thus don't know when to start sending the initial HUD graphics.
PressCommandMapping bCtrlPressed(drivers(), {&clientDisplayCommand}, RemoteMapState({Remote::Key::B}));

// HoldCommandMapping rightClickMouse(
//     drivers(),
//     {&},
//     RemoteMapState(RemoteMapState::MouseButton::LEFT));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&feeder);
    drivers->commandScheduler.registerSubsystem(&gimbal);
    drivers->commandScheduler.registerSubsystem(&shooter);
    drivers->commandScheduler.registerSubsystem(&hopper);
    drivers->commandScheduler.registerSubsystem(&response);
    drivers->commandScheduler.registerSubsystem(&clientDisplay);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    chassis.initialize();
    feeder.initialize();
    gimbal.initialize();
    shooter.initialize();
    hopper.initialize();
    response.initialize();
    clientDisplay.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    feeder.setDefaultCommand(&stopFeederCommand);
    shooter.setDefaultCommand(&stopShooterComprisedCommand);
}

// Set commands scheduled on startup
void startupCommands(src::Drivers *drivers) {
    // no startup commands should be set
    // yet...
    // TODO: Possibly add some sort of hardware test command
    //       that will move all the parts so we
    //       can make sure they're fully operational.
    drivers->refSerial.attachRobotToRobotMessageHandler(SENTRY_RESPONSE_MESSAGE_ID, &responseHandler);
    drivers->commandScheduler.addCommand(&clientDisplayCommand);
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&leftSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchMid);
    drivers->commandMapper.addMap(&rightSwitchDown);
    drivers->commandMapper.addMap(&leftClickMouse);
}

}  // namespace StandardControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    StandardControl::initializeSubsystems();
    StandardControl::registerSubsystems(drivers);
    StandardControl::setDefaultCommands(drivers);
    StandardControl::startupCommands(drivers);
    StandardControl::registerIOMappings(drivers);
}
}  // namespace src::Control

//temp

#endif  // TARGET_STANDARD
