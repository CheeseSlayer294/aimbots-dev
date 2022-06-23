#ifdef TARGET_SWERVE_ENGINEER

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
#include "subsystems/chassis/chassis.hpp"
#include "subsystems/chassis/chassis_manual_drive_command.hpp"


#include "subsystems/solenoid/solenoid.hpp"
#include "subsystems/solenoid/solenoid_controller.hpp"

using namespace src::Chassis;
using namespace src::Solenoid;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;

namespace EngineerControl {

// Define subsystems here ------------------------------------------------
ChassisSubsystem chassis(drivers());
SolenoidSubsytem solenoid(drivers());

// Define commands here ---------------------------------------------------
ChassisManualDriveCommand chassisManualDriveCommand(drivers(), &chassis);
SolenoidController solenoidControllerC1(drivers(), &solenoid, "C1");
SolenoidController solenoidControllerC2(drivers(), &solenoid, "C2");
SolenoidController solenoidControllerC3(drivers(), &solenoid, "C3");

// Define command mappings here -------------------------------------------
HoldCommandMapping leftSwitchUp(
    drivers(),
    {&chassisManualDriveCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));

HoldCommandMapping rightSwitchUp(
    drivers(),
    {&solenoidControllerC1},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&solenoid);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    chassis.initialize();
    solenoid.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    // no default commands should be set
}

// Set commands scheduled on startup
void startupCommands(src::Drivers *drivers) {
    // no startup commands should be set
    // yet...
    // TODO: Possibly add some sort of hardware test command
    //       that will move all the parts so we
    //       can make sure they're fully operational.
}

// Register IO mappings here -----------------------------------------------
void registerIOMappings(src::Drivers *drivers) {
    drivers->commandMapper.addMap(&leftSwitchUp);
    drivers->commandMapper.addMap(&rightSwitchUp);
}

}  // namespace StandardControl

namespace src::Control {
    // Initialize subsystems ---------------------------------------------------
    void initializeSubsystemCommands(src::Drivers * drivers) {
        EngineerControl::initializeSubsystems();
        EngineerControl::registerSubsystems(drivers);
        EngineerControl::setDefaultCommands(drivers);
        EngineerControl::startupCommands(drivers);
        EngineerControl::registerIOMappings(drivers);
    }
}  // namespace src::Control

#endif  //TARGET_ENGINEER
