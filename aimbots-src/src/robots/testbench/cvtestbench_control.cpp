#ifdef TARGET_CVTESTBENCH

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
#include "subsystems/feeder/feeder.hpp"
//TODO: Add Command Includes Here
//
#include "subsystems/shooter/shooter.hpp"
//TODO: ADd Command Includes Here

using namespace src::Feeder;
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
using namespace tap::communication::serial;

namespace CVTestbenchControl {

BarrelID currentBarrel = BarrelID::TURRET_17MM_1;

src::Utils::RefereeHelperTurreted refHelper(drivers(), currentBarrel, 10);

// Define subsystems here ------------------------------------------------
FeederSubsystem feeder(drivers());
ShooterSubsystem shooter(drivers(), &refHelper);

// Ballistics Solver -------------------------------------------------------
src::Utils::Ballistics::BallisticsSolver ballisticsSolver(drivers(), BARREL_POSITION_FROM_GIMBAL_ORIGIN);

// Configs -----------------------------------------------------------------


// Define commands here ---------------------------------------------------


// Define command mappings here -------------------------------------------

/*HoldCommandMapping rightSwitchMid(
    drivers(),
    {&insertCommandHere},
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID));*/

// Register subsystems here -----------------------------------------------
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&feeder);
    drivers->commandScheduler.registerSubsystem(&shooter);
}

// Initialize subsystems here ---------------------------------------------
void initializeSubsystems() {
    feeder.initialize();
    shooter.initialize();
}

// Set default command here -----------------------------------------------
void setDefaultCommands(src::Drivers *) {
    //TODO: Add something here
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
    //drivers->commandMapper.addMap(&rightSwitchMid);  TODO: Add Mappings Here
}

}  // namespace CVTestbenchControl

namespace src::Control {
// Initialize subsystems ---------------------------------------------------
void initializeSubsystemCommands(src::Drivers *drivers) {
    CVTestbenchControl::initializeSubsystems();
    CVTestbenchControl::registerSubsystems(drivers);
    CVTestbenchControl::setDefaultCommands(drivers);
    CVTestbenchControl::startupCommands(drivers);
    CVTestbenchControl::registerIOMappings(drivers);
}
}  // namespace src::Control

#endif  // TARGET_CVTESTBENCH
