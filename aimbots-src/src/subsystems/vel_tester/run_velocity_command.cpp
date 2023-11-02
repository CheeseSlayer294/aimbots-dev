#include "run_velocity_command.hpp"

namespace src::vel_tester {
Velocity_Control_Command::Velocity_Control_Command(
    src::Drivers* drivers, 
    Velocity_Control* vel_tester
    )
    : drivers(drivers),
      vel_tester(vel_tester)
    {
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(vel_tester));
    }
    bool isCommandRunningDisplay = false;

    void Velocity_Control_Command::initialize(){
        // vel_tester->setTargetRPM(0.0f);
    }

    void Velocity_Control_Command::execute() {
        isCommandRunningDisplay = true;
        vel_tester->updateMotorVelocityPID();
        vel_tester->setTargetRPM(500.0f);
    } 

    void Velocity_Control_Command::end(bool) {
        // vel_tester->setTargetRPM(0.0f);
        isCommandRunningDisplay = false;
    }
    
    bool Velocity_Control_Command::isReady() { return true; }

    bool Velocity_Control_Command::isFinished() const { return false; }
}