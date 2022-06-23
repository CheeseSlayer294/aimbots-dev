#include "subsystems/solenoid/solenoid_controller.hpp"


// #if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

namespace src::Solenoid{
    SolenoidController::SolenoidController(src::Drivers* drivers, SolenoidSubsytem* solenoid, std::string pin) : 
        drivers(drivers),
        solenoid(solenoid)
    {
        // this->drivers = drivers;
        // this->solenoid = solenoid;
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(solenoid));
        this->pin = pin;
    };

    void SolenoidController::initialize(){
        C1LastState = false;
        C2LastState = false;
        C3LastState = false;
    }

    void SolenoidController::execute(){
        if (pin == "C1") {
            C1LastState = !C1LastState;
            solenoid->solenoidWrite(C1LastState, "C1");
        } else if (pin == "C2") {
            C2LastState = !C2LastState;
            solenoid->solenoidWrite(C2LastState, "C2");
        } else if (pin == "C3") {
            C3LastState = !C3LastState;
            solenoid->solenoidWrite(C3LastState, "C3");
        }
        // solenoid->solenoidWrite(true, pin);
    }

    void SolenoidController::end(bool interrupted){

    }

    bool SolenoidController::isReady(){
        return true;
    }

    bool SolenoidController::isFinished() const {
        return false;
    }



};//namespace src::Solenoid    

// #endif