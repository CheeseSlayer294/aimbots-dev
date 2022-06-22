#include "subsystems/solenoid/solenoid_controller.hpp"


// #if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

namespace src::Solenoid{
    SolenoidController::SolenoidController(src::Drivers* drivers, SolenoidSubsytem* solenoid) {
        this->drivers = drivers;
        this->solenoid = solenoid;
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(solenoid));
    };

    void SolenoidController::initialize(){
        solenoid->initialize();
    }

    void SolenoidController::execute(){
        solenoid->solenoidWrite(true, " ");
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