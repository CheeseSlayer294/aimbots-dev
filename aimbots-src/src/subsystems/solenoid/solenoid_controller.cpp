#include "subsystems/solenoid/solenoid_controller.hpp"


#if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

namespace src::Solenoid{
    SolenoidController::SolenoidController(tap::Drivers* drivers, Solenoid* solenoid) {
        this->drivers = drivers;
        this->solenoid = solenoid;
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(solenoid));
    };

    void SolenoidController::intialize(){

    }

    void SolenoidController::execute(){
        solenoid->updateSolenoid();
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

#endif