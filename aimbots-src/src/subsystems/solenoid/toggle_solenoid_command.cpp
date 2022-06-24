#include "subsystems/solenoid/toggle_solenoid_command.hpp"


// #if defined(TARGET_SWERVE_ENGINEER) && defined(TARGET_ENGINEER)

namespace src::Solenoid{
    ToggleSolenoidCommand::ToggleSolenoidCommand(src::Drivers* drivers, SolenoidSubsytem* solenoid, std::string solenoidName) : 
        drivers(drivers),
        solenoid(solenoid)
    {
        // this->drivers = drivers;
        // this->solenoid = solenoid;
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(solenoid));
        this->solenoidName = solenoidName;
    };

    void ToggleSolenoidCommand::initialize(){
        uint8_t state = solenoid->getSolenoidState(solenoidName);
        if(state == UNKNOWN){
            solenoid->setSolenoidState(CLOSED, solenoidName);
            solenoid->setSolenoidPostion(SOLENOID_CLOSED_ANGLE, solenoidName);
        }else {
            solenoid->setSolenoidPostion(state ? SOLENOID_CLOSED_ANGLE : SOLENOID_OPEN_ANGLE, solenoidName);
            solenoid->setSolenoidState(state, solenoidName);
        }

    }

    void ToggleSolenoidCommand::execute(){
        
    }

    void ToggleSolenoidCommand::end(bool interrupted){

    }

    bool ToggleSolenoidCommand::isReady(){
        return true;
    }

    bool ToggleSolenoidCommand::isFinished() const {
        return solenoid->isSolenoidReady();
    }



};//namespace src::Solenoid    

// #endif