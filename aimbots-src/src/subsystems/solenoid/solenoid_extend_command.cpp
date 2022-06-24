#include "subsystems/solenoid/solenoid_extend_command.hpp"

namespace src::Solenoid{

ExtendSolenoidCommand::ExtendSolenoidCommand(src::Drivers* drivers, SolenoidSubsytem* solenoid, std::string solenoidName):
    drivers(drivers),
    solenoid(solenoid),
    solenoidName(solenoidName) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(solenoid));
}

void ExtendSolenoidCommand::initialize(){
    solenoid->setSolenoidPostion(SOLENOID_OPEN_ANGLE, solenoidName);
}

void ExtendSolenoidCommand::execute(){

}

void ExtendSolenoidCommand::end(bool){

}

bool ExtendSolenoidCommand::isReady(){
    return true;
}

bool ExtendSolenoidCommand::isFinished() const{
    return solenoid -> isSolenoidReady();
}

}; //namespace src::Solenoid