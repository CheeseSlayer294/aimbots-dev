#include "subsystems/solenoid/solenoid_closed_command.hpp"

namespace src::Solenoid{

ClosedSolenoidCommand::ClosedSolenoidCommand(src::Drivers* drivers, SolenoidSubsytem* solenoid, std::string solenoidName) {
    this->drivers = drivers;
    this->solenoid = solenoid;
    this->solenoidName = solenoidName;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(solenoid));
}

void ClosedSolenoidCommand::initialize(){
    solenoid->setSolenoidPostion(SOLENOID_CLOSED_ANGLE, solenoidName);
}

void ClosedSolenoidCommand::execute(){

}

void ClosedSolenoidCommand::end(bool){

}

bool ClosedSolenoidCommand::isReady(){
    return true;
}

bool ClosedSolenoidCommand::isFinished() const{
    return solenoid -> isSolenoidReady();
}

}; //namespace src::Solenoid