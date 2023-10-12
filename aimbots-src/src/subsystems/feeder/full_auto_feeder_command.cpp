#include "full_auto_feeder_command.hpp"


namespace src::Feeder{
    
    RunFeederCommand::RunFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float FEEDER_RPM){
        this->drivers = drivers;
        this->feeder = feeder;
        this->FEEDER_RPM = FEEDER_RPM;
    }

    void RunFeederCommand::initialize(){
        feeder->setTargetRPM(FEEDER_RPM);
    }

    void RunFeederCommand::execute() {
        // feed motor with RPM speed
    }
    void RunFeederCommand::end(bool interrupted){
        feeder->setTargetRPM(0.0f);
    }
    bool RunFeederCommand::isReady() {return true;}
    bool RunFeederCommand::isFinished() const { return false;}
    

    
};
    
