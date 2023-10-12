#include "full_auto_feeder_command.hpp"


namespace src::Feeder{
    
    RunFeederCommand::RunFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float FEEDER_RPM){
        this->drivers = drivers;
        this->feeder = feeder;
        this->FEEDER_RPM = FEEDER_RPM;
    }

    void RunFeederCommand::intialize(){
        feeder->setTargetRPM(FEEDER_RPM);

    }
    void RunFeederCommand::execute() {}
    void RunFeederCommand::end(bool interrupted){}
    bool RunFeederCommand::isReady() {return true;}
    bool RunFeederCommand::isFinished() const override{return false;}
    

    
};
    
