#include "full_auto_feeder_command.hpp"


namespace src::Feeder{
    
    RunFeederCommand::RunFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float FEEDER_RPM){
        this->drivers = drivers;
        this->feeder = feeder;
        this->FEEDER_RPM = FEEDER_RPM;
        addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
    }

    void RunFeederCommand::initialize() {}

    bool isRunning_display = false;
    float currentRPM_display = 0.0f;

    void RunFeederCommand::execute() {
        feeder->setTargetRPM(FEEDER_RPM);
        isRunning_display = true;
        currentRPM_display = feeder->getTargetRPM();
    }

    void RunFeederCommand::end(bool interrupted) {}

    bool RunFeederCommand::isReady() {return true;}
    
    bool RunFeederCommand::isFinished() const { return false;}
};
    
