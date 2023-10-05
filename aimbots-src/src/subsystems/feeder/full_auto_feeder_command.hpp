#pragma once


#include "subsystems/feeder/feeder.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"
  
#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

class RunFeederCommand: public TapCommand{
public:
    RunFeederCommand(src::Drivers* drivers, FeederSubsystem* feeder, float FEEDER_RPM);
    
    void intialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override;
    bool isFinished() const override;
    const char* getName() const override {return "run feeder command";}
    
private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
    float FEEDER_RPM;
    
};
}; //namespace src::Feeder

#endif //ifdef FEEDER_COMPATIBLE