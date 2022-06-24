#pragma once


#include "drivers.hpp"
#include "subsystems/solenoid/solenoid_subsystem.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Solenoid {

class ExtendSolenoidCommand : public TapCommand {

    public:
    ExtendSolenoidCommand(src::Drivers* drivers, SolenoidSubsytem* solenoid, std::string solenoidName);

    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;

    bool isReady() override;
    bool isFinished() const override;

    private:
    src::Drivers* drivers;
    SolenoidSubsytem* solenoid;
    std::string solenoidName;
    const char* getName() const override { return "extend solenoid command"; }



};


}//namespace src::Solenoid