#include "utils/robot_specific_inc.hpp"
#include "subsystems/shooter/shooter.hpp"
#include "subsystems/shooter/run_shooter_command.hpp"
#ifdef SHOOTER_COMPATIBLE

namespace src::Shooter{

RunShooterCommand::RunShooterCommand(tap::Drivers* drivers, ShooterSubsystem* shooter, int shooterRPM)
    : drivers(drivers),
      shooter(shooter),
      shooterRPM(shooterRPM)
{
    addSubsystemRequirment(dynamic_cast<tap::control::Subsystem*->(shooter));
}

    
    void RunShooterCommand::execute(){
        MotorIndex mi = static_cast<MotorIndex>(1);
        MotorIndex mi2 = static_cast<MotorIndex>(2);
        shooter->setTargetRPM(mi, 3900);
        shooter->setTargetRPM(mi2, -3900);
    }

    void RunShooterCommand::initialize(){}

    void RunShooterCommand::end(bool interupted){}

    bool RunShooterCommand::isReady(){return true;}

    bool RunShooterCommand::isFinished(){return false;}



}
#endif