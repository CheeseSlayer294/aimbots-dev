#include "subsystems/shooter/shooter.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

#include "utils/common_types.hpp"

namespace src::Shooter {

ShooterSubsystem::ShooterSubsystem(
    tap::Drivers* drivers,
    src::Utils::RefereeHelperTurreted* refHelper,
    uint8_t SHOOTER_MOTOR_COUNT,
    SmoothPIDConfig SHOOTER_VELOCITY_PID_CONFIG,
    CANBus SHOOTER_BUS,
    const MotorID* SHOOTER_ID_ARRAY,
    const bool* SHOOTER_DIRECTION_ARRAY)
    : Subsystem(drivers),
      SHOOTER_MOTOR_COUNT(SHOOTER_MOTOR_COUNT),
      targetRPMs(Matrix<float, ARRAY_SIZE, 1>::zeroMatrix()),
      desiredOutputs(Matrix<int32_t, ARRAY_SIZE, 1>::zeroMatrix()),
      motors(Matrix<DJIMotor*, ARRAY_SIZE, 1>::zeroMatrix()),
      velocityPIDs(Matrix<SmoothPID*, ARRAY_SIZE, 1>::zeroMatrix()),
      refHelper(refHelper) /*,
       flywheel1(drivers, SHOOTER_ID_ARRAY[0], SHOOTER_BUS, SHOOTER_DIRECTION_ARRAY[0], "Flywheel One"),
       flywheel2(drivers, SHOOTER_ID_ARRAY[1], SHOOTER_BUS, SHOOTER_DIRECTION_ARRAY[1], "Flywheel Two"),
       flywheel1PID(SHOOTER_VELOCITY_PID_CONFIG),
       flywheel2PID(SHOOTER_VELOCITY_PID_CONFIG)
 #ifdef TARGET_SENTRY
           flywheel3(drivers, SHOOTER_ID_ARRAY[2], SHOOTER_BUS, SHOOTER_DIRECTION_ARRAY[2], "Flywheel Three"),
       flywheel4(drivers, SHOOTER_ID_ARRAY[3], SHOOTER_BUS, SHOOTER_DIRECTION_ARRAY[3], "Flywheel Four"),
       flywheel3PID(SHOOTER_VELOCITY_PID_CONFIG),
       flywheel4PID(SHOOTER_VELOCITY_PID_CONFIG),
 #endif*/
{
    for (int i = 0; i < SHOOTER_MOTOR_COUNT; i++) {
        motors[i][0] = &DJIMotor(drivers, SHOOTER_ID_ARRAY[i], SHOOTER_BUS, SHOOTER_DIRECTION_ARRAY[i], "flywheel");
        velocityPIDs[i][0] = &SmoothPID(SHOOTER_VELOCITY_PID_CONFIG);
    }
    // motors[RIGHT][0] = &flywheel1;  // TOP_RIGHT == RIGHT
    // motors[LEFT][0] = &flywheel2;   // BOT_RIGHT == LEFT
    // velocityPIDs[RIGHT][0] = &flywheel1PID;
    // velocityPIDs[LEFT][0] = &flywheel2PID;
    // #ifdef TARGET_SENTRY
    //     motors[TOP_LEFT][0] = &flywheel3;
    //     motors[BOT_LEFT][0] = &flywheel4;
    //     velocityPIDs[TOP_LEFT][0] = &flywheel3PID;
    //     velocityPIDs[BOT_LEFT][0] = &flywheel4PID;
    // #endif
}

void ShooterSubsystem::initialize() {
    ForAllShooterMotors(&DJIMotor::initialize);

    ForAllShooterMotors(&DJIMotor::setDesiredOutput, static_cast<int32_t>(0.0f));
}

float PIDoutDisplay = 0.0f;
float shaftSpeedDisplay = 0.0f;

float FWRight1 = 0.0f;  // RIGHT / TOP_RIGHT
float FWLeft1 = 0.0f;   // LEFT / BOT_LEFT
#ifdef TARGET_SENTRY
float FWTopLeft = 0.0f;  // TOP_LEFT
float FWBotLeft = 0.0f;  // BOT_LEFT
#endif

// Update the actual RPMs of the motors; the calculation is called from ShooterCommand
void ShooterSubsystem::refresh() {
    // Debug info
    if (motors[0][0]->isMotorOnline()) {
        shaftSpeedDisplay = motors[0][0]->getShaftRPM();
        PIDoutDisplay = velocityPIDs[0][0]->getOutput();

        FWRight1 = motors[0][0]->getShaftRPM();
    }
    if (motors[1][0]->isMotorOnline()) {
        FWLeft1 = motors[1][0]->getShaftRPM();
    }
#ifdef TARGET_SENTRY
    if (motors[2][0]->isMotorOnline()) {
        FWTopLeft = motors[2][0]->getShaftRPM();
    }
    if (motors[3][0]->isMotorOnline()) {
        FWBotLeft = motors[3][0]->getShaftRPM();
    }
#endif
    ForAllShooterMotors(&ShooterSubsystem::setDesiredOutputToMotor);

    refHelper->updatePredictedProjectileSpeed();
}

// Returns the speed of the shooter motor with the highest absolute value of RPM
float ShooterSubsystem::getHighestMotorSpeed() const {
    float highestMotorSpeed = 0.0f;
    for (int i = 0; i < SHOOTER_MOTOR_COUNT; i++) {
        if (motors[i][0]->isMotorOnline()) {
            float motorSpeed = motors[i][0]->getShaftRPM();
            if (fabs(motorSpeed) > highestMotorSpeed) {
                highestMotorSpeed = motorSpeed;
            }
        }
    }
    return highestMotorSpeed;
}

float ShooterSubsystem::getMotorSpeed(MotorIndex motorIdx) const {
    if (motors[motorIdx][0]->isMotorOnline()) {
        return motors[motorIdx][0]->getShaftRPM();
    }
    return 0.0f;
}

void ShooterSubsystem::updateMotorVelocityPID(MotorIndex motorIdx) {
    if (motors[motorIdx][0]->isMotorOnline()) {  // Check if motor is online when getting info from it
        float err = targetRPMs[motorIdx][0] - motors[motorIdx][0]->getShaftRPM();
        float PIDOut = velocityPIDs[motorIdx][0]->runController(err, motors[motorIdx][0]->getTorque());
        setDesiredOutput(motorIdx, PIDOut);
    }
}

void ShooterSubsystem::setTargetRPM(MotorIndex motorIdx, float targetRPM) { targetRPMs[motorIdx][0] = targetRPM; }

float powerDisplay = 0.0f;

void ShooterSubsystem::setDesiredOutput(MotorIndex motorIdx, float desiredOutput) {
    desiredOutputs[motorIdx][0] = static_cast<int32_t>(desiredOutput);
}

void ShooterSubsystem::setDesiredOutputToMotor(MotorIndex motorIdx) {
    if (motors[motorIdx][0]->isMotorOnline()) {  // Check if motor is online when setting to it
        motors[motorIdx][0]->setDesiredOutput(desiredOutputs[motorIdx][0]);
    }
}

/**
 * Allows user to call a DJIMotor member function on all shooter motors
 *
 * @param function pointer to a member function of DJIMotor
 * @param args arguments to pass to the member function
 */
template <class... Args>
void ForAllShooterMotors(void (DJIMotor::*func)(Args...), Args... args) {
    for (auto i = 0; i < SHOOTER_MOTOR_COUNT; i++) {
        (motors[i][0]->*func)(args...);
    }
}

/**
 * Allows user to call a ShooterSubsystem function on all shooter motors.
 *
 * @param function pointer to a member function of ShooterSubsystem that takes a MotorIndex as it's first argument
 * @param args arguments to pass to the member function
 */
template <class... Args>
void ForAllShooterMotors(void (ShooterSubsystem::*func)(MotorIndex, Args...), Args... args) {
    for (auto i = 0; i < SHOOTER_MOTOR_COUNT; i++) {
        MotorIndex mi = static_cast<MotorIndex>(i);
        (this->*func)(mi, args...);
    }
}

};  // namespace src::Shooter