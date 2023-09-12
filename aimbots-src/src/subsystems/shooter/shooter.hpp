#pragma once
#include <vector>

#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"
// delete this when done refactoring pls !!!
// #include "utils/robot_specific_inc.hpp"

namespace src::Shooter {

enum MotorIndex {
    RIGHT = 0,
    LEFT = 1,
#ifdef TARGET_SENTRY
    TOP_RIGHT = 0,
    BOT_RIGHT = 1,
    TOP_LEFT = 2,
    BOT_LEFT = 3,
#endif
};

class ShooterSubsystem : public tap::control::Subsystem {
public:
    ShooterSubsystem(
        tap::Drivers* drivers,
        src::Utils::RefereeHelperTurreted* refHelper,
        uint8_t SHOOTER_MOTOR_COUNT,
        SmoothPIDConfig SHOOTER_VELOCITY_PID_CONFIG,
        CANBus SHOOTER_BUS,
        MotorID SHOOTER_ID_ARRAY[],
        bool SHOOTER_DIRECTION_ARRAY[]
        // ,MotorID shooter_1_id,
        // MotorID shooter_2_id
    );

    mockable void initialize() override;
    void refresh() override;

    float getHighestMotorSpeed() const;

    float getMotorSpeed(MotorIndex motorIdx) const;

    /**
     * @brief Updates velocity PID and motor RPM for a single motor. Can be used with ForAllShooterMotors().
     * Should be called continuously in subsystem refresh.
     *
     * @param motorIdx index for DJIMotor matrix
     */
    void updateMotorVelocityPID(MotorIndex motorIdx);

    /**
     * @brief Changes the target RPM for a single motor. Intended for use with ForAllShooterMotors(),
     * and should be called from a command to declare intended RPM. Does not necessarily need to be called continuously
     *
     * @param motorIdx index for DJIMotor matrix
     * @param targetRPM intended target RPM
     */
    void setTargetRPM(MotorIndex motorIdx, float targetRPM);

    /**
     * @brief Updates the desiredOutputs matrix with the desired output of a single motor.
     * Intended to be called from commands.
     *
     * @param motorIdx
     * @param desiredOutput
     */
    void setDesiredOutput(MotorIndex motorIdx, float desiredOutput);

    /**
     * @brief Sets the desired output of a single motor from the desiredOutputs matrix
     * Should only be called once per loop for consistency in Shooter refresh.
     *
     * @param motorIdx
     */
    void setDesiredOutputToMotor(MotorIndex motorIdx);

    bool isOnline() const {
        for (auto i = 0; i < SHOOTER_MOTOR_COUNT; i++) {
            if (!motors[i][0]->isMotorOnline()) {
                return false;
            }
        }
        return true;
    }

#ifndef ENV_UNIT_TESTS
private:
#else
private:
    ;

public:  // why is this public:
#endif
    // just set this bigger than how many shooter motors team is using
    static constexpr int ARRAY_SIZE = 10;
    // DJIMotor flywheel1, flywheel2;
    // DJIMotor flywheel_array[ARRAY_SIZE];
    // SmoothPID flywheel1PID, flywheel2PID;
    // SmoothPID flywheel_PID_array[ARRAY_SIZE];

#ifdef TARGET_SENTRY
    DJIMotor flywheel3, flywheel4;
    SmoothPID flywheel3PID, flywheel4PID;
#endif

    // CONSTANTS
    uint8_t SHOOTER_MOTOR_COUNT;

    // we should never be using more than 10 shooter motors
    Matrix<float, ARRAY_SIZE, 1> targetRPMs;
    Matrix<int32_t, ARRAY_SIZE, 1> desiredOutputs;
    Matrix<DJIMotor*, ARRAY_SIZE, 1> motors;
    Matrix<SmoothPID*, ARRAY_SIZE, 1> velocityPIDs;

    // maybe I'll just stick to matrix, then...
    //  std::array<float, 8> targetRPMs;
    //  std::array<int32_t, 8> desiredOutputs;
    //  std::array<DJIMotor*, 8> motors;
    //  std::array<SmoothPID*, 8> velocityPIDs;

    src::Utils::RefereeHelperTurreted* refHelper;
};
};  // namespace src::Shooter