#pragma once

#include <drivers.hpp>
#include <tap/algorithms/contiguous_float.hpp>
#include <tap/control/subsystem.hpp>
#include <utils/common_types.hpp>
#include <utils/robot_specific_inc.hpp>

static inline float DJIEncoderValueToRadians(int64_t encoderValue) {
    return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
}

namespace src::LinearStage {

enum LinearStageAxis { X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2 };

class LinearStageSubsystem : public tap::control::Subsystem {
public:
    LinearStageSubsystem(src::Drivers*);
    ~LinearStageSubsystem() = default;

    void BuildXAxisMotors() {
        for (auto i = 0; i < X_AXIS_STAGE_MOTOR_COUNT; i++) {
            xAxisMotors[i] = new DJIMotor(
                drivers,
                X_AXIS_MOTOR_IDS[i],
                LINEAR_STAGE_BUS,
                X_AXIS_MOTOR_DIRECTIONS[i],
                X_AXIS_MOTOR_NAMES[i]);
            // currentXAxisAnglesByMotor[i] = new tap::algorithms::ContiguousFloat(0.0f, -M_PI, M_PI);
        }
    }

    void BuildYAxisMotors() {
        for (auto i = 0; i < Y_AXIS_STAGE_MOTOR_COUNT; i++) {
            yAxisMotors[i] = new DJIMotor(
                drivers,
                Y_AXIS_MOTOR_IDS[i],
                LINEAR_STAGE_BUS,
                Y_AXIS_MOTOR_DIRECTIONS[i],
                Y_AXIS_MOTOR_NAMES[i]);
            // currentXAxisAnglesByMotor[i] = new tap::algorithms::ContiguousFloat(0.0f, -M_PI, M_PI);
        }
    }

    void BuildZAxisMotors() {
        for (auto i = 0; i < Z_AXIS_STAGE_MOTOR_COUNT; i++) {
            zAxisMotors[i] = new DJIMotor(
                drivers,
                Z_AXIS_MOTOR_IDS[i],
                LINEAR_STAGE_BUS,
                Z_AXIS_MOTOR_DIRECTIONS[i],
                Z_AXIS_MOTOR_NAMES[i]);
            // currentXAxisAnglesByMotor[i] = new tap::algorithms::ContiguousFloat(0.0f, -M_PI, M_PI);
        }
    }

    /*
    void BuildPitchMotors() {
        for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
            pitchMotors[i] = new DJIMotor(
                drivers,
                X_AXIS_MOTOR_IDS[i],
                PITCH_GIMBAL_BUS,
                PITCH_MOTOR_DIRECTIONS[i],
                PITCH_MOTOR_NAMES[i]);
            currentPitchAxisAnglesByMotor[i] = new tap::algorithms::ContiguousFloat(0.0f, -M_PI, M_PI);
        }
    }*/

    /**
     * Allows user to call a DJIMotor member function on all gimbal motors
     *
     * @param function pointer to a member function of DJIMotor
     * @param args arguments to pass to the member function
     */
    template <class... Args>
    void ForAllXAxisMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto& xAxisMotor : xAxisMotors) {
            (xAxisMotor->*func)(args...);
        }
    }
    template <class... Args>
    void ForAllYAxisMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto& yAxisMotor : yAxisMotors) {
            (yAxisMotor->*func)(args...);
        }
    }
    template <class... Args>
    void ForAllZAxisMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto& zAxisMotor : zAxisMotors) {
            (zAxisMotor->*func)(args...);
        }
    }

    /*
    template <class... Args>
    void ForAllPitchMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto& pitchMotor : pitchMotors) {
            (pitchMotor->*func)(args...);
        }
    }
    */

    // PARDON OUR MESS
    // WARCRIMES IN PROGRESS BELOW ... AND ABOVE

    /**
     * Allows user to call a GimbalSubsystem function on all gimbal motors.
     *
     * @param function pointer to a member function of GimbalSubsystem that takes a WheelIndex
     * as its first argument and the motor-per-wheel index as the second argument
     * @param args arguments to pass to the member function
     */
    template <class... Args>
    void ForAllYawMotors(void (GimbalSubsystem::*func)(uint8_t YawIdx, Args...), Args... args) {
        for (uint8_t i = 0; i < YAW_MOTOR_COUNT; i++) {
            (this->*func)(i, args...);
        }
    }
    template <class... Args>
    void ForAllPitchMotors(void (GimbalSubsystem::*func)(uint8_t PitchIdx, Args...), Args... args) {
        for (uint8_t i = 0; i < PITCH_MOTOR_COUNT; i++) {
            (this->*func)(i, args...);
        }
    }

    mockable void initialize() override;
    void refresh() override;

    const char* getName() override { return "Gimbal Subsystem"; }

    inline bool isOnline() const {
        for (auto& yawMotor : yawMotors) {
            if (!yawMotor->isMotorOnline()) {
                return false;
            }
        }
        for (auto& pitchMotor : pitchMotors) {
            if (!pitchMotor->isMotorOnline()) {
                return false;
            }
        }
        return true;
    }

    void setDesiredYawMotorOutput(uint8_t YawIdx, float output) { desiredYawMotorOutputs[YawIdx] = output; }
    void setDesiredPitchMotorOutput(uint8_t PitchIdx, float output) { desiredPitchMotorOutputs[PitchIdx] = output; }

    void setAllDesiredYawMotorOutputs(uint16_t output) { desiredYawMotorOutputs.fill(output); }
    void setAllDesiredPitchOutputs(uint16_t output) { desiredPitchMotorOutputs.fill(output); }

    inline int16_t getYawMotorRPM(uint8_t YawIdx) const {
        return (yawMotors[YawIdx]->isMotorOnline()) ? yawMotors[YawIdx]->getShaftRPM() : 0;
    }

    inline int16_t getPitchMotorRPM(uint8_t PitchIdx) const {
        return (pitchMotors[PitchIdx]->isMotorOnline()) ? pitchMotors[PitchIdx]->getShaftRPM() : 0;
    }

    inline int16_t getYawMotorTorque(uint8_t yawIdx) const {
        return yawMotors[yawIdx]->isMotorOnline() ? yawMotors[yawIdx]->getTorque() : 0;
    }

    inline int16_t getPitchMotorTorque(uint8_t pitchIdx) const {
        return pitchMotors[pitchIdx]->isMotorOnline() ? pitchMotors[pitchIdx]->getTorque() : 0;
    }

    inline float getYawAxisRPM() const {
        int16_t rpm = 0;
        uint8_t onlineMotors = 0;
        for (auto& yawMotor : yawMotors) {
            if (yawMotor->isMotorOnline()) {
                rpm += yawMotor->getShaftRPM();
                onlineMotors++;
            }
        }
        return (onlineMotors > 0) ? rpm / onlineMotors : 0.0f;
    }
    inline float getPitchAxisRPM() const {
        int16_t rpm = 0;
        uint8_t onlineMotors = 0;
        for (auto& pitchMotor : pitchMotors) {
            if (pitchMotor->isMotorOnline()) {
                rpm += pitchMotor->getShaftRPM();
                onlineMotors++;
            }
        }
        return (onlineMotors > 0) ? rpm / onlineMotors : 0.0f;
    }

    inline float getCurrentYawAxisAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? currentYawAxisAngle.getValue()
                                            : modm::toDegree(currentYawAxisAngle.getValue());
    }

    inline float getCurrentPitchAxisAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? currentPitchAxisAngle.getValue()
                                            : modm::toDegree(currentPitchAxisAngle.getValue());
    }

    inline tap::algorithms::ContiguousFloat const& getCurrentYawAxisAngleAsContiguousFloat() const {
        return currentYawAxisAngle;
    }
    inline tap::algorithms::ContiguousFloat const& getCurrentPitchAxisAngleAsContiguousFloat() const {
        return currentPitchAxisAngle;
    }

    inline float getYawMotorAngleUnwrapped(uint8_t YawIdx) const {
        return (yawMotors[YawIdx]->isMotorOnline()) ? DJIEncoderValueToRadians(yawMotors[YawIdx]->getEncoderUnwrapped())
                                                    : 0.0f;
    }

    inline float getYawMotorAngleWrapped(uint8_t YawIdx) const {
        return (yawMotors[YawIdx]->isMotorOnline()) ? DJIEncoderValueToRadians(yawMotors[YawIdx]->getEncoderWrapped())
                                                    : 0.0f;
    }
    inline float getPitchMotorAngleWrapped(uint8_t PitchIdx) const {
        return (pitchMotors[PitchIdx]->isMotorOnline())
                   ? DJIEncoderValueToRadians(pitchMotors[PitchIdx]->getEncoderWrapped())
                   : 0.0f;
    }

    inline float getTargetYawAxisAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? targetYawAxisAngle.getValue() : modm::toDegree(targetYawAxisAngle.getValue());
    }
    inline void setTargetYawAxisAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Radians) ? angle : modm::toRadian(angle);
        targetYawAxisAngle.setValue(angle);
    }

    inline float getTargetPitchAxisAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? targetPitchAxisAngle.getValue()
                                            : modm::toDegree(targetPitchAxisAngle.getValue());
    }
    inline void setTargetPitchAxisAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Radians) ? angle : modm::toRadian(angle);
        targetPitchAxisAngle.setValue(tap::algorithms::limitVal(angle, PITCH_AXIS_SOFTSTOP_LOW, PITCH_AXIS_SOFTSTOP_HIGH));
    }

    float getYawSetpointError(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? targetYawAxisAngle.difference(currentYawAxisAngle)
                                            : modm::toDegree(targetYawAxisAngle.difference(currentYawAxisAngle));
    }
    float getPitchSetpointError(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? targetPitchAxisAngle.difference(currentPitchAxisAngle)
                                            : modm::toDegree(targetPitchAxisAngle.difference(currentPitchAxisAngle));
    }

    float getYawMotorSetpointError(uint8_t YawIdx, AngleUnit unit) const;
    float getPitchMotorSetpointError(uint8_t PitchIdx, AngleUnit unit) const;

    // from the buffer. gimbal orientation from the buffer.
    inline std::pair<float, float>& getGimbalOrientation(int index) { return gimbalOrientationBuffer[index]; }

    // put in your time, we get the closest orientation entry at that time.
    inline std::pair<float, float>& getGimbalOrientationAtTime(uint32_t time_ms) {
        // assume 2 ms delay between gimbal updates
        int index = std::min(time_ms / 2, GIMBAL_BUFFER_SIZE - 1);
        return gimbalOrientationBuffer[index];
    }
    // just in case?
    inline void clearGimbalOrientationBuffer() { gimbalOrientationBuffer.clear(); }

private:
    src::Drivers* drivers;

    static const uint8_t X_AXIS_STAGE_MOTOR_COUNT;
    static const uint8_t Y_AXIS_STAGE_MOTOR_COUNT;
    static const uint8_t Z_AXIS_STAGE_MOTOR_COUNT;

    std::array<DJIMotor*, X_AXIS_STAGE_MOTOR_COUNT> xAxisMotors;
    std::array<DJIMotor*, Y_AXIS_STAGE_MOTOR_COUNT> yAxisMotors;
    std::array<DJIMotor*, Z_AXIS_STAGE_MOTOR_COUNT> zAxisMotors;

    std::array<float, X_AXIS_STAGE_MOTOR_COUNT> currentXAxisAnglesByMotor;  // chassis relative, in radians
    std::array<float, Y_AXIS_STAGE_MOTOR_COUNT> currentYAxisAnglesByMotor;  // chassis relative, in radians
    std::array<float, Z_AXIS_STAGE_MOTOR_COUNT> currentZAxisAnglesByMotor;  // chassis relative, in radians

    static const float X_AXIS_TICKS_PER_METER;
    static const float Y_AXIS_TICKS_PER_METER;
    static const float Z_AXIS_TICKS_PER_METER;

    std::array<float, X_AXIS_STAGE_MOTOR_COUNT> desiredXAxisMotorOutputs;
    std::array<float, Y_AXIS_STAGE_MOTOR_COUNT> desiredYAxisMotorOutputs;
    std::array<float, Z_AXIS_STAGE_MOTOR_COUNT> desiredZAxisMotorOutputs;

    float currentXAxisAngle;  // average of currentYawAxisAnglesByMotor
    float currentYAxisAngle;  // chassis relative, in radians
    float currentZAxisAngle;  // average of currentYawAxisAnglesByMotor

    float targetXAxisAngle;  // chassis relative, in radians
    float targetYAxisAngle;  // chassis relative, in radians
    float targetZAxisAngle;  // chassis relative, in radians

    static constexpr CANBus LINEAR_STAGE_BUS;

    void setDesiredOutputToXAxisMotor(uint8_t XIdx);
    void setDesiredOutputToYAxisMotor(uint8_t YIdx);
    void setDesiredOutputToZAxisMotor(uint8_t ZIdx);

    static const std::array<MotorID, X_AXIS_STAGE_MOTOR_COUNT> X_AXIS_MOTOR_IDS;
    static const std::array<MotorID, Y_AXIS_STAGE_MOTOR_COUNT> Y_AXIS_MOTOR_IDS;
    static const std::array<MotorID, Z_AXIS_STAGE_MOTOR_COUNT> Z_AXIS_MOTOR_IDS;

    static const std::array<bool, X_AXIS_STAGE_MOTOR_COUNT> X_AXIS_MOTOR_DIRECTIONS;
    static const std::array<bool, Y_AXIS_STAGE_MOTOR_COUNT> Y_AXIS_MOTOR_DIRECTIONS;
    static const std::array<bool, Z_AXIS_STAGE_MOTOR_COUNT> Z_AXIS_MOTOR_DIRECTIONS;

    static const std::array<const char*, X_AXIS_STAGE_MOTOR_COUNT> X_AXIS_MOTOR_NAMES;
    static const std::array<const char*, Y_AXIS_STAGE_MOTOR_COUNT> Y_AXIS_MOTOR_NAMES;
    static const std::array<const char*, Z_AXIS_STAGE_MOTOR_COUNT> Z_AXIS_MOTOR_NAMES;

    /*
static const uint32_t GIMBAL_BUFFER_SIZE = 40;

// gimbal yaw / pitch buffer
// pitch, yaw is first, second, respectively in the pair
Deque<std::pair<float, float>, GIMBAL_BUFFER_SIZE> gimbalOrientationBuffer;
*/
};

}  // namespace src::LinearStage