#include "subsystems/chassis/chassis.hpp"

#include "tap/communication/gpio/leds.hpp"

#include "utils/common_types.hpp"

#include "drivers.hpp"

using namespace tap::algorithms;

namespace src::Chassis {

ChassisSubsystem::ChassisSubsystem(
    src::Drivers* drivers,
    uint8_t DRIVEN_WHEEL_COUNT,
    uint8_t MOTORS_PER_WHEEL,
    int WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE,
    int MIN_CHASSIS_POWER,
    int MIN_WHEEL_SPEED_SINGLE_MOTOR,
    int MAX_WHEEL_SPEED_SINGLE_MOTOR,
    SmoothPIDConfig CHASSIS_VELOCITY_PID_CONFIG,
    MotorID LEFT_BACK_WHEEL_ID,
    MotorID LEFT_FRONT_WHEEL_ID,
    MotorID RIGHT_FRONT_WHEEL_ID,
    MotorID RIGHT_BACK_WHEEL_ID,
    CANBus CHASSIS_BUS,
    float STARTING_ENERGY_BUFFER,
    float ENERGY_BUFFER_LIMIT_THRESHOLD,
    float ENERGY_BUFFER_CRIT_THRESHOLD,
    float POWER_LIMIT_SAFETY_FACTOR,
    float CHASSIS_GEARBOX_RATIO,
    float WHEELBASE_WIDTH,
    float WHEELBASE_LENGTH,
    float WHEELBASE_HYPOTENUSE,
    float WHEEL_RADIUS,
    float GIMBAL_X_OFFSET,
    float GIMBAL_Y_OFFSET,
    float MIN_ROTATION_THRESHOLD)
    : ChassisSubsystemInterface(drivers),
      drivers(drivers),

      leftBackWheel(drivers, LEFT_BACK_WHEEL_ID, CHASSIS_BUS, false, "Left Back Wheel Motor"),
      leftFrontWheel(drivers, LEFT_FRONT_WHEEL_ID, CHASSIS_BUS, false, "Left Front Wheel Motor"),
      rightFrontWheel(drivers, RIGHT_FRONT_WHEEL_ID, CHASSIS_BUS, false, "Right Front Wheel Motor"),
      rightBackWheel(drivers, RIGHT_BACK_WHEEL_ID, CHASSIS_BUS, false, "Right Back Wheel Motor"),

      leftBackWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),
      leftFrontWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),
      rightFrontWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),
      rightBackWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),
#ifdef SWERVE
      leftBackYaw(drivers, LEFT_BACK_YAW_ID, CHASSIS_BUS, false, "Left Back Yaw Motor"),
      leftFrontYaw(drivers, LEFT_FRONT_YAW_ID, CHASSIS_BUS, false, "Left Front Yaw Motor"),
      rightFrontYaw(drivers, RIGHT_FRONT_YAW_ID, CHASSIS_BUS, false, "Right Front Yaw Motor"),
      rightBackYaw(drivers, RIGHT_BACK_YAW_ID, CHASSIS_BUS, false, "Right Back Yaw Motor"),
      leftBackYawPosPID(CHASSIS_YAW_PID_CONFIG),
      leftFrontYawPosPID(CHASSIS_YAW_PID_CONFIG),
      rightBackYawPosPID(CHASSIS_YAW_PID_CONFIG),
      rightFrontYawPosPID(CHASSIS_YAW_PID_CONFIG),
#endif
      targetRPMs(Matrix<float, ARBITRARY_MAX_DRIVEN_WHEEL_COUNT, ARBITRARY_MAX_MOTORS_PER_WHEEL>::zeroMatrix()),
      desiredOutputs(Matrix<float, ARBITRARY_MAX_DRIVEN_WHEEL_COUNT, ARBITRARY_MAX_MOTORS_PER_WHEEL>::zeroMatrix()),
      motors(Matrix<DJIMotor*, ARBITRARY_MAX_DRIVEN_WHEEL_COUNT, ARBITRARY_MAX_MOTORS_PER_WHEEL>::zeroMatrix()),
      velocityPIDs(Matrix<SmoothPID*, ARBITRARY_MAX_DRIVEN_WHEEL_COUNT, ARBITRARY_MAX_MOTORS_PER_WHEEL>::zeroMatrix()),
      powerLimiter(
          drivers,
          STARTING_ENERGY_BUFFER,
          ENERGY_BUFFER_LIMIT_THRESHOLD,
          ENERGY_BUFFER_CRIT_THRESHOLD,
          POWER_LIMIT_SAFETY_FACTOR),
      DRIVEN_WHEEL_COUNT(DRIVEN_WHEEL_COUNT),
      MOTORS_PER_WHEEL(MOTORS_PER_WHEEL),
      CHASSIS_VELOCITY_PID_CONFIG(CHASSIS_VELOCITY_PID_CONFIG),
      LEFT_BACK_WHEEL_ID(LEFT_BACK_WHEEL_ID),
      LEFT_FRONT_WHEEL_ID(LEFT_FRONT_WHEEL_ID),
      RIGHT_FRONT_WHEEL_ID(RIGHT_FRONT_WHEEL_ID),
      RIGHT_BACK_WHEEL_ID(RIGHT_BACK_WHEEL_ID),
      CHASSIS_BUS(CHASSIS_BUS),
      STARTING_ENERGY_BUFFER(STARTING_ENERGY_BUFFER),
      ENERGY_BUFFER_LIMIT_THRESHOLD(ENERGY_BUFFER_LIMIT_THRESHOLD),
      ENERGY_BUFFER_CRIT_THRESHOLD(ENERGY_BUFFER_CRIT_THRESHOLD),
      POWER_LIMIT_SAFETY_FACTOR(POWER_LIMIT_SAFETY_FACTOR),
      CHASSIS_GEARBOX_RATIO(CHASSIS_GEARBOX_RATIO),
      WHEELBASE_WIDTH(WHEELBASE_WIDTH),
      WHEELBASE_LENGTH(WHEELBASE_LENGTH),
      WHEELBASE_HYPOTENUSE(WHEELBASE_HYPOTENUSE),
      WHEEL_RADIUS(WHEEL_RADIUS),
      GIMBAL_X_OFFSET(GIMBAL_X_OFFSET),
      GIMBAL_Y_OFFSET(GIMBAL_Y_OFFSET),
      MIN_ROTATION_THRESHOLD(MIN_ROTATION_THRESHOLD)
//
{
    motors[LB][0] = &leftBackWheel;
    motors[LF][0] = &leftFrontWheel;
    motors[RF][0] = &rightFrontWheel;
    motors[RB][0] = &rightBackWheel;

    velocityPIDs[LB][0] = &leftBackWheelVelPID;
    velocityPIDs[LF][0] = &leftFrontWheelVelPID;
    velocityPIDs[RF][0] = &rightFrontWheelVelPID;
    velocityPIDs[RB][0] = &rightBackWheelVelPID;

    // static constexpr float WHEELBASE_HYPOTENUSE = 2 / (WHEELBASE_WIDTH + WHEELBASE_LENGTH);

    wheelVelToChassisVelMat[X][LF] = 1;
    wheelVelToChassisVelMat[X][RF] = 1;
    wheelVelToChassisVelMat[X][LB] = -1;
    wheelVelToChassisVelMat[X][RB] = -1;

    wheelVelToChassisVelMat[Y][LF] = 1;
    wheelVelToChassisVelMat[Y][RF] = -1;
    wheelVelToChassisVelMat[Y][LB] = 1;
    wheelVelToChassisVelMat[Y][RB] = -1;

    wheelVelToChassisVelMat[R][LF] = 1.0f / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][RF] = 1.0f / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][LB] = 1.0f / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][RB] = 1.0f / WHEELBASE_HYPOTENUSE;

    wheelVelToChassisVelMat *= (WHEEL_RADIUS / 4);

    ChassisSubsystem::WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE = WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE;
    ChassisSubsystem::MIN_CHASSIS_POWER = MIN_CHASSIS_POWER;
    ChassisSubsystem::MIN_WHEEL_SPEED_SINGLE_MOTOR = MIN_WHEEL_SPEED_SINGLE_MOTOR;
    ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR = MAX_WHEEL_SPEED_SINGLE_MOTOR;

#ifdef SWERVE
    // SWERVE ROBOTS
    motors[LB][1] = &leftBackYaw;
    motors[LF][1] = &leftFrontYaw;
    motors[RF][1] = &rightFrontYaw;
    motors[RB][1] = &rightBackYaw;

    velocityPIDs[LB][1] = &leftBackYawPosPID;
    velocityPIDs[LF][1] = &leftFrontYawPosPID;
    velocityPIDs[RF][1] = &rightFrontYawPosPID;
    velocityPIDs[RB][1] = &rightBackYawPosPID;
#endif
}

void ChassisSubsystem::initialize() {
    ForAllChassisMotors(&DJIMotor::initialize);

    setTargetRPMs(0, 0, 0);
    ForAllChassisMotors(&ChassisSubsystem::setDesiredOutput);
}

int refSerialWorkingDisplay = 0;
uint16_t chassisPowerLimitDisplay = 0;

float motorOutputDisplay = 0.0f;

void ChassisSubsystem::refresh() {
    ForAllChassisMotors(&ChassisSubsystem::updateMotorVelocityPID);

    ForAllChassisMotors(&ChassisSubsystem::setDesiredOutput);

    limitChassisPower();

    motorOutputDisplay = motors[RB][0]->getOutputDesired();
}

void ChassisSubsystem::limitChassisPower() {
    float powerLimitFrac = powerLimiter.getPowerLimitRatio();

    if (compareFloatClose(1.0f, powerLimitFrac, 0.001f)) {
        return;
    }

    float totalError = 0.0f;
    for (size_t i = 0; i < DRIVEN_WHEEL_COUNT; i++) {
        totalError += abs(velocityPIDs[i][0]->getError());
    }

    bool totalErrorZero = compareFloatClose(totalError, 0.0f, 0.001f);

    for (size_t i = 0; i < DRIVEN_WHEEL_COUNT; i++) {
        float velocityErrorFrac =
            totalErrorZero ? (1.0f / DRIVEN_WHEEL_COUNT) : (abs(velocityPIDs[i][0]->getError()) / totalError);

        float modifiedPowerLimitFrac = limitVal(DRIVEN_WHEEL_COUNT * powerLimitFrac * velocityErrorFrac, 0.0f, 1.0f);

        motors[i][0]->setDesiredOutput(motors[i][0]->getOutputDesired() * modifiedPowerLimitFrac);
    }
}

float targetRpmDisplay = 0.0f;
float motorRpmDisplay = 0.0f;
void ChassisSubsystem::updateMotorVelocityPID(WheelIndex WheelIdx, MotorOnWheelIndex MotorPerWheelIdx) {
    float err = 0;
    err = targetRPMs[WheelIdx][MotorPerWheelIdx] - motors[WheelIdx][MotorPerWheelIdx]->getShaftRPM();
    if (MotorPerWheelIdx == DRIVER) {
        err = targetRPMs[WheelIdx][MotorPerWheelIdx] - motors[WheelIdx][MotorPerWheelIdx]->getShaftRPM();
    } else if (MotorPerWheelIdx == YAW) {
        err = targetRPMs[WheelIdx][MotorPerWheelIdx] - motors[WheelIdx][MotorPerWheelIdx]->getEncoderWrapped();
        if (abs(err) > 4096) {
            int err_int =
                (((-1 * static_cast<int>(err)) / (abs(static_cast<int>(err)))) * (8192 - static_cast<int>(err))) % 8192;
            err = err_int * 1.0f;
        }
    }

    targetRpmDisplay = targetRPMs[LF][MotorPerWheelIdx];
    motorRpmDisplay = motors[LF][MotorPerWheelIdx]->getShaftRPM();

    velocityPIDs[WheelIdx][MotorPerWheelIdx]->runControllerDerivateError(
        err/*,
        motors[WheelIdx][MotorPerWheelIdx]->getTorque()*/);
    desiredOutputs[WheelIdx][MotorPerWheelIdx] = velocityPIDs[WheelIdx][MotorPerWheelIdx]->getOutput();
}

#if defined(SWERVE)
void ChassisSubsystem::setTargetRPMs(float x, float y, float r) {
    calculateSwerve(
        x,
        y,
        r,
        ChassisSubsystem::getMaxRefWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit));
#else
void ChassisSubsystem::setTargetRPMs(float x, float y, float r) {
    calculateHolonomic(
        x,
        y,
        r,
        ChassisSubsystem::getMaxRefWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit));
#endif
}

void ChassisSubsystem::setDesiredOutput(WheelIndex WheelIdx, MotorOnWheelIndex MotorPerWheelIdx) {
    motors[WheelIdx][MotorPerWheelIdx]->setDesiredOutput(static_cast<int32_t>(desiredOutputs[WheelIdx][MotorPerWheelIdx]));
}

#ifndef SWERVE
float xInputDisplay = 0.0f;
float yInputDisplay = 0.0f;
float rInputDisplay = 0.0f;
void ChassisSubsystem::calculateHolonomic(float x, float y, float r, float maxWheelSpeed) {
    xInputDisplay = x;
    yInputDisplay = y;
    rInputDisplay = r;
    // get distance from wheel to center of wheelbase
    float wheelbaseCenterDist = sqrtf(pow2(WHEELBASE_WIDTH / 2.0f) + pow2(WHEELBASE_LENGTH / 2.0f));

    // offset gimbal center from center of wheelbase so we rotate around the gimbal
    float leftFrontRotationRatio = modm::toRadian(wheelbaseCenterDist - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightFrontRotationRatio = modm::toRadian(wheelbaseCenterDist - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
    float leftBackRotationRatio = modm::toRadian(wheelbaseCenterDist + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightBackRotationRatio = modm::toRadian(wheelbaseCenterDist + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);

    float chassisRotateTranslated = modm::toDegree(r) / wheelbaseCenterDist;

    targetRPMs[LF][0] =
        limitVal<float>(x + y + chassisRotateTranslated * leftFrontRotationRatio, -maxWheelSpeed, maxWheelSpeed);
    targetRPMs[RF][0] =
        limitVal<float>(x - y + chassisRotateTranslated * rightFrontRotationRatio, -maxWheelSpeed, maxWheelSpeed);
    targetRPMs[LB][0] =
        limitVal<float>(-x + y + chassisRotateTranslated * leftBackRotationRatio, -maxWheelSpeed, maxWheelSpeed);
    targetRPMs[RB][0] =
        limitVal<float>(-x - y + chassisRotateTranslated * rightBackRotationRatio, -maxWheelSpeed, maxWheelSpeed);

    desiredRotation = r;
}
#endif

#ifdef SWERVE
float left_front_yaw_actual = 0.0f;
float right_front_yaw_actual = 0.0f;
float left_back_yaw_actual = 0.0f;
float right_back_yaw_actual = 0.0f;

int left_front_yaw_db;
int right_front_yaw_db;
int left_back_yaw_db;
int right_back_yaw_db;

int left_front_yaw;
int right_front_yaw;
int left_back_yaw;
int right_back_yaw;

void ChassisSubsystem::calculateSwerve(float x, float y, float r, float maxWheelSpeed) {
    // float theta = fieldRelativeInformant->getYaw();
    // float temp = y*cos(theta)+x*sin(theta);
    // x = -y*sin(theta)+x*cos(theta);
    // y = temp;

    float wheelbaseCenterDist = sqrtf(powf(WHEELBASE_WIDTH / 2.0f, 2.0f) + powf(WHEELBASE_LENGTH / 2.0f, 2.0f));

    float a = x - r * (WHEELBASE_LENGTH / wheelbaseCenterDist);
    float b = x + r * (WHEELBASE_LENGTH / wheelbaseCenterDist);
    float c = y + r * (WHEELBASE_WIDTH / wheelbaseCenterDist);
    float d = y - r * (WHEELBASE_WIDTH / wheelbaseCenterDist);

    targetRPMs[LF][0] = limitVal<float>(sqrtf(powf(b, 2.0f) + powf(d, 2.0f)), -maxWheelSpeed, maxWheelSpeed);
    left_front_yaw = (atan2f(d, b) + 3 * M_PI / 2) * (180 / M_PI) / 360 * 8191 + LEFT_FRONT_YAW_OFFSET;
    targetRPMs[LF][1] = left_front_yaw % 8191;
    // targetRPMs[LF][1] = 35/360 * 8191;
    left_front_yaw_db = targetRPMs[LF][1];
    left_front_yaw_actual = motors[LF][1]->getEncoderWrapped();

    targetRPMs[RF][0] = limitVal<float>(sqrtf(powf(b, 2.0f) + powf(c, 2.0f)), -maxWheelSpeed, maxWheelSpeed);
    right_front_yaw = (atan2f(c, b) + 3 * M_PI / 2) * (180 / M_PI) / 360 * 8191 + RIGHT_FRONT_YAW_OFFSET;
    targetRPMs[RF][1] = right_front_yaw % 8191;
    right_front_yaw_actual = motors[RF][1]->getEncoderWrapped();
    right_front_yaw_db = targetRPMs[RF][1];

    targetRPMs[LB][0] = limitVal<float>(sqrtf(powf(a, 2.0f) + powf(d, 2.0f)), -maxWheelSpeed, maxWheelSpeed);
    left_back_yaw = (atan2f(d, a) + 3 * M_PI / 2) * (180 / M_PI) / 360 * 8191 + LEFT_BACK_YAW_OFFSET;
    targetRPMs[LB][1] = left_back_yaw % 8191;
    left_back_yaw_actual = motors[LB][1]->getEncoderWrapped();
    left_back_yaw_db = targetRPMs[LB][1];

    targetRPMs[RB][0] = limitVal<float>(sqrtf(powf(a, 2.0f) + powf(c, 2.0f)), -maxWheelSpeed, maxWheelSpeed);
    int right_back_yaw = (atan2f(c, a) + 3 * M_PI / 2) * (180 / M_PI) / 360 * 8191 + RIGHT_BACK_YAW_OFFSET;
    targetRPMs[RB][1] = right_back_yaw % 8191;
    right_back_yaw_actual = motors[RB][1]->getEncoderWrapped();
    right_back_yaw_db = targetRPMs[RB][1];
    // wooo! just for commants
}
#endif

void ChassisSubsystem::calculateRail(float x, float maxWheelSpeed) {
    targetRPMs[RAIL][0] = limitVal<float>(x, -maxWheelSpeed, maxWheelSpeed);
}

float ChassisSubsystem::calculateRotationLimitedTranslationalWheelspeed(
    float chassisRotationDesiredWheelspeed,
    float maxWheelSpeed) {
    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain = 1.0f;

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing power
    // consumption when the wheel rotation speed for chassis rotation is greater than the MIN_ROTATION_THRESHOLD
    if (fabsf(chassisRotationDesiredWheelspeed) > MIN_ROTATION_THRESHOLD) {
        rTranslationalGain =
            pow2(maxWheelSpeed + MIN_ROTATION_THRESHOLD - fabsf(chassisRotationDesiredWheelspeed) / maxWheelSpeed);

        rTranslationalGain = tap::algorithms::limitVal<float>(rTranslationalGain, 0.0f, 1.0f);
    }
    return rTranslationalGain * maxWheelSpeed;
}

bool ChassisSubsystem::getTokyoDrift() const { return tokyoDrift; }
};  // namespace src::Chassis
