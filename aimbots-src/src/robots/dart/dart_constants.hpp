#pragma once
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

#define NO_CHASSIS

static constexpr uint8_t DRIVEN_WHEEL_COUNT = 0;
static constexpr uint8_t MOTORS_PER_WHEEL = 0;

static constexpr uint8_t SHOOTER_MOTOR_COUNT = 6;

static constexpr float DEV_BOARD_YAW_OFFSET = 180.0f;  // in radians

static constexpr SmoothPIDConfig FEEDER_VELOCITY_PID_CONFIG = {
    .kp = 15.0f,
    .ki = 0.0f,
    .kd = 0.8f,
    .maxICumulative = 10.0f,
    .maxOutput = M2006_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

/**
 * @brief Position PID constants
 */
static constexpr SmoothPIDConfig YAW_POSITION_PID_CONFIG = {
    .kp = 600.0f,
    .ki = 0.0f,
    .kd = 500.0f,
    .maxICumulative = 10.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr SmoothPIDConfig PITCH_POSITION_PID_CONFIG = {
    .kp = 1850.0f,
    .ki = 0.0f,
    .kd = 150.0f,
    .maxICumulative = 10.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr float kGRAVITY = 0.0f;
static constexpr float HORIZON_OFFSET = -0.0f;

static constexpr SmoothPIDConfig SHOOTER_VELOCITY_PID_CONFIG = {
    .kp = 40.0f,
    .ki = 0.10f,
    .kd = 0.00f,
    .maxICumulative = 10.0f,
    .maxOutput = 30000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

static constexpr uint16_t shooter_speed_array[6] = {
    15,
    3900,  // {ball m/s, flywheel rpm}
    18,
    4500,
    30,
    9000};

static const Matrix<uint16_t, 3, 2> SHOOTER_SPEED_MATRIX(shooter_speed_array);

// static constexpr float FEEDER_DEFAULT_RPM = 3000.0f;  // FEEDER RPM: 3000.0f
// static constexpr int DEFAULT_BURST_LENGTH = 5;        // balls

// CAN Bus 1
static constexpr CANBus SHOOTER_BUS = CANBus::CAN_BUS1;
static constexpr CANBus FEED_BUS = CANBus::CAN_BUS1;

static constexpr MotorID SHOOTER_1_ID = MotorID::MOTOR1;  // middle left
static constexpr MotorID SHOOTER_2_ID = MotorID::MOTOR2;  // middle right
static constexpr MotorID SHOOTER_3_ID = MotorID::MOTOR3;  // bottom right
static constexpr MotorID SHOOTER_4_ID = MotorID::MOTOR4;  // bottom left
static constexpr MotorID SHOOTER_5_ID = MotorID::MOTOR5;  // top left
static constexpr MotorID SHOOTER_6_ID = MotorID::MOTOR6;  // top right

static constexpr MotorID FEEDER_ID = MotorID::MOTOR7;

static constexpr bool SHOOTER_1_DIRECTION = false;
static constexpr bool SHOOTER_2_DIRECTION = true;

static constexpr bool FEEDER_DIRECTION = false;

static constexpr float FEEDER_DEFAULT_RPM = 3000.0f;  // FEEDER RPM: 3000.0f

// CAN Bus 2
static constexpr CANBus GIMBAL_BUS = CANBus::CAN_BUS2;

static constexpr MotorID YAW_MOTOR_ID = MotorID::MOTOR5;
static constexpr MotorID PITCH_MOTOR_ID = MotorID::MOTOR6;

static constexpr bool YAW_DIRECTION = true;
static constexpr bool PITCH_DIRECTION = true;
/**
 * This max output is measured in the c620 robomaster translated current.
 * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
 * The corresponding speed controller output torque current range is
 * -20 ~ 0 ~ 20 A.
 */
static constexpr float VELOCITY_PID_MAX_OUTPUT = 16000.0f;
static constexpr float POSITION_PID_MAX_OUTPUT = 16000.0f;

static constexpr float GIMBAL_X_OFFSET = 0.0f;
static constexpr float GIMBAL_Y_OFFSET = 0.0f;

static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

static constexpr float YAW_START_ANGLE = 135.0f;
static constexpr float PITCH_START_ANGLE = 2.5f;

static constexpr float PITCH_SOFTSTOP_LOW = 346.0f;
static constexpr float PITCH_SOFTSTOP_HIGH = 44.75f;

static const Matrix<float, 1, 3> ROBOT_STARTING_POSITION = Matrix<float, 1, 3>::zeroMatrix();

static constexpr int DEFAULT_BURST_LENGTH = 5;  // darts lol