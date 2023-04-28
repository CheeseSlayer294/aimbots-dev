#pragma once
#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

#define TOKYO_COMPATIBLE

/**
 * @brief Defines the number of motors created for the chassis.
 */
static constexpr uint8_t DRIVEN_WHEEL_COUNT = 4;
static constexpr uint8_t MOTORS_PER_WHEEL = 1;

static constexpr uint8_t SHOOTER_MOTOR_COUNT = 2;

/**
 * @brief GIMBAL SETUP
 */
static constexpr CANBus YAW_GIMBAL_BUS = CANBus::CAN_BUS2;
static constexpr CANBus PITCH_GIMBAL_BUS = CANBus::CAN_BUS1;

static constexpr uint8_t YAW_MOTOR_COUNT = 2;
static constexpr uint8_t PITCH_MOTOR_COUNT = 1;

static const std::array<bool, YAW_MOTOR_COUNT> YAW_MOTOR_DIRECTIONS = {false, false};
static const std::array<MotorID, YAW_MOTOR_COUNT> YAW_MOTOR_IDS = {MotorID::MOTOR5, MotorID::MOTOR7};
static const std::array<const char*, YAW_MOTOR_COUNT> YAW_MOTOR_NAMES = {"Yaw Motor 1", "Yaw Motor 2"};
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */
static const std::array<float, YAW_MOTOR_COUNT> YAW_MOTOR_OFFSET_ANGLES = {
    wrapTo0To2PIRange(modm::toRadian(9.97f)),
    wrapTo0To2PIRange(modm::toRadian(-106.39f))};
static constexpr float YAW_AXIS_START_ANGLE = 0.0f;

static constexpr float GIMBAL_YAW_GEAR_RATIO = (1.0f / 2.0f);  // for 2023 Standard
/*Changing this means the encoder-readable range of the YAW axis is reduced to 360deg * GIMBAL_YAW_GEAR_RATIO before the
 * encoder readings will repeat. We will assume that the robot will be started within the same GIMBAL_YAW_GEAR_RATIO range
 * every time. We also assume that 1 / GIMBAL_YAW_GEAR_RATIO is an integer multiple of 360deg. */

static const std::array<bool, PITCH_MOTOR_COUNT> PITCH_MOTOR_DIRECTIONS = {true};
static const std::array<MotorID, PITCH_MOTOR_COUNT> PITCH_MOTOR_IDS = {MotorID::MOTOR6};
static const std::array<const char*, PITCH_MOTOR_COUNT> PITCH_MOTOR_NAMES = {"Pitch Motor 1"};
static const std::array<float, YAW_MOTOR_COUNT> PITCH_MOTOR_OFFSET_ANGLES = {wrapTo0To2PIRange(modm::toRadian(-144.88f))};
/* What motor angles ensures that the barrel is pointing straight forward and level relative to the robot chassis? */
static constexpr float PITCH_AXIS_START_ANGLE = 0.0f;

static constexpr float GIMBAL_PITCH_GEAR_RATIO = (30.0f / 102.0f);  // for 2023 Standard
/*Changing this means the encoder-readable range of the PITCH axis is reduced to 360deg * GIMBAL_PITCH_GEAR_RATIO before the
 * encoder readings will repeat. We will assume that the range of the pitch axis is hardware-limited to not exceed this
 * range, but the motor angle may cross 0 in this range. Example Range: 278deg to 28deg */

static constexpr float PITCH_AXIS_SOFTSTOP_LOW = modm::toRadian(-23.0f);
static constexpr float PITCH_AXIS_SOFTSTOP_HIGH = modm::toRadian(22.0f);
// LOW should be lesser than HIGH, otherwise switch the motor direction
// -------------------------------------------------------------------------------------------------------------------------

static constexpr float DEV_BOARD_YAW_OFFSET = M_PI_2;  // in radians

static Vector3f IMU_MOUNT_POSITION{0.0f, 0.0f, 0.0f};

static constexpr SmoothPIDConfig CHASSIS_VELOCITY_PID_CONFIG = {
    .kp = 18.0f,
    .ki = 0.0f,
    .kd = 1.0f,
    .maxICumulative = 10.0f,
    .maxOutput = M3508_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

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
/*static constexpr SmoothPIDConfig YAW_POSITION_PID_CONFIG = {
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
};*/

static constexpr SmoothPIDConfig YAW_POSITION_PID_CONFIG = {
    .kp = 200.0f,
    .ki = 3.0f,
    .kd = 50.0f,
    .maxICumulative = 10.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

/*static constexpr SmoothPIDConfig PITCH_POSITION_PID_CONFIG = {
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
};*/

static constexpr SmoothPIDConfig PITCH_POSITION_PID_CONFIG = {
    .kp = 100.0f,
    .ki = 0.0f,
    .kd = 50.0f,
    .maxICumulative = 1000.0f,
    .maxOutput = GM6020_MAX_OUTPUT,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

// VISION PID CONSTANTS
static constexpr SmoothPIDConfig YAW_VISION_PID_CONFIG = {
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

static constexpr SmoothPIDConfig PITCH_VISION_PID_CONFIG = {
    .kp = 1000.0f,
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

// clang-format off
static constexpr uint16_t shooter_speed_array[6] = {
    15, 3900,  // {ball m/s, flywheel rpm}
    18, 4500,
    30, 9000};
// clang-format on

static const Matrix<uint16_t, 3, 2> SHOOTER_SPEED_MATRIX(shooter_speed_array);

static constexpr float FEEDER_DEFAULT_RPM = 3000.0f;
static constexpr int DEFAULT_BURST_LENGTH = 5;  // balls

// CAN Bus 2
static constexpr CANBus CHASSIS_BUS = CANBus::CAN_BUS2;

static constexpr MotorID LEFT_BACK_WHEEL_ID = MotorID::MOTOR1;
static constexpr MotorID LEFT_FRONT_WHEEL_ID = MotorID::MOTOR2;
static constexpr MotorID RIGHT_FRONT_WHEEL_ID = MotorID::MOTOR3;
static constexpr MotorID RIGHT_BACK_WHEEL_ID = MotorID::MOTOR4;

// CAN Bus 1
static constexpr CANBus SHOOTER_BUS = CANBus::CAN_BUS1;
static constexpr CANBus FEED_BUS = CANBus::CAN_BUS1;

//
static constexpr MotorID FEEDER_ID = MotorID::MOTOR7;
//
static constexpr MotorID SHOOTER_1_ID = MotorID::MOTOR3;
static constexpr MotorID SHOOTER_2_ID = MotorID::MOTOR4;

static constexpr bool SHOOTER_1_DIRECTION = false;
static constexpr bool SHOOTER_2_DIRECTION = true;

static constexpr bool FEEDER_DIRECTION = false;

// Hopper constants
static constexpr tap::gpio::Pwm::Pin HOPPER_PIN = tap::gpio::Pwm::C1;

static constexpr float HOPPER_PWM_RAMP_SPEED = 0.01f;  // pwm percent per millisecond

static constexpr float HOPPER_MIN_PWM = DS3218_MIN_PWM;
static constexpr float HOPPER_MAX_PWM = DS3218_MAX_PWM;

static constexpr float HOPPER_MIN_ANGLE = 0.0f;
static constexpr float HOPPER_MAX_ANGLE = 270.0f;

static constexpr float HOPPER_OPEN_ANGLE = 60.0f;
static constexpr float HOPPER_CLOSED_ANGLE = 155.0f;

static constexpr uint32_t HOPPER_MIN_ACTION_DELAY = 1000;  // Minimum time in ms between hopper lid flips

/**
 * This max output is measured in the c620 robomaster translated current.
 * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
 * The corresponding speed controller output torque current range is
 * -20 ~ 0 ~ 20 A.
 */
static constexpr float VELOCITY_PID_MAX_OUTPUT = 16000.0f;
static constexpr float POSITION_PID_MAX_OUTPUT = 16000.0f;

// Mechanical chassis constants, all in m
/**
 * Radius of the wheels (m).
 */
static constexpr float WHEEL_RADIUS = 0.076;

static constexpr float WHEELBASE_WIDTH = 0.366f;

static constexpr float WHEELBASE_LENGTH = 0.366f;

static constexpr float GIMBAL_X_OFFSET = 0.0f;
static constexpr float GIMBAL_Y_OFFSET = 0.0f;

// Distance from gimbal to tip of barrel, in m
static constexpr float GIMBAL_BARREL_LENGTH = 0.1f;  // Measured from 2022 Standard
// 0.205f normally

static const Matrix<float, 1, 3> ROBOT_STARTING_POSITION = Matrix<float, 1, 3>::zeroMatrix();

static constexpr float CHASSIS_GEARBOX_RATIO = (1.0f / 19.0f);

static constexpr float CHASSIS_VELOCITY_YAW_FEEDFORWARD = 0.0f;

/**
 * Max wheel speed, measured in RPM of the 3508 motor shaft.
 */
static constexpr int MAX_3508_ENC_RPM = 7000;

// Power limiting constants, will explain later
static constexpr float POWER_LIMIT_SAFETY_FACTOR = 0.85f;
static constexpr float STARTING_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 10.0f;

/**
 * @brief Power constants for chassis
 */
static constexpr int MIN_WHEEL_SPEED_SINGLE_MOTOR = 4000;
static constexpr int MAX_WHEEL_SPEED_SINGLE_MOTOR = 8000;
static constexpr int MIN_CHASSIS_POWER = 40;
static constexpr int MAX_CHASSIS_POWER = 120;
static constexpr int WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE =
    (MAX_WHEEL_SPEED_SINGLE_MOTOR - MIN_WHEEL_SPEED_SINGLE_MOTOR) / (MAX_CHASSIS_POWER - MIN_CHASSIS_POWER);
static_assert(WHEEL_SPEED_OVER_CHASSIS_POWER_SLOPE >= 0);

/**
 * @brief Behavior constants for chassis
 */

/**
 * The minimum desired wheel speed for chassis rotation, measured in RPM before
 * we start slowing down translational speed.
 */
static constexpr float MIN_ROTATION_THRESHOLD = 800.0f;

static constexpr float FOLLOW_GIMBAL_ANGLE_THRESHOLD = modm::toRadian(20.0f);

static constexpr SmoothPIDConfig ROTATION_POSITION_PID_CONFIG = {
    .kp = 1.25f,
    .ki = 0.0f,
    .kd = 0.00625f,
    .maxICumulative = 10.0f,
    .maxOutput = 1.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 1.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 1.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

/**
 * @brief TOKYO CONSTANTS
 */
// Fraction that user input is multiplied by when "drifting"
static constexpr float TOKYO_TRANSLATIONAL_SPEED_MULTIPLIER = 0.6f;
// Fraction of the maximum translation speed for when rotation speed should be reduced
static constexpr float TOKYO_TRANSLATION_THRESHOLD_TO_DECREASE_ROTATION_SPEED = 0.5f;
// Fraction of max chassis speed applied to rotation speed
static constexpr float TOKYO_ROTATIONAL_SPEED_FRACTION_OF_MAX = 0.75f;
// Fraction to cut rotation speed by when the robot is "drifting"
static constexpr float TOKYO_ROTATIONAL_SPEED_MULTIPLIER_WHEN_TRANSLATING = 0.7f;
// Rotational speed increment per iteration to apply until rotation setpoint is reached
static constexpr float TOKYO_ROTATIONAL_SPEED_INCREMENT = 50.0f;  // rpm

/**
 * @brief Transformation Matrices, specific to robot
 */

// clang-format off
static Vector3f CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN{ // in meters
    -0.085f, // x
    0.07f, // y
    0.0f,  // z
};

static Vector3f TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN{
    0.0f, // x
    0.0f, // y
    0.0f  // z
};

static Vector3f CHASSIS_START_POSITION_RELATIVE_TO_WORLD{
    0.0f, // x
    0.0f, // y
    0.0f, // z
};

static Vector3f BARREL_POSITION_FROM_GIMBAL_ORIGIN{
    0.0f, //x = 0.04498
    0.0f, //y - does not matter too much 
    0.0f, //z = 0.01683
};
// clang-format on

static constexpr float CHASSIS_START_ANGLE_WORLD = 0.0f;  // theta (about z axis) IN DEGREES

static constexpr float CIMU_X_EULER = 0.0f;
static constexpr float CIMU_Y_EULER = 0.0f;  // XYZ Euler Angles, All in Degrees!!!
static constexpr float CIMU_Z_EULER = 0.0f;