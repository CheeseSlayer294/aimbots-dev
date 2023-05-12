#include "kinematic_informant.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

namespace src::Informants {

KinematicInformant::KinematicInformant(src::Drivers* drivers) : drivers(drivers) {}

void KinematicInformant::initialize(float imuFrequency, float imukP, float imukI) {
    drivers->bmi088.initialize(imuFrequency, imukP, imukI);
}

void KinematicInformant::recalibrateIMU() { drivers->bmi088.requestRecalibration(); };

tap::communication::sensors::imu::ImuInterface::ImuState KinematicInformant::getIMUState() {
    return drivers->bmi088.getImuState();
}

void KinematicInformant::updateChassisIMUAngles() {
    Vector3f imuAngles = {
        drivers->bmi088.getRoll(),  // swaps roll and pitch axes
        drivers->bmi088.getPitch(),
        drivers->bmi088.getYaw() - 180.0f};  // for some reason yaw is 180.0 degrees off at calibration

    // Gets chassis angles
    Vector3f chassisAngles =
        drivers->kinematicInformant.getRobotFrames()
            .getFrame(Transformers::FrameType::CHASSIS_IMU_FRAME)
            .getPointInFrame(
                drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::CHASSIS_FRAME),
                imuAngles);

    chassisIMUAngles = chassisAngles * (M_PI / 180.0f);  // convert to radians
}

float KinematicInformant::getChassisIMUAngle(AngularAxis axis, AngleUnit unit) {
    float angle = 0.0f;

    switch (axis) {
        case YAW_AXIS: {
            angle = chassisIMUAngles.getZ();
            break;
        }
        case PITCH_AXIS: {
            angle = chassisIMUAngles.getX();
            break;
        }
        case ROLL_AXIS: {
            angle = chassisIMUAngles.getY();
            break;
        }
    }

    return unit == AngleUnit::Degrees ? angle * (180.0f / M_PI) : angle;
}

float KinematicInformant::getIMUAngularVelocity(AngularAxis axis, AngleUnit unit) {
    float angularVelocity = 0.0f;
    switch (axis) {
        case YAW_AXIS: {
            angularVelocity = -drivers->bmi088.getGz();
            break;
        }
        case PITCH_AXIS: {
            angularVelocity = drivers->bmi088.getGy();
            float roll = drivers->bmi088.getGx();
            tap::algorithms::rotateVector(
                &angularVelocity,
                &roll,
                getChassisIMUAngle(YAW_AXIS, AngleUnit::Radians) + DEV_BOARD_YAW_OFFSET);
            break;
        }
        case ROLL_AXIS: {
            float pitch = drivers->bmi088.getGy();
            angularVelocity = drivers->bmi088.getGx();
            tap::algorithms::rotateVector(
                &pitch,
                &angularVelocity,
                getChassisIMUAngle(YAW_AXIS, AngleUnit::Radians) + DEV_BOARD_YAW_OFFSET);
            break;
        }
    }
    return unit == AngleUnit::Degrees ? angularVelocity : modm::toRadian(angularVelocity);
}

float KinematicInformant::getIMUAngularAcceleration(AngularAxis axis, AngleUnit unit) {
    UNUSED(unit);  // fuck units always return in radians
    return imuAngularState[axis].getAcceleration();
}

void KinematicInformant::updateChassisAcceleration() {
    float wx = getIMUAngularVelocity(PITCH_AXIS, AngleUnit::Radians);
    float wy = getIMUAngularVelocity(ROLL_AXIS, AngleUnit::Radians);
    float wz = getIMUAngularVelocity(YAW_AXIS, AngleUnit::Radians);

    Vector3f w = {wx, wy, wz};

    float alphax = getIMUAngularAcceleration(PITCH_AXIS, AngleUnit::Radians);
    float alphay = getIMUAngularAcceleration(ROLL_AXIS, AngleUnit::Radians);
    float alphaz = getIMUAngularAcceleration(YAW_AXIS, AngleUnit::Radians);

    Vector3f alpha = {alphax, alphay, alphaz};

    float ax = drivers->bmi088.getAx();
    float ay = drivers->bmi088.getAy();
    float az = drivers->bmi088.getAz();

    Vector3f a = {ax, ay, az};

    Vector3f linearChassisAcceleration = a - (alpha ^ IMU_MOUNT_POSITION) - (w ^ (w ^ IMU_MOUNT_POSITION));
    chassisLinearState[1].updateFromAcceleration(linearChassisAcceleration.getX());
    chassisLinearState[0].updateFromAcceleration(linearChassisAcceleration.getY());
    chassisLinearState[2].updateFromAcceleration(linearChassisAcceleration.getZ());
}

ContiguousFloat KinematicInformant::getCurrentFieldRelativeYawAngleAsContiguousFloat() { return ContiguousFloat(0, -1, 1); }

void KinematicInformant::updateRobotFrames() {
    updateChassisIMUAngles();

    robotFrames.updateFrames(
        gimbalSubsystem->getChassisRelativeYawAngle(AngleUnit::Radians),
        gimbalSubsystem->getChassisRelativePitchAngle(AngleUnit::Radians),
        getChassisIMUAngle(YAW_AXIS, AngleUnit::Radians),
        {0, 0, 0},
        AngleUnit::Radians);

    imuLinearState[X_AXIS].updateFromAcceleration(drivers->bmi088.getAx());
    imuLinearState[Y_AXIS].updateFromAcceleration(drivers->bmi088.getAz());
    imuLinearState[Z_AXIS].updateFromAcceleration(drivers->bmi088.getAy());

    imuAngularState[X_AXIS].updateFromPosition(getIMUAngularVelocity(PITCH_AXIS, AngleUnit::Radians));
    imuAngularState[Y_AXIS].updateFromPosition(getIMUAngularVelocity(ROLL_AXIS, AngleUnit::Radians));
    imuAngularState[Z_AXIS].updateFromPosition(getIMUAngularVelocity(YAW_AXIS, AngleUnit::Radians));

    imuAngularState[X_AXIS].updateFromVelocity(getChassisIMUAngle(PITCH_AXIS, AngleUnit::Radians), false);
    imuAngularState[Y_AXIS].updateFromVelocity(getChassisIMUAngle(ROLL_AXIS, AngleUnit::Radians), false);
    imuAngularState[Z_AXIS].updateFromVelocity(getChassisIMUAngle(YAW_AXIS, AngleUnit::Radians), false);

    updateChassisAcceleration();
}

void KinematicInformant::mirrorPastRobotFrame(uint32_t frameDelay_ms) {
    std::pair<float, float> gimbalAngles = gimbalSubsystem->getGimbalOrientationAtTime(frameDelay_ms);
    robotFrames.mirrorPastCameraFrame(gimbalAngles.first, gimbalAngles.second, AngleUnit::Radians);
}

}  // namespace src::Informants