#include "jetson_communicator.hpp"

#include <drivers.hpp>

#include "tap/communication/sensors/buzzer/buzzer.hpp"

#define READ(data, length) drivers->uart.read(JETSON_UART_PORT, data, length)
#define WRITE(data, length) drivers->uart.write(JETSON_UART_PORT, data, length)

namespace src::Informants::vision {

JetsonCommunicator::JetsonCommunicator(src::Drivers* drivers)
    : drivers(drivers),
      lastMessage(),
      currentSerialState(JetsonCommunicatorSerialState::SearchingForMagic),
      nextByteIndex(0),
      jetsonOfflineTimeout(),
#ifdef TARGET_SENTRY
      fieldRelativeYawAngleAtVisionUpdate(modm::toRadian(YAW_START_ANGLE)),
#else
      fieldRelativeYawAngleAtVisionUpdate(0.0f),
#endif
      chassisRelativePitchAngleAtVisionUpdate(modm::toRadian(PITCH_START_ANGLE))  //
{
}

void JetsonCommunicator::initialize() {
    jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);
    drivers->uart.init<JETSON_UART_PORT, JETSON_BAUD_RATE>();
}

uint8_t displayBuffer[JETSON_MESSAGE_SIZE];
int displayBufIndex = 0;

float yawOffsetDisplay = 0;
float pitchOffsetDisplay = 0;
CVState cvStateDisplay = CVState::NOT_FOUND;

float fieldRelativeYawAngleDisplay = 0;
float chassisRelativePitchAngleDisplay = 0;

int lastMsgTimeDisplay = 0;
int msBetweenLastMessageDisplay = 0;

/**
 * @brief Need to use modm's uart functions to read from the Jetson.
 * The Jetson sends information in the form of a JetsonMessage.
 *
 * We send a magic number from the Jetson to the Development Board as the header of every message, and we can use that magic
 * number to determine whether a message is (probably) going to be valid. If a long magic number comes through as valid, we can
 * assume that the rest of the message is valid as well.
 *
 * modm currently loads received bytes into an internal buffer, accessible using the READ() call.
 * When we receive the message-agnostic end byte we unload from the buffer, check the message length, and reinterpret a JetsonMessage
 * from our received bytes.
 */
void JetsonCommunicator::updateSerial() {
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();

    size_t bytesRead = READ(&rawSerialBuffer[nextByteIndex], 1);  // attempts to pull one byte from the buffer
    if (bytesRead != 1) return;

    // We've successfully read a new byte from the Jetson, so we can restart this.
    jetsonOfflineTimeout.restart(JETSON_OFFLINE_TIMEOUT_MILLISECONDS);

    displayBuffer[displayBufIndex] = rawSerialBuffer[0];            // copy byte to display buffer
    displayBufIndex = (displayBufIndex + 1) % JETSON_MESSAGE_SIZE;  // increment display index and wrap around if necessary

    switch (currentSerialState) {
        case JetsonCommunicatorSerialState::SearchingForMagic: {
            // Check if the byte we just read is the byte we expected in the magic number.
            if (rawSerialBuffer[nextByteIndex] == ((JETSON_MESSAGE_MAGIC >> (8 * nextByteIndex)) & 0xff)) {
                nextByteIndex++;
            } else {
                nextByteIndex = 0;  // if not, reset the index and start over.
            }

            // Wait until we've reached the end of the magic number. If any of the bytes in the magic number weren't a match, we
            // wouldn't have gotten this far.
            if (nextByteIndex == sizeof(decltype(JETSON_MESSAGE_MAGIC))) {
                currentSerialState = JetsonCommunicatorSerialState::AssemblingMessage;
            }
            break;
        }
        case JetsonCommunicatorSerialState::AssemblingMessage: {
            nextByteIndex++;

            // Increment the byte index until we reach the expected end of a message, then parse the message.
            if (nextByteIndex == JETSON_MESSAGE_SIZE) {
                // Reinterpret the received bytes into a JetsonMessage
                lastMessage = *reinterpret_cast<JetsonMessage*>(&rawSerialBuffer);

                // Update last message time and time between last message and now.
                if (lastMsgTimeDisplay == 0) {
                    lastMsgTimeDisplay = tap::arch::clock::getTimeMilliseconds();
                } else {
                    msBetweenLastMessageDisplay = currTime - lastMsgTimeDisplay;  // Should be pretty close to the message send rate.
                    lastMsgTimeDisplay = currTime;
                }

                yawOffsetDisplay = lastMessage.targetYawOffset;
                pitchOffsetDisplay = lastMessage.targetPitchOffset;
                cvStateDisplay = lastMessage.cvState;

                if (lastMessage.cvState >= CVState::FOUND) {  // If the CV state is FOUND or better
                    fieldRelativeYawAngleAtVisionUpdate = gimbal->getCurrentFieldRelativeYawAngle(AngleUnit::Radians);
                    chassisRelativePitchAngleAtVisionUpdate = gimbal->getCurrentChassisRelativePitchAngle(AngleUnit::Radians);

                    fieldRelativeYawAngleDisplay = fieldRelativeYawAngleAtVisionUpdate;
                    chassisRelativePitchAngleDisplay = chassisRelativePitchAngleAtVisionUpdate;

                    // Find the robot-relative target angles calculated at the vision update. Theoretically, we can snap to a target
                    // using just one update from the vision system, and this target is refreshed on every update of the vision system.
                    visionTargetAngles[0][yaw] = fieldRelativeYawAngleAtVisionUpdate + lastMessage.targetYawOffset;
                    visionTargetAngles[0][pitch] = chassisRelativePitchAngleAtVisionUpdate + lastMessage.targetPitchOffset;
                    // TODO: Explore using predictors to smoothen effect of large time gap between vision updates.
                }

                // Auditory indicator that helps debug our vision pipeline.
                if (lastMessage.cvState == CVState::FOUND) {
                    tap::buzzer::playNote(&drivers->pwm, 466);
                } else if (lastMessage.cvState == CVState::FIRE) {
                    tap::buzzer::playNote(&drivers->pwm, 932);
                } else {
                    tap::buzzer::playNote(&drivers->pwm, 0);
                }

                // As we've received a full message, reset the byte index and go back to searching for the magic number.
                nextByteIndex = 0;
                currentSerialState = JetsonCommunicatorSerialState::SearchingForMagic;
            }
            break;
        }
    }

    if (!isJetsonOnline()) {
        lastMessage.targetYawOffset = 0.0f;
        lastMessage.targetPitchOffset = 0.0f;
    }
}

}  // namespace src::Informants::vision
