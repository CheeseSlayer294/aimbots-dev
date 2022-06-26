#include "reticle_grid.hpp"
#include "drivers.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

using namespace tap::communication::serial;

namespace src::HUD_Display {

    ReticleGrid::ReticleGrid(src::Drivers &drivers, tap::communication::serial::RefSerialTransmitter &refSerialTransmitter) {

    }

    modm::ResumableResult<bool> ReticleGrid::sendInitialGraphics() {
        //This piece is copy-pasted from ARUW for reference, haven't quite puzzled out all of the pieces yet.
        /*RF_BEGIN(0);

        // send reticle
        for (reticleIndex = 0; reticleIndex < MODM_ARRAY_SIZE(reticleMsg); reticleIndex++)
        {
            RF_CALL(refSerialTransmitter.sendGraphic(&reticleMsg[reticleIndex]));
        }

        RF_END();*/
    }

    modm::ResumableResult<bool> ReticleGrid::update() {
        RF_BEGIN(1);
        RF_END();
    }

}