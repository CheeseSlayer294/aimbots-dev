#pragma once

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "modm/processing/resumable.hpp"
#include "drivers.hpp"

#include "hud_constants.hpp"

namespace src::HUD_Display {

class ReticleGrid : protected modm::Resumable<2> {
    public:
        ReticleGrid(src::Drivers &drivers, tap::communication::serial::RefSerialTransmitter &refSerialTransmitter);

    modm::ResumableResult<bool> sendInitialGraphics();

    modm::ResumableResult<bool> update();

    void initialize();

    private:
        static constexpr uint16_t LINE_THICKNESS = 1;



    
};
}