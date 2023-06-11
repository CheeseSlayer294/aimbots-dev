#pragma once

#include <modm/processing/protothread.hpp>
#include <modm/processing/timer.hpp>
#include <modm/architecture/interface/i2c_device.hpp>

#include "utils/common_types.hpp"
#include "ist8310_info.hpp"

namespace src::Informants {

class Ist8310 : modm::I2cDevice<Ist8310Info::I2CMaster>, modm::pt::Protothread {
public:
    Ist8310()
        : modm::I2cDevice<Ist8310Info::I2CMaster>( Ist8310Info::I2C_ADDRESS )
        , timer( 50 )
    { }

    void init();
    bool update();

    float getLastX() { return x; }
    float getLastY() { return y; }
    float getLastZ() { return z; }

private:
    void parseRawData();

    modm::ResumableResult<bool> readRegister( Ist8310Info::Register reg, size_t size = 1 );
    modm::ResumableResult<bool> writeToRegister( Ist8310Info::Register reg, Ist8310Info::RegisterData data );

private:
    MilliTimeout timer;

    bool isDeviceVerified = false;
    uint8_t rawData[6];

    float x;
    float y;
    float z;
};

} // namespace src::Informants
