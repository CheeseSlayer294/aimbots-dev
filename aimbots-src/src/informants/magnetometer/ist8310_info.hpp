#pragma once


#include <modm/platform/gpio/gpio_A8.hpp>
#include <modm/platform/gpio/gpio_C9.hpp>
#include <modm/platform/gpio/gpio_G3.hpp>
#include <modm/platform/gpio/gpio_G6.hpp>
#include <modm/platform/i2c/i2c_master_3.hpp>


namespace src::Informants::Ist8310Info {

static constexpr uint8_t I2C_ADDRESS = 0x0E;
static constexpr float   RAW_TO_uT_FACTOR = 0.3f; // Conversion factor to get from raw data to uT (microteslas)

using I2CMaster = modm::platform::I2cMaster3;

using ResetPin     = modm::platform::GpioG6;
using DataReadyPin = modm::platform::GpioG3;
using SclPin       = modm::platform::GpioA8;
using SdaPin       = modm::platform::GpioC9;

enum class Register : uint8_t {
    WhoAmI         = 0x00,
    Status1        = 0x02,
    OutputData     = 0x03,
    OutputStatus2  = 0x09,
    Control1       = 0x0A,
    Control2       = 0x0B,
    OutputTempLow  = 0x1C,
    OutputTempHigh = 0x1D,
    AverageControl = 0x41,
};

enum class RegisterData : uint8_t {
    WhoAmI_DeviceID = 0x10,

    Control1_OutputDataRate_SingleMeasurement = 0x01,
    Control2_PowerOnReset                     = 0x01,

    Average_16x = 0x24,
};

} // namespace src::Informants::Ist8310Info