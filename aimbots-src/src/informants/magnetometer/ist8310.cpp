#include "ist8310.hpp"

#include <tap/board/board.hpp>


namespace src::Informants {


float DBG_magX_uT  = 420.0f;
float DBG_magY_uT  = 420.0f;
float DBG_magZ_uT  = 420.0f;

float DBG_magX_raw = 420.0f;
float DBG_magY_raw = 420.0f;
float DBG_magZ_raw = 420.0f;


void Ist8310::init()
{
    Ist8310Info::I2CMaster::connect<Ist8310Info::SdaPin::Sda, Ist8310Info::SclPin::Scl>();
    Ist8310Info::I2CMaster::initialize<Board::SystemClock, 100'000>();
}


bool Ist8310::update()
{
    PT_BEGIN();

    // Give the imu time to boot up
    PT_WAIT_UNTIL( timer.execute() );
    timer.restart( 33 );

    PT_CALL( readRegister( Ist8310Info::Register::WhoAmI ) );
    if( rawData[0] != (uint8_t)Ist8310Info::RegisterData::WhoAmI_DeviceID )
    {
        isDeviceVerified = false;
    }
    else
    {
        isDeviceVerified = true;
    }

    PT_CALL( writeToRegister( Ist8310Info::Register::Control1, Ist8310Info::RegisterData::Control1_OutputDataRate_SingleMeasurement ) );
    PT_CALL( writeToRegister( Ist8310Info::Register::AverageControl, Ist8310Info::RegisterData::Average_16x ) );

    for( ;; )
    {
        PT_WAIT_UNTIL( timer.execute() );

        PT_CALL( writeToRegister( Ist8310Info::Register::Control1, Ist8310Info::RegisterData::Control1_OutputDataRate_SingleMeasurement ) );
        PT_CALL( readRegister( Ist8310Info::Register::OutputData, 6 ) );

        parseRawData();

        PT_YIELD();
    }

    PT_END();
}


void Ist8310::parseRawData()
{
    float rawX = (float)( ( (uint16_t)rawData[1] << 8 ) | rawData[0] );
    float rawY = (float)( ( (uint16_t)rawData[3] << 8 ) | rawData[2] );
    float rawZ = (float)( ( (uint16_t)rawData[5] << 8 ) | rawData[4] );

    x = Ist8310Info::RAW_TO_uT_FACTOR * rawX;
    y = Ist8310Info::RAW_TO_uT_FACTOR * rawY;
    z = Ist8310Info::RAW_TO_uT_FACTOR * rawZ;

    DBG_magX_uT = x;
    DBG_magY_uT = y;
    DBG_magZ_uT = z;
    DBG_magX_raw = rawX;
    DBG_magY_raw = rawY;
    DBG_magZ_raw = rawZ;
}


modm::ResumableResult<bool> Ist8310::readRegister( Ist8310Info::Register reg, size_t size )
{
    RF_BEGIN();

    if ( size > 6 )
        size = 6; // no segfaults pls :)

    rawData[0] = (uint8_t)reg;
    transaction.configureWriteRead( rawData, 1, rawData, size );
    RF_END_RETURN_CALL( runTransaction() );
}


modm::ResumableResult<bool> Ist8310::writeToRegister( Ist8310Info::Register reg, Ist8310Info::RegisterData data )
{
    RF_BEGIN();

    rawData[0] = (uint8_t)reg;
    rawData[1] = (uint8_t)data;

    transaction.configureWrite( rawData, 2 );
    RF_END_RETURN_CALL( runTransaction() );
}

} // namespace src::Informants
