#include "hud_display_command.hpp"
#include "hud_display.hpp"
#include "drivers.hpp"

namespace src::HUD_Display {

HUD_DisplayCommand::HUD_DisplayCommand(src::Drivers &drivers,HUD_DisplaySubsystem &HUD_Display) :
Command(), drivers(drivers), refSerialTransmitter(&drivers) {
    addSubsystemRequirement(&HUD_Display);

    
}

void HUD_DisplayCommand::initialize() {
    
    restart();  // restart protothread  <-- Directly copied from ARUW, no idea what's going on here but I figured it's important
    
    //Initialive other drawing commands here
}

void HUD_DisplayCommand::execute() {
    run();
}

//Not sure why, but the protothread macros only work within methods named run, so it has to be declared separately from execute
bool HUD_DisplayCommand::run() {
    PT_BEGIN();

    PT_WAIT_UNTIL(drivers.refSerial.getRefSerialReceivingData());

    //PT_CALL(booleanHudIndicators.sendInitialGraphics());

    while (true)
    {
        //PT_CALL(booleanHudIndicators.update());
        PT_YIELD();
    }
    PT_END();

}


}