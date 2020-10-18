#include "NES.h"

NES::NES()
    : cpu6502(std::make_unique<CPU>(&addressBus))
{
}

void NES::tick()
{
    // TODO - Tick PPU here.
    if (clock % 3) {
        cpu6502->step();
    }
    clock++;
}

void NES::reset()
{
	// TODO - Call Reset Signals on our NES.
}
