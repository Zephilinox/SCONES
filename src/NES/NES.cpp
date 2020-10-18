#include "NES.hpp"

NES::NES()
    : cpu6502(std::make_unique<CPU>(&addressBus))
    , ppu2C02(std::make_unique<PPU>(&addressBus))
{
}

void NES::tick()
{
    ppu2C02->step();
    if (clock % 3) {
        cpu6502->step();
    }
    clock++;
}

void NES::reset()
{
	// TODO - Call Reset Signals on our NES.
}
