#include "NES.hpp"

NES::NES()
    : cpu6502(std::make_unique<CPU>(&addressBus))
    , fb(std::make_unique<Framebuffer>(256, 240))
    , ppu2C02(std::make_unique<PPU>(&addressBus, fb.get()))
{
}

void NES::tick()
{
    clock++;
    ppu2C02->step();
    if (clock % 3) {
        cpu6502->step();
    }
}

void NES::reset()
{
	// TODO - Call Reset Signals on our NES.
}
