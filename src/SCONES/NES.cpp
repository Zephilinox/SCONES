#include "NES.hpp"

NES::NES()
    : cpu6502(std::make_unique<CPU>(&addressBus))
    , fb(std::make_unique<Framebuffer>(256, 240))
    , ppu2C02(std::make_unique<PPU>(&addressBus, fb.get()))
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
    cpu6502->reset();
}

bool NES::insert_cartridge(const std::string& path)
{
    Cartridge cartridge;
    const auto result = cartridge.loadFromFile(path.c_str());
    if (!result)
        return result;

    addressBus.insert_cartridge(std::move(cartridge));
    reset();
    return result;
}
