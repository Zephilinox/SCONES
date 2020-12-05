#include "NES.hpp"

NES::NES()
    : cpu6502(std::make_unique<CPU>(&addressBus))
    , fb(std::make_unique<Framebuffer>(256, 240))
    , ppu2C02(std::make_unique<PPU>(&addressBus, fb.get()))
{
    addressBus.connect_ppu(ppu2C02.get());
}

void NES::tick()
{
    ppu2C02->step();
    if (clock % 3) {
        cpu6502->step();
    }

    // Process interrupts.
    if (ppu2C02->nmi_set()) {
        ppu2C02->nmi_reset();
        cpu6502->interrupt_request();
    }

    clock++;
}

void NES::reset()
{
    // TODO - Call Reset Signals on our NES.
    ppu2C02->reset();
    cpu6502->reset();
    clock = 0;
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

const Framebuffer* NES::get_debug_framebuffer()
{
    ppu2C02->get_pallete_contents(&patternTableView);
    return const_cast<Framebuffer*>(&patternTableView);
}
