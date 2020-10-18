#include "PPU.hpp"

PPU::PPU(Bus* bus)
    : addBus(bus)
    , vram(std::make_unique<std::uint8_t[]>(PPU_MAX_ADDRESSABLE_MEMEORY))
{
}

PPU::~PPU()
{

}

void PPU::step()
{
    // Tick the CPU.
}
