#include "Framebuffer.h"
#include "PPU.hpp"

PPU::PPU(Bus* bus, Framebuffer* fbuffer)
    : addBus(bus)
    , vram(std::make_unique<std::uint8_t[]>(PPU_MAX_ADDRESSABLE_MEMEORY))
    , fb(fbuffer)
{}

PPU::~PPU()
{

}

void PPU::step()
{
    // Tick the PPU.
    clock++;
    if (clock >= 341)
    {
        clock = 0;
        scanline++;
        if (scanline >= 261)
        {
            scanline = -1;
            frameready = true;
        }
    }
}
