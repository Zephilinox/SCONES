#pragma once

#include <cstdint>
#include <memory>

#include "Framebuffer.hpp"

class Bus;

constexpr std::uint32_t PPU_MAX_ADDRESSABLE_MEMORY = 16 * 1024;
constexpr std::uint32_t PPU_PAL_PALETTE_SIZE = 0x40;

// PPU I/O Addresses
constexpr std::uint8_t PPU_ADDRESS_CONTROL_REG_1 = 0x2000;
constexpr std::uint8_t PPU_ADDRESS_CONTROL_REG_2 = 0x2001;
constexpr std::uint8_t PPU_ADDRESS_STATUS_REG = 0x2002;
constexpr std::uint8_t PPU_ADDRESS_SPR_RAM_ADDRESS_REG = 0x2003;
constexpr std::uint8_t PPU_ADDRESS_SPR_RAM_IO_REG = 0x2004;
constexpr std::uint8_t PPU_ADDRESS_VRAM_REG_1 = 0x2005;
constexpr std::uint8_t PPU_ADDRESS_VRAM_REG_2 = 0x2006;
constexpr std::uint8_t PPU_ADDRESS_VRAM_IO_REG = 0x2007;
constexpr std::uint8_t PPU_ADDRESS_OAMDMA = 0x4014;

// PPU Rendering constants.
constexpr std::uint32_t PPU_SCANLINE_CYCLE_COUNT = 341;
constexpr std::uint32_t PPU_TOTAL_SCANLINES_PER_FRAME = 262;

// MEGA TODO - Define data structures for the Status registers based on PPU notes.
// Ensure that state is easy to change.
// Map out process and being render cycle implementation.


// Register Data structures. (Makes data modification and lookup easier.)
union PPUControlReg {
    struct
    {
        std::uint8_t nametable_select : 2;
        std::uint8_t increment_mode : 1;
        std::uint8_t sprite_tile : 1;
        std::uint8_t background_tile : 1;
        std::uint8_t sprite_height : 1;
        std::uint8_t ppu_master_slave : 1;
        std::uint8_t nmi_enable : 1;
    } bits;
    std::uint8_t data = 0x0;
};

// Register render masks.
union PPURenderControlReg
{
    struct
    {
        std::uint8_t greyscale : 1;
        std::uint8_t background_left_column : 1;
        std::uint8_t sprite_left_column : 1;
        std::uint8_t background_enable : 1;
        std::uint8_t sprite_enable : 1;
        std::uint8_t BGR_colour_emphesis : 3;
    } bits;
    std::uint8_t data = 0x0;
};

union PPUStatusReg
{
    struct
    {
        std::uint8_t previous_bits_written_ppu : 5;
        std::uint8_t sprite_overflow : 1;
        std::uint8_t sprite_0_hit : 1;
        std::uint8_t vblank : 1;
    } bits;
    std::uint8_t data = 0x0;
};

union PPUScrollReg
{
    struct
    {
        std::uint8_t scroll_position_x : 4;
        std::uint8_t scroll_position_y : 4;
    } bits;

    std::uint8_t data = 0x0;
};


class PPU
{
public:
    PPU(Bus* bus, Framebuffer* fbuffer);
    ~PPU(); 

    void step();
    void create_palette();

private:
    // Internal functions
    void set_vblank();

private:
    Bus* addBus = nullptr;
    Framebuffer* fb = nullptr;
    std::unique_ptr<std::uint8_t[]> vram;

    std::uint16_t patternDataRegister[2];
    std::uint8_t  paletteAtributesRegister[2];

    RGB palPalette[PPU_PAL_PALETTE_SIZE];

    std::uint32_t clock = 0;
    std::uint32_t scanline = 0;
    bool frameready = false;
};