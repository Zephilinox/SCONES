#pragma once

#include <cstdint>
#include <memory>

#include "Framebuffer.hpp"

class Bus;

constexpr std::uint32_t PPU_MAX_ADDRESSABLE_MEMORY = 16 * 1024;
constexpr std::uint16_t PPU_MAX_OAM_REG_SIZE = 256;
constexpr std::uint32_t PPU_PAL_PALETTE_SIZE = 0x40;

// PPU I/O Addresses
constexpr std::uint8_t PPU_ADDRESS_PPUCTRL_REG = 0x2000;
constexpr std::uint8_t PPU_ADDRESS_MASK_REG = 0x2001;
constexpr std::uint8_t PPU_ADDRESS_STATUS_REG = 0x2002;
constexpr std::uint8_t PPU_ADDRESS_OAM_ADDR_REG = 0x2003;
constexpr std::uint8_t PPU_ADDRESS_OAM_DATA_REG = 0x2004;
constexpr std::uint8_t PPU_ADDRESS_SCROLL_REG = 0x2005;
constexpr std::uint8_t PPU_ADDRESS_ADDR_REG = 0x2006;
constexpr std::uint8_t PPU_ADDRESS_DATA_REG = 0x2007;
constexpr std::uint8_t PPU_ADDRESS_OAMDMA_REG = 0x4014;

// PPU Rendering constants.
constexpr std::uint32_t PPU_SCANLINE_CYCLE_COUNT = 341;
constexpr std::uint32_t PPU_TOTAL_SCANLINES_PER_FRAME = 262;

// PPU buffers for multicycle operations.
constexpr std::uint32_t PPU_SCROLL_BUFFER_COUNT = 2;
constexpr std::uint32_t PPU_SCROLL_BUFFER_FRONT = 0;
constexpr std::uint32_t PPU_SCROLL_BUFFER_BACK = 1;

// MEGA TODO - Define data structures for the Status registers based on PPU notes.
// Ensure that state is easy to change.
// Map out process and being render cycle implementation.

// Register Data structures. (Makes data modification and lookup easier.)
union PPUControlReg
{
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
    bool nmi_set() const { return nmi; }
    void nmi_reset() { nmi = false; }

    void bus_write(std::uint16_t address, std::uint8_t data);
    std::uint8_t bus_read(std::uint16_t address);

private:
    // Internal functions.
    void create_palette();

    // Register R/W event functions.
    void ppu_data_reg_write(std::uint8_t data);
    void ppu_scroll_reg_write(std::uint8_t data);
    void ppu_address_reg_write(std::uint8_t data);
    void ppu_data_status_reg_write(std::uint8_t data);

    void ppu_address_status_reg_read(std::uint8_t& data);
    void ppu_data_reg_read(std::uint8_t& data);
    void ppu_address_reg_read(std::uint8_t& data);

    void invert_latch_bit() { address_latch == 0x0 ? address_latch = 0x8 : address_latch = 0x0; }

    // Swap buffers every cycle.
    void swap_buffers();

private:
    Bus* addBus = nullptr;
    Framebuffer* fb = nullptr;
    std::unique_ptr<std::uint8_t[]> oam;

    // Internal CPU mapped registers.
    PPUControlReg PPUCTRL;
    PPURenderControlReg PPUMASK;
    PPUStatusReg PPUSTATUS;
    std::uint8_t PPUDATA;

    // Buffered registers (Due to multiple cycles).
    // Write to backbuffer then swap after reading. (Alllows operation to be deffered.)
    PPUScrollReg PPUSCROLL[PPU_SCROLL_BUFFER_COUNT];
    std::uint8_t PPUADDR[PPU_SCROLL_BUFFER_COUNT];

    // Specifies whether to write high or low bytes in 16bit address.
    std::uint8_t address_latch = 0x0;

    std::uint16_t patternDataRegister[2];
    std::uint8_t paletteAtributesRegister[2];

    RGB palPalette[PPU_PAL_PALETTE_SIZE];

    std::uint32_t clock = 0;
    std::int16_t scanline = -1;
    bool frameready = false;

    // NMI interrupt flag.
    bool nmi = false;
};