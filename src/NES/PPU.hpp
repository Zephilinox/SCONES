#pragma once

#include <cstdint>
#include <memory>

class Bus;

constexpr std::uint32_t PPU_MAX_ADDRESSABLE_MEMEORY = 16 * 1024;

// PPU I/O Addresses
constexpr std::uint8_t PPU_ADRESS_CONTROL_REG_1 = 0x2000;
constexpr std::uint8_t PPU_ADRESS_CONTROL_REG_2 = 0x2001;
constexpr std::uint8_t PPU_ADRESS_STATUS_REG = 0x2002;
constexpr std::uint8_t PPU_ADRESS_SPR_RAM_ADDRESS_REG = 0x2003;
constexpr std::uint8_t PPU_ADRESS_SPR_RAM_IO_REG = 0x2004;
constexpr std::uint8_t PPU_ADRESS_VRAM_REG_1 = 0x2005;
constexpr std::uint8_t PPU_ADRESS_VRAM_REG_2 = 0x2006;
constexpr std::uint8_t PPU_ADRESS_VRAM_IO_REG = 0x2007;

class PPU
{
public:
    PPU(Bus* bus);
    ~PPU(); 



private:
    Bus* addBus = nullptr;
    std::unique_ptr<std::uint8_t[]> vram;
};