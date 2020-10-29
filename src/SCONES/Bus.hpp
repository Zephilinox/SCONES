#pragma once

//STD
#include <cstdint>
#include <memory>

//SELF
#include "Cartridge.hpp"

//https://wiki.nesdev.com/w/index.php/CPU_memory_map
constexpr std::int32_t ADDRESS_CPU_RAM_SIZE = 0x800;
constexpr std::int32_t ADDRESS_CPU_RAM_UPPER = 0x07FF;
constexpr std::int32_t ADDRESS_CPU_RAM_UPPER_MIRROR = 0x1FFF;

constexpr std::int32_t ADDRESS_PPU_REGISTER_SIZE = 0x8;
constexpr std::int32_t ADDRESS_PPU_REGISTER_UPPER = 0x2007;
constexpr std::int32_t ADDRESS_PPU_REGISTER_UPPER_MIRROR = 0x3FFF;

constexpr std::int32_t ADDRESS_APU_IO_REGISTER_SIZE = 0x18;
constexpr std::int32_t ADDRESS_APU_IO_REGISTER_UPPER = 0x4017;

constexpr std::int32_t ADDRESS_APU_IO_DISABLED_REGISTER_SIZE = 0x8;
constexpr std::int32_t ADDRESS_APU_IO_DISABLED_REGISTER_UPPER = 0x401F;

constexpr std::int32_t ADDRESS_CARTRIDGE_SIZE = 0xBFE0;
constexpr std::int32_t ADDRESS_CARTRIDGE_UPPER = 0xFFFF;

class Bus
{
public:
    Bus();

    void insert_cartridge(Cartridge&& cartridge);

    std::uint8_t cpu_read(std::uint16_t address);
    void cpu_write(std::uint16_t address, std::uint8_t data);
    void cpu_write(std::uint16_t address, const std::uint8_t* data_start, std::size_t data_size);

    std::array<std::uint8_t, ADDRESS_CPU_RAM_SIZE>& get_cpu_ram() const;

private:
    std::unique_ptr<std::array<std::uint8_t, ADDRESS_CPU_RAM_SIZE>> memory_cpu_ram;
    std::unique_ptr<std::array<std::uint8_t, ADDRESS_PPU_REGISTER_SIZE>> memory_ppu_register;
    std::unique_ptr<std::array<std::uint8_t, ADDRESS_APU_IO_REGISTER_SIZE>> memory_apu_io;
    std::unique_ptr<std::array<std::uint8_t, ADDRESS_APU_IO_DISABLED_REGISTER_SIZE>> memory_apu_io_disabled;
    std::unique_ptr<std::array<std::uint8_t, ADDRESS_CARTRIDGE_SIZE>> memory_cartridge;
    Cartridge cartridge;
};