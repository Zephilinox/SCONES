#pragma once

//STD
#include <cstdint>
#include <memory>

//SELF
#include "Cartridge.hpp"

constexpr std::int32_t ADDRESS_BUS_RAM_SIZE = 64 * 1024;

class Bus
{
public:
    Bus();

    void insert_cartridge(Cartridge&& cartridge);

    std::uint8_t read(std::uint16_t address);
    void write(std::uint16_t address, std::uint8_t data);
    void write(std::uint16_t address, const std::uint8_t* data_start, std::size_t data_size);

private:
    std::unique_ptr<std::uint8_t[]> ram;
    Cartridge cartridge;
};