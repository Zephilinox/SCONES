#include "Bus.hpp"

//LIBS
#include <spdlog/spdlog.h>

Bus::Bus()
    : ram(std::make_unique<std::uint8_t[]>(ADDRESS_BUS_RAM_SIZE))
{
}

std::uint8_t Bus::read(std::uint16_t address)
{
    //todo: peak-only read which doesn't allow for the read to cause side-effects in the system. Used for debugging
    spdlog::trace("[BUS] read from {:#x}", address);

    std::uint8_t value = 0;

    if (address >= 0x0000 && address <= 0x00FF)
    {
        value = ram.get()[address];
    }

    spdlog::trace("[BUS] read from {:#x} -> {:#x}", address, value);
    return value;
}

void Bus::write(std::uint16_t address, std::uint8_t data)
{
    spdlog::trace("[BUS] write {:#x} to {:#x}", data, address);

    if (address >= 0x0000 && address <= 0x00FF)
    {
        ram.get()[address] = data;
    }
}