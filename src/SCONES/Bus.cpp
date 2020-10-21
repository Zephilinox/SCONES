#include "Bus.hpp"

//LIBS
#include <spdlog/spdlog.h>

Bus::Bus()
    : ram(std::make_unique<std::uint8_t[]>(ADDRESS_BUS_RAM_SIZE))
{
}

void Bus::insert_cartridge(Cartridge&& cartridge)
{
    this->cartridge = std::move(cartridge);
}

std::uint8_t Bus::read(std::uint16_t address)
{
    //todo: peak-only read which doesn't allow for the read to cause side-effects in the system. Used for debugging
    spdlog::trace("[BUS] read from {:#x}", address);

    std::uint8_t value = 0;

    //todo: 0x1FFF
    if (address >= 0x0000 && address <= 0xFFFF)
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

void Bus::write(std::uint16_t address, const std::uint8_t* data_start, std::size_t data_size)
{
    std::copy_n(data_start, data_size, ram.get() + address);
}
