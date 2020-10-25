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

    //addressable range
    if (address <= 0xFFFF)
    {
        if (address <= 0x1FFF)
        {
            //8kb (2kb mirror) internal RAM
            //only the lower 11 bits matter, the 14th (16kb), 15th (32kb), and 16th (64kb) bits are ignored, causing 2kb to mirror to 8kb
            const auto relevant_address = address & 0x1FFF;
            value = ram.get()[relevant_address];
        }
        else if (address <= 0x3FFF)
        {
            //PPU registers (8 bits) mirror to 0x3FFF (14 bits)
            //todo
            auto relevant_address = address & 0x3FFF;
            assert(!"failed to access PPU registers, not implemented");
        }
        else if (address <= 0x4017)
        {
            //APU and I/O registers
            //todo
            assert(!"failed to access APU and I/O registers, not implemented");
        }
        else if (address <= 0x401F)
        {
            //APU and I/O registers that are normally disabled (CPU test mode)
            //todo
            assert(!"failed to access disabled APU and I/O registers, not implemented");
        }
        else
        {
            //cartridge space. PRG ROM, PRG RAM, and Mapper registers
            //todo
            //spdlog::warn("Accessing cartridge space but not mapped to loaded cartridge.");
            value = ram.get()[address];
        }
    }
    else
    {
        assert(!"address out of range");
    }

    spdlog::trace("[BUS] read from {:#x} -> {:#x}", address, value);
    return value;
}

void Bus::write(std::uint16_t address, std::uint8_t data)
{
    spdlog::trace("[BUS] write {:#x} to {:#x}", data, address);

    //addressable range
    if (address <= 0xFFFF)
    {
        if (address <= 0x1FFF)
        {
            //8kb (2kb mirror) internal RAM
            //only the lower 11 bits matter, the 14th (16kb), 15th (32kb), and 16th (64kb) bits are ignored, causing 2kb to mirror to 8kb
            const auto relevant_address = address & 0x1FFF;
            ram.get()[relevant_address] = data;
        }
        else if (address <= 0x3FFF)
        {
            //PPU registers (8 bits) mirror to 0x3FFF (14 bits)
            //todo
            auto relevant_address = address & 0x3FFF;
            assert(!"failed to access PPU registers, not implemented");
        }
        else if (address <= 0x4017)
        {
            //APU and I/O registers
            //todo
            assert(!"failed to access APU and I/O registers, not implemented");
        }
        else if (address <= 0x401F)
        {
            //APU and I/O registers that are normally disabled (CPU test mode)
            //todo
            assert(!"failed to access disabled APU and I/O registers, not implemented");
        }
        else
        {
            //cartridge space. PRG ROM, PRG RAM, and Mapper registers
            //todo
            //spdlog::warn("Accessing cartridge space but not mapped to loaded cartridge.");
            ram.get()[address] = data;
        }
    }
    else
    {
        assert(!"address out of range");
    }
}

void Bus::write(std::uint16_t address, const std::uint8_t* data_start, std::size_t data_size)
{
    std::copy_n(data_start, data_size, ram.get() + address);
}
