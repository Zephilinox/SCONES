#include "Bus.hpp"

//LIBS
#include <spdlog/spdlog.h>

Bus::Bus()
    : memory_cpu_ram(std::make_unique<std::array<std::uint8_t, ADDRESS_CPU_RAM_SIZE>>())
    , memory_ppu_register(std::make_unique<std::array<std::uint8_t, ADDRESS_PPU_REGISTER_SIZE>>())
    , memory_apu_io(std::make_unique<std::array<std::uint8_t, ADDRESS_APU_IO_REGISTER_SIZE>>())
    , memory_apu_io_disabled(std::make_unique<std::array<std::uint8_t, ADDRESS_APU_IO_DISABLED_REGISTER_SIZE>>())
    , memory_cartridge(std::make_unique<std::array<std::uint8_t, ADDRESS_CARTRIDGE_SIZE>>())
{
}

void Bus::insert_cartridge(Cartridge&& c)
{
    cartridge = std::move(c);
}

std::uint8_t Bus::cpu_read(std::uint16_t address)
{
    //todo: peak-only read which doesn't allow for the read to cause side-effects in the system. Used for debugging
    spdlog::trace("[BUS] read from {:#x}", address);

    std::uint8_t value = 0;

    const auto cartridge_read = cartridge.cpu_read(address);

    if (cartridge_read.handled_by_cartridge)
    {
        value = cartridge_read.data;
    }
    else if (address <= ADDRESS_CPU_RAM_UPPER_MIRROR) //lower 13 bits (8kb)
    {
        //8kb (2kb mirror) internal RAM
        //only the lower 11 bits (2kb) matter, the upper 5 bits are ignored, causing 2kb to mirror to 8kb
        const auto relevant_address = address & (ADDRESS_CPU_RAM_SIZE - 1);
        value = (*memory_cpu_ram)[relevant_address];
    }
    else if (address <= ADDRESS_PPU_REGISTER_UPPER_MIRROR) //lower 14 bits (16kb)
    {
        //PPU registers (8 bits) mirror until 0x3FFF
        //todo
        const auto relevant_address = address & (ADDRESS_PPU_REGISTER_SIZE - 1);
        if (ppu)
        {
            value = ppu->bus_read(relevant_address);
        }
        else
        {
            spdlog::error("PPU not mapped to address bus. Address fetch failed.");
        }
    }
    else if (address <= ADDRESS_APU_IO_REGISTER_UPPER)
    {
        //APU and I/O registers (18 bits)
        //todo
        const auto relevant_address = address - ADDRESS_PPU_REGISTER_UPPER_MIRROR - 1;
        value = (*memory_apu_io)[relevant_address];
        spdlog::warn("Reading apu io space but not mapped to component");
    }
    else if (address <= ADDRESS_APU_IO_DISABLED_REGISTER_UPPER)
    {
        //APU and I/O registers that are normally disabled (CPU test mode, 8 bits)
        //todo
        const auto relevant_address = address - ADDRESS_APU_IO_REGISTER_UPPER - 1;
        value = (*memory_apu_io_disabled)[relevant_address];
        spdlog::warn("Reading apu io disabled space but not mapped to component");
    }
    else
    {
        spdlog::error("Read accessed cartridge space but the cartridge did not handle the operation");
    }

    spdlog::trace("[BUS] read from {:#x} -> {:#x}", address, value);
    return value;
}

void Bus::cpu_write(std::uint16_t address, std::uint8_t data)
{
    spdlog::trace("[BUS] write {:#x} to {:#x}", data, address);

    if (cartridge.cpu_write(address, data))
    {
        
    }
    else if (address <= ADDRESS_CPU_RAM_UPPER_MIRROR) //lower 13 bits (8kb)
    {
        //8kb (2kb mirror) internal RAM
        //only the lower 11 bits (2kb) matter, the upper 5 bits are ignored, causing 2kb to mirror to 8kb
        const auto relevant_address = address & (ADDRESS_CPU_RAM_SIZE - 1);
        (*memory_cpu_ram)[relevant_address] = data;
    }
    else if (address <= ADDRESS_PPU_REGISTER_UPPER_MIRROR) //lower 14 bits (16kb)
    {
        //PPU registers (8 bits) mirror until 0x3FFF
        //todo
        const auto relevant_address = address & (ADDRESS_PPU_REGISTER_SIZE - 1);
        if (ppu)
        {
            ppu->bus_write(relevant_address, data);
        }
        else
        {
            spdlog::error("PPU not mapped to address bus. Address write failed.");
        }
    }
    else if (address <= ADDRESS_APU_IO_REGISTER_UPPER)
    {
        //APU and I/O registers (18 bits)
        //todo
        const auto relevant_address = address - ADDRESS_PPU_REGISTER_UPPER_MIRROR - 1;
        (*memory_apu_io)[relevant_address] = data;
        spdlog::warn("Writing apu io space but not mapped to component");
    }
    else if (address <= ADDRESS_APU_IO_DISABLED_REGISTER_UPPER)
    {
        //APU and I/O registers that are normally disabled (CPU test mode, 8 bits)
        //todo
        const auto relevant_address = address - ADDRESS_APU_IO_REGISTER_UPPER - 1;
        (*memory_apu_io_disabled)[relevant_address] = data;
        spdlog::warn("Writing apu io disabled space but not mapped to component");
    }
    else
    {
        spdlog::error("Write accessed cartridge space but the cartridge did not handle the operation");
    }
}

void Bus::cpu_write(std::uint16_t address, const std::uint8_t* data_start, std::size_t data_size)
{
    assert(static_cast<int>(address) + data_size - 1 <= 0xFFFF);
    for (int i = 0; i < data_size; ++i)
        cpu_write(address + i, data_start[i]);
}

std::array<std::uint8_t, ADDRESS_CPU_RAM_SIZE>& Bus::get_cpu_ram() const
{
    return *memory_cpu_ram;
}
