#include "Mapper0.hpp"

constexpr std::uint16_t prg_start_address = 0x8000;
constexpr std::uint16_t prg_mirror_address = 0x3FFF;
constexpr std::uint16_t prg_nonmirror_address = 0x7FFF;
constexpr std::uint16_t chr_end_address = 0x1FFF;

Mapper0::Mapper0(std::uint8_t prg_banks, std::uint8_t chr_banks)
    : prg_banks(prg_banks)
    , chr_banks(chr_banks)
    , mirror_prg(prg_banks <= 1)
    , chr_is_ram(chr_banks == 0)
{
}

Mapper::MapperResult Mapper0::cpu_map_write_address(std::uint16_t address, std::uint8_t data)
{
    if (address < prg_start_address)
        return {false, address};

    if (mirror_prg)
        return { true, static_cast<std::uint16_t>(address & prg_mirror_address)};

    return { true, static_cast<std::uint16_t>(address & prg_nonmirror_address) };
}

Mapper::MapperResult Mapper0::cpu_map_read_address(std::uint16_t address)
{
    if (address < prg_start_address)
        return {false, address};

    if (mirror_prg)
        return { true, static_cast<std::uint16_t>(address & prg_mirror_address) };

    return { true, static_cast<std::uint16_t>(address & prg_nonmirror_address) };
}

Mapper::MapperResult Mapper0::ppu_map_write_address(std::uint16_t address, std::uint8_t data)
{
    const bool valid_address = address <= chr_end_address;
    return { valid_address, address };
}

Mapper::MapperResult Mapper0::ppu_map_read_address(std::uint16_t address)
{
    const bool valid_address = address <= chr_end_address && chr_is_ram;
    return { valid_address, address };
}
