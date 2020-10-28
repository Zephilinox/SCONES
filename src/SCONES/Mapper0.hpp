#pragma once

//SELF
#include "Mapper.hpp"

class Mapper0 final : public Mapper
{
public:
    Mapper0(std::uint8_t prg_banks, std::uint8_t chr_banks);

    MapperResult cpu_map_write_address(std::uint16_t address, std::uint8_t data) override;
    MapperResult cpu_map_read_address(std::uint16_t address) override;

    MapperResult ppu_map_write_address(std::uint16_t address, std::uint8_t data) override;
    MapperResult ppu_map_read_address(std::uint16_t address) override;

private:
    std::uint8_t prg_banks = 0;
    std::uint8_t chr_banks = 0;
    bool mirror_prg = false;
    bool chr_is_ram = false;
};