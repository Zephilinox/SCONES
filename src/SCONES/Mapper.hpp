#pragma once

//STD
#include <cstdint>

class Mapper
{
public:
    virtual ~Mapper() = default;

    struct MapperResult
    {
        bool address_was_mapped = false;
        std::uint16_t address = 0;
        bool handled_by_mapper = false;
    };

    virtual MapperResult cpu_map_write_address(std::uint16_t address, std::uint8_t data) = 0;
    virtual MapperResult cpu_map_read_address(std::uint16_t address) = 0;

    virtual MapperResult ppu_map_write_address(std::uint16_t address, std::uint8_t data) = 0;
    virtual MapperResult ppu_map_read_address(std::uint16_t address) = 0;
};