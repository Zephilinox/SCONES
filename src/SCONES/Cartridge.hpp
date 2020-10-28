#pragma once

//SELF
#include <cstdint>
#include <vector>
#include <memory>

//SELF
#include "Mapper.hpp"

class Cartridge
{
public:
    bool loadFromFile(const char* path);

    struct ReadData
    {
        bool handled_by_cartridge = false;
        std::uint8_t data = 0;
    };

    ReadData cpu_read(std::uint16_t address);
    bool cpu_write(std::uint16_t address, std::uint8_t data);

    ReadData ppu_read(std::uint16_t address);
    bool ppu_write(std::uint16_t address, std::uint8_t data);

    [[nodiscard]] const std::vector<std::uint8_t>& getROM() const;
    [[nodiscard]] const std::vector<std::uint8_t>& getVROM() const;
    [[nodiscard]] std::uint8_t getMapperID() const;
    [[nodiscard]] Mapper* getMapper() const;
    [[nodiscard]] std::uint8_t getNameTable() const;
    [[nodiscard]] bool hasExtendedRam() const;

private:
    std::unique_ptr<Mapper> create_mapper(std::uint8_t mapperID) const;

    std::unique_ptr<Mapper> mapper;

    std::vector<std::uint8_t> PRGROM;
    std::vector<std::uint8_t> CHRROM;
    std::uint8_t  nameTableMirroring = 0;
    std::uint8_t mapperNumber = 0;
    bool extendedRAM = false;
    std::uint8_t prg_banks = 0;
    std::uint8_t chr_banks = 0;
};