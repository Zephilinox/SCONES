#pragma once

#include <vector>

class Cartridge
{
public:
    Cartridge() = default;
    ~Cartridge() = default;
    
    bool loadFromFile(const char* path);
    
    const std::vector<std::uint8_t>& getROM() const { return PRGROM; }
    const std::vector<std::uint8_t>& getVROM() const { return CHRROM; }
    std::uint8_t getMapper() const { return mapperNumber; }
    std::uint8_t  getNameTable() const { return nameTableMirroring; }
    bool hasExtendedRam() const { return extendedRAM; };
private:
    std::vector<std::uint8_t> PRGROM;
    std::vector<std::uint8_t> CHRROM;
    std::uint8_t  nameTableMirroring = 0;
    std::uint8_t mapperNumber = 0;
    bool extendedRAM = false;
    bool chrRam;
};