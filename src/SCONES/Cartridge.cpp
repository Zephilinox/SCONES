#include "Cartridge.hpp"

//STD
#include <fstream>

//LIBS
#include <spdlog/spdlog.h>

//SELF
#include "Mapper0.hpp"

bool Cartridge::loadFromFile(const char* path)
{
    std::ifstream romFile(path, std::ios_base::binary | std::ios_base::in);
    if (!romFile)
    {
        spdlog::error("Cannot load from file: {}\n", path);
        return false;
    }

    std::vector<uint8_t> header;
    
    //A bunch of checks to see if the ROM is valid
    header.resize(0x10);
    if (!romFile.read(reinterpret_cast<char*>(&header[0]), 0x10))
    {
        spdlog::error("Could not read header\n");
        return false;
    }
    if (std::string(&header[0], &header[4]) != "NES\x1A")
    {
        return false;
    }
    
    prg_banks = header[4];
    if (!prg_banks)
        return false;
    
    chr_banks = header[5];
    nameTableMirroring = header[6] & 0xB;
    
    mapperNumber = ((header[6] >> 4) & 0xf) | (header[7] & 0xf0);
    
    extendedRAM = header[6] & 0x2;
    
    if (header[6] & 0x4){
        return false;
    }
    
    if ((header[0xA] & 0x3) == 0x2 || (header[0xA] & 0x1))
    {
        spdlog::error("PAL ROM not supported");
        return false;
    }
    else
    {
        spdlog::debug("Using NTSC ROM");
    }
    
    PRGROM.resize((0x4000 * prg_banks));
    if (!romFile.read(reinterpret_cast<char*>(&PRGROM[0]), 0x4000 * prg_banks))
    {
        return false;
    }
    
    if (chr_banks)
    {
        CHRROM.resize(0x2000 * chr_banks);
        if (!romFile.read(reinterpret_cast<char*>(&CHRROM[0]), 0x2000 * chr_banks))
        {
            spdlog::error("Could not read CHR-ROM");
            return false;
        }
    }
    else
    {
        spdlog::debug("Has CHR-RAM");
    }

    mapper = create_mapper(mapperNumber);
    return true;
}

Cartridge::ReadData Cartridge::cpu_read(std::uint16_t address)
{
    const auto result = mapper->cpu_map_read_address(address);
    if (result.address_was_mapped && !result.handled_by_mapper)
        return { true, PRGROM[result.address] };

    return { result.address_was_mapped };
}

bool Cartridge::cpu_write(std::uint16_t address, std::uint8_t data)
{
    const auto result = mapper->cpu_map_write_address(address, data);
    if (result.address_was_mapped && !result.handled_by_mapper)
        PRGROM[result.address] = data;

    return result.address_was_mapped;
}

Cartridge::ReadData Cartridge::ppu_read(std::uint16_t address)
{
    const auto result = mapper->ppu_map_read_address(address);
    if (result.address_was_mapped)
        return { true, CHRROM[result.address] };

    return {};
}

bool Cartridge::ppu_write(std::uint16_t address, std::uint8_t data)
{
    const auto result = mapper->ppu_map_write_address(address, data);
    if (result.address_was_mapped)
        CHRROM[result.address] = data;

    return result.address_was_mapped;
}

const std::vector<std::uint8_t>& Cartridge::getROM() const
{
    return PRGROM;
}

const std::vector<std::uint8_t>& Cartridge::getVROM() const
{
    return CHRROM;
}

std::uint8_t Cartridge::getMapperID() const
{
    return mapperNumber;
}

Mapper* Cartridge::getMapper() const
{
    return mapper.get();
}

std::uint8_t Cartridge::getNameTable() const
{
    return nameTableMirroring;
}

bool Cartridge::hasExtendedRam() const
{
    return extendedRAM;
}

std::unique_ptr<Mapper> Cartridge::create_mapper(std::uint8_t mapperID) const
{
    switch (mapperID)
    {
    case 0:
        return std::make_unique<Mapper0>(prg_banks, chr_banks);
    default:
        spdlog::error("Mapper with id {} does not exist, but is being loaded.", mapperID);
        assert(false);
        return nullptr;
    }
}
