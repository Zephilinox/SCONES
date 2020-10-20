#include "Cartridge.hpp"

#include <fstream>
#include <spdlog/spdlog.h>

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
    
    uint8_t banks = header[4];
    if (!banks)
    {
        return false;
    }
    
    uint8_t vBanks = header[5];
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
    
    PRGROM.resize((0x4000 * banks));
    if(!romFile.read(reinterpret_cast<char*>(&PRGROM[0]), 0x4000 * banks))
    {
        return false;
    }
    
    if (vBanks)
    {
        CHRROM.resize(0x2000 * vBanks);
        if(!romFile.read(reinterpret_cast<char*>(&CHRROM[0]), 0x2000 * vBanks))
        {
            spdlog::error("Could not read CHR-ROM");
            return false;
        }
    }
    else
    {
        spdlog::debug("Has CHR-RAM");
    }
    
    return true;
}
