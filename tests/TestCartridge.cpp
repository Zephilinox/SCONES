#include <doctest/doctest.h>
#include <string>
#include <iostream>

#include "SCONES/Cartridge.hpp"

TEST_CASE("Cartridge")
{
    SUBCASE("Can read file")
    {
        Cartridge cartridge;
        std::string filePath = "TestRom/other/nestest.nes";
        
        REQUIRE(cartridge.loadFromFile(filePath.c_str()));
        REQUIRE(cartridge.getROM().size() == 16384);
        REQUIRE(cartridge.getVROM().size() == 8192);
        REQUIRE(cartridge.getMapper() == 0);
        REQUIRE(cartridge.getNameTable() == 0);
    }
}