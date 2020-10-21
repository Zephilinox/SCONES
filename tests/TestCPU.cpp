//STD
#include <iostream>

//LIBS
#include <doctest/doctest.h>
#include <SCONES/CPU.hpp>
#include <SCONES/Cartridge.hpp>

//SELF

TEST_CASE("CPU Nestest")
{
    //https://www.qmtpro.com/~nes/misc/nestest.txt

    Cartridge cartridge;
    REQUIRE(cartridge.loadFromFile("nestest.nes"));

    Bus bus;
    bus.write(0x8000, cartridge.getROM().data(), cartridge.getROM().size());
    bus.write(0xC000, cartridge.getROM().data(), cartridge.getROM().size());
    bus.insert_cartridge(std::move(cartridge));

    auto diff = 0xC000 - 0x8000;
    auto start = bus.read(0x8000);
    auto start1 = bus.read(0x8100);
    auto end = bus.read(0x8100);
    auto pc = bus.read(0xC000 - 1);
    auto pc2 = bus.read(0xC000);

    REQUIRE(start != 0);
    REQUIRE(pc != 0);

    CPU cpu(&bus);
    cpu.reset();
    cpu.set_program_counter(0xC000);

    while (true) {
        cpu.step();
        if (cpu.get_program_counter() == 0)
          break;
    }

    auto a = bus.read(0x2);
    auto b = bus.read(0x3);

        
}