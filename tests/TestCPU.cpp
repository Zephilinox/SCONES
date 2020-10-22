//STD
#include <iostream>
#include <fstream>

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

    CPU cpu(&bus);
    cpu.reset();
    cpu.set_program_counter(0xC000);

    auto level = spdlog::get_level();
    spdlog::set_level(spdlog::level::info);

    while (cpu.get_clock() <= 30000)
    {
        cpu.step();
        if (cpu.get_program_counter() == 0)
          break;
    }

    spdlog::set_level(level);

    auto nestest_log_file = std::ifstream("nestest_log.txt");
    auto dissassembly_log_file = std::ifstream("dissasembly.txt");

    std::vector<std::string> nestest_log_file_lines;
    std::vector<std::string> dissassembly_log_file_lines;

    std::string line;

    while (std::getline(nestest_log_file, line))
        nestest_log_file_lines.push_back(line);

    while (std::getline(dissassembly_log_file, line))
        dissassembly_log_file_lines.push_back(line);

    int base_cycle_divergence = 0;

    for (unsigned i = 0; i < nestest_log_file_lines.size(); ++i)
    {
        const auto& nestest_line = nestest_log_file_lines[i];

        REQUIRE(dissassembly_log_file_lines.size() > i);

        const auto& dissassembly_line = dissassembly_log_file_lines[i];

        const auto string_check = [&dissassembly_line, &nestest_line](int offset, int count) {
            CHECK(nestest_line.substr(offset, count) == dissassembly_line.substr(offset, count));
        };

        INFO("Line " << i + 1);

        {
            INFO("Program Counter");
            string_check(0, 4);
        }

        {
            INFO("Opcode");
            string_check(6, 3);
        }

        //Opcode Name
        {
            INFO("Opcode Name");
            string_check(16, 4);
        }
        //Accumulator
        string_check(48, 4);
        //X
        string_check(53, 4);
        //Y
        string_check(58, 4);
        //P
        string_check(63, 4);
        //SP
        string_check(68, 5);

        //Cycle difference
        int cyc_pos = 90;
        int nes_cyc_digits = static_cast<int>(nestest_line.size()) - cyc_pos;
        int our_cyc_digits = static_cast<int>(dissassembly_line.size()) - cyc_pos;

        int nes_cycles = std::stoi(nestest_line.substr(cyc_pos, nes_cyc_digits));
        int our_cycles = std::stoi(dissassembly_line.substr(cyc_pos, our_cyc_digits));
        int cycle_divergence = nes_cycles - our_cycles;

        if (i == 0)
            base_cycle_divergence = cycle_divergence;

        {
            INFO("Cycles diverged more than the starting divergence of " << base_cycle_divergence << ". " << nes_cycles << " - " << our_cycles << " = " << cycle_divergence);
            CHECK(cycle_divergence == base_cycle_divergence);
        }

        //Stop if program counter is wrong
        const bool program_counter_correct = nestest_line.substr(0, 4) == dissassembly_line.substr(0, 4);
        REQUIRE(program_counter_correct == true);
    }

    REQUIRE(nestest_log_file_lines.size() == dissassembly_log_file_lines.size());
}