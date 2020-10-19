#pragma once

#include <cstdint>

#include "Bus.hpp"
#include "CPU.hpp"
#include "PPU.hpp"

class NES
{
public:
    NES();
    ~NES() = default;

    void tick();  // Update the processor clocks for the NES.
    void reset(); // Reset our NES console.

    [[nodiscard]] std::uint32_t get_clock() const { return clock; }

private:
    // NES components.
    std::unique_ptr<CPU> cpu6502;
    std::unique_ptr<PPU> ppu2C02;
    Bus addressBus;

    // Internal clock.
    uint32_t clock = 0;

    // display framebuffer.
    std::unique_ptr<Framebuffer> fb;
};