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

    bool insert_cartridge(const std::string& path);

    [[nodiscard]] std::uint32_t get_clock() const { return clock; }
    [[nodiscard]] Framebuffer* get_framebuffer() const { return fb.get(); }
    [[nodiscard]] const Framebuffer* get_debug_framebuffer();
    [[nodiscard]] bool frame_ready() const { return clock % 89342; }
    [[nodiscard]] Bus& get_bus() { return addressBus; }
    [[nodiscard]] CPU& get_cpu() { return *cpu6502; }
    [[nodiscard]] PPU& get_ppu() { return *ppu2C02; }

private:
    // NES components.
    std::unique_ptr<CPU> cpu6502;
    std::unique_ptr<PPU> ppu2C02;
    Bus addressBus;

    // Internal clock.
    uint32_t clock = 0;

    // display framebuffer.
    std::unique_ptr<Framebuffer> fb;
    Framebuffer patternTableView = Framebuffer(256, 256);
};