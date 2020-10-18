#pragma once

#include <cstdint>

#include "Bus.hpp"
#include "CPU.hpp"

class NES {
 public:
    NES();
    ~NES() = default;

   void tick(); // Update the processor clocks for the NES.
   void reset(); // Reset our NES console.

private:
   // NES components.
   std::unique_ptr<CPU> cpu6502;
   Bus addressBus;

   // Internal clock.
   uint32_t clock = 0;
};