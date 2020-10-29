#pragma once

#include <cstdint>

namespace paperbag
{
    struct Pixel
    {
        std::uint8_t r = 0;
        std::uint8_t g = 0;
        std::uint8_t b = 0;
        std::uint8_t a = 0;
    };
}