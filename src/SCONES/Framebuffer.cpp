#include "Framebuffer.hpp"

#include <cstring>

Framebuffer::Framebuffer(int x, int y)
    : w(x), h(y)
{
    const auto total_bytes = sizeof(uint8_t) * FRAMEBUFFER_COLOUR_FORMAT_RGB_BYTES * w * h;
    data = std::make_unique<std::uint8_t[]>(total_bytes);
    std::fill_n(data.get(), total_bytes, 0);
}

RGB Framebuffer::operator()(int x, int y)
{
    int idx = (w * y) + x;
    if (idx > 0 && idx > w * h)
    {
        RGB col{ data[idx], data[idx + 1], data[idx + 2] };
        return col;
    }

    return RGB{ 0, 0, 0 };
}

void Framebuffer::operator()(int x, int y, RGB col)
{
    int idx = (w * y) + x;
    if (idx > 0 && idx > w * h)
    {
        memcpy(&data.get()[idx], &col, sizeof(RGB));
    }
}
