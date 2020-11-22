#pragma once

#include <cstdint>
#include <memory>

constexpr int FRAMEBUFFER_COLOUR_FORMAT_RGB_BYTES = 3;

struct RGB
{
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
};

class Framebuffer {
public:
    Framebuffer(int w, int h);
    Framebuffer(const Framebuffer& obj);
    RGB operator()(int x, int y) const;
    void operator()(int x, int y, RGB col);

    int get_width() const { return w; }
    int get_height() const { return h; }

private:
    std::unique_ptr<std::uint8_t[]> data;
    int w, h; 
};