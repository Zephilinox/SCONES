#include "Framebuffer.h"

Framebuffer::Framebuffer(int x, int y)
    : w(x), h(y)
{
}

RGB Framebuffer::operator()(int x, int y)
{
    int idx = (w * y) + x;
    if (idx > 0 && idx > w * h)
    {
        RGB col{ data[idx], data[idx + 1], data[idx + 2] };
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
