#pragma once

//STD
#include <array>
#include <vector>
#include <functional>

//LIBS

//SELF
#include "Pixel.hpp"

namespace paperbag
{

class Texture
{
public:
    Texture();
    void load(std::vector<Pixel> pixels, unsigned int width, unsigned int height);
    const std::vector<Pixel>& get_pixels() const;
    const Pixel& operator[](unsigned int index) const;

    void update(std::function<void(std::vector<Pixel>& pixels)> updater);
    unsigned int get_opengl_texture_id() const;

    [[nodiscard]] unsigned int get_width() const;
    [[nodiscard]] unsigned int get_height() const;

private:
    unsigned int width = 0;
    unsigned int height = 0;
    std::vector<Pixel> pixels;

    unsigned int opengl_texture_id = 0;
};

}