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
    // clang-format off
    static constexpr std::array vertices{
        // positions            // colors               // texture coords
        0.5f, 0.5f, 0.0f,       1.0f, 0.0f, 0.0f,       1.0f, 1.0f, // top right
        0.5f, -0.5f, 0.0f,      0.0f, 1.0f, 0.0f,       1.0f, 0.0f, // bottom right
        -0.5f, -0.5f, 0.0f,     0.0f, 0.0f, 1.0f,       0.0f, 0.0f, // bottom left
        -0.5f, 0.5f, 0.0f,      1.0f, 1.0f, 0.0f,       0.0f, 1.0f  // top left
    };
    // clang-format on

    static constexpr std::array indices{
        0, 1, 3, // first triangle
        1, 2, 3  // second triangle
    };

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