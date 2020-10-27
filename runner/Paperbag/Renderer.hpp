#pragma once

//STD
#include <iostream>

//LIBS

//SELF
#include "Window.hpp"
#include "Pixel.hpp"
#include "Texture.hpp"
#include "RandomCrap.hpp"

namespace paperbag
{

class Texture;

//todo: base class and shit
class Renderer
{
public:
    Renderer(Window* window);
    ~Renderer();

    SDL_GLContext& get_context();

    std::unique_ptr<Texture> make_texture(std::vector<Pixel> p, unsigned int w, unsigned int h) const;

    void end();

private:
    Window* window = nullptr;
    SDL_GLContext context = nullptr;
};

}