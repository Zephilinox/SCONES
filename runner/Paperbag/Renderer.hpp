#pragma once

//STD

//LIBS

//SELF
#include "Window.hpp"

namespace paperbag
{

//todo: base class and shit
class Renderer
{
public:
    Renderer(Window* window);
    ~Renderer();

    SDL_GLContext& get_context();

private:
    Window* window = nullptr;
    SDL_GLContext context = nullptr;
};

}