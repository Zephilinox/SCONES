#pragma once

//STD

//LIBS

//SELF
#include "Window.hpp"
#include "Renderer.hpp"

namespace paperbag
{

class GUI
{
public:
    GUI(Window* window, Renderer* renderer);
    ~GUI();

    void input(SDL_Event& event);
    void update();
    void render();

private:
    Window* window;
    Renderer* renderer;
};

}