#pragma once

//STD

//LIBS
#include "IMGUI/imgui.h"

//SELF
#include "Paperbag/Window.hpp"
#include "Paperbag/Renderer.hpp"
#include "Paperbag/GUI.hpp"

class NotAGame
{
public:
    NotAGame();

    int run();

private:
    void input();
    void update();
    void render();

    paperbag::Window window;
    paperbag::Renderer renderer;
    paperbag::GUI gui;

    bool done = false;
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
};