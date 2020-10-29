#pragma once

//STD

//LIBS
#include <IMGUI/imgui.h>
#include <SCONES/NES.hpp>

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

    NES nes;

    paperbag::Window window;
    paperbag::Renderer renderer;
    paperbag::GUI gui;

    bool done = false;
    bool show_demo_window = false;

    bool show_nes_screen = true;
    bool pause_nes_when_not_active = true;
    bool update_nes_per_instruction = false;
    bool update_nes_screen_every_cycle = false;
    bool show_nes_cpu_memory_editor = true;

    int nes_updates_per_frame = 1;

    std::string cartridge_path = "../../tests/resources/nestest.nes";
    std::string program_counter_value = "0xC000";

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    std::unique_ptr<paperbag::Texture> texture;
};