#pragma once

//STD
#include <string>

//LIBS
#include <SDL.h>
#include <backends/imgui_impl_sdl.h>
#include <backends/imgui_impl_opengl3.h>
#include <spdlog/spdlog.h>
#include <glad/glad.h>

namespace paperbag
{
    
//todo: base class and shit
class Window
{
public:
    struct Settings
    {
        std::string title = "Title";
        unsigned int width = 1280;
        unsigned int height = 720;
    };

    Window(Settings settings);
    ~Window();

    void end();
    void clear(float r, float g, float b, float a);

    SDL_Window* get_sdl_handle() const;

private:
    Settings settings;
    SDL_Window* sdl_window = nullptr;
};

}