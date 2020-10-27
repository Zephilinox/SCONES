#pragma once

//STD
#include <string>

//LIBS
#include <GL/gl3w.h>
#include <SDL.h>
#include <backends/imgui_impl_sdl.h>
#include <backends/imgui_impl_opengl3.h>
#include <spdlog/spdlog.h>

#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h> // Initialize with gl3wInit()
constexpr bool opengl_loader_gl3w = true;
#else
constexpr bool opengl_loader_gl3w = false;
#endif

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

    void render();
    void clear(float r, float g, float b, float a);

    SDL_Window* get_sdl_handle() const;

private:
    Settings settings;
    SDL_Window* sdl_window = nullptr;
};

}