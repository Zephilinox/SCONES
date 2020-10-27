#include "Window.hpp"


using namespace paperbag;

Window::Window(Settings s)
    : settings(std::move(s))
{
    if constexpr (!opengl_loader_gl3w)
        throw("GL3W issue");

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0)
        throw(fmt::format("Error: {}\n", SDL_GetError()));

    const char* glsl_version = "#version 130";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    sdl_window = SDL_CreateWindow(settings.title.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, settings.width, settings.height, window_flags);
}

Window::~Window()
{
    SDL_DestroyWindow(sdl_window);
    SDL_Quit();
}

void Window::render()
{
    SDL_GL_SwapWindow(sdl_window);
}

void Window::clear(float r, float g, float b, float a)
{
    ImGuiIO& io = ImGui::GetIO();
    glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
    glClearColor(r, g, b, a);
    glClear(GL_COLOR_BUFFER_BIT);
}

SDL_Window* Window::get_sdl_handle() const
{
    return sdl_window;
}
