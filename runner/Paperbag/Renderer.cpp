#include "Renderer.hpp"

using namespace paperbag;

Renderer::Renderer(Window* w)
    : window(w)
{
    context = SDL_GL_CreateContext(window->get_sdl_handle());
    SDL_GL_MakeCurrent(window->get_sdl_handle(), context);
    SDL_GL_SetSwapInterval(1);

    bool err = gl3wInit() != 0;
    if (err)
        throw(fmt::format("Failed to initialize OpenGL loader!\n"));
}

Renderer::~Renderer()
{
    SDL_GL_DeleteContext(context);
}

SDL_GLContext& Renderer::get_context()
{
    return context;
}
