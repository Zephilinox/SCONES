#include "Renderer.hpp"

//SELF
#include "Texture.hpp"

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

std::unique_ptr<Texture> Renderer::make_texture(std::vector<Pixel> p, unsigned int w, unsigned int h) const
{
    auto texture = std::make_unique<Texture>();
    texture->load(std::move(p), w, h);
    return texture;
}

void Renderer::end()
{
    //deal with rendering textures etc. does the actual drawing
}
