#include "Renderer.hpp"

//SELF
#include "Texture.hpp"

using namespace paperbag;

const char vertex_shader[] = R"VERTEX(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
layout(location = 2) in vec2 aTexCoord;

out vec3 ourColor;
out vec2 TexCoord;

void main()
{
    gl_Position = vec4(aPos, 1.0);
    ourColor = aColor;
    TexCoord = aTexCoord;
}
)VERTEX";

const char fragment_shader[] = R"FRAGMENT(
#version 330 core
out vec4 FragColor;
  
in vec3 ourColor;
in vec2 TexCoord;

uniform sampler2D ourTexture;

void main()
{
    FragColor = texture(ourTexture, TexCoord);
}
)FRAGMENT";

Renderer::Renderer(Window* w)
    : window(w)
{
    context = SDL_GL_CreateContext(window->get_sdl_handle());
    SDL_GL_MakeCurrent(window->get_sdl_handle(), context);
    SDL_GL_SetSwapInterval(1);

    bool err = gl3wInit() != 0;
    if (err)
        throw(fmt::format("Failed to initialize OpenGL loader!\n"));

    //todo: regen when rendering new texture? I think this only works for one
    glGenBuffers(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, Texture::vertices.size(), Texture::vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, EBO);
    glBufferData(GL_ARRAY_BUFFER, Texture::indices.size(), Texture::indices.data(), GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    // texture coord attribute
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    shader = std::make_unique<Shader>(vertex_shader, fragment_shader);
    //todo: move to end() if we have multiple that can switch
    shader->use();
}

Renderer::~Renderer()
{
    //todo: keep track of how many arrays we've allocated and need to be deleted
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
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

void Renderer::render(Texture& texture)
{
    textures.push_back(&texture);
}

void Renderer::end()
{
    if (!first_frame && texture_count_last_frame < textures.size())
    {
        //rebind stuff to grow?
        assert(false);
    }

    for (auto* texture : textures)
    {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture->get_opengl_texture_id());
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, 2, GL_UNSIGNED_INT, nullptr);
    }

    first_frame = false;
    texture_count_last_frame = textures.size();
    textures.clear();
}
