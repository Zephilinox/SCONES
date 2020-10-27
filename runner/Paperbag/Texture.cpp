#include "Texture.hpp"

//SELF
#include "Renderer.hpp"

using namespace paperbag;

Texture::Texture()
{
    glGenTextures(1, &opengl_texture_id);
}

void Texture::load(std::vector<Pixel> p, unsigned int w, unsigned int h)
{
    pixels = std::move(p);
    pixels.resize(w * h);
    width = w;
    height = h;

    glBindTexture(GL_TEXTURE, opengl_texture_id);
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); // set texture wrapping to GL_REPEAT (default wrapping method)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    glGenerateMipmap(GL_TEXTURE_2D);
}

const Pixel& Texture::operator[](unsigned int index) const
{
    return pixels[index];
}

void Texture::update(std::function<void(std::vector<Pixel>& pixels)> updater)
{
    updater(pixels);

    glBindTexture(GL_TEXTURE, opengl_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    glGenerateMipmap(GL_TEXTURE_2D);
}

unsigned int Texture::get_opengl_texture_id() const
{
    return opengl_texture_id;
}

const std::vector<Pixel>& Texture::get_pixels() const
{
    return pixels;
}