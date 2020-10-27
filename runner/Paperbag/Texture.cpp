#include "Texture.hpp"

//SELF
#include "Renderer.hpp"

using namespace paperbag;

Texture::Texture()
{
    magic_function();
    glGenTextures(1, &opengl_texture_id);
    magic_function();
}

void Texture::load(std::vector<Pixel> p, unsigned int w, unsigned int h)
{
    pixels = std::move(p);
    pixels.resize(w * h);
    width = w;
    height = h;

    magic_function();
    glBindTexture(GL_TEXTURE_2D, opengl_texture_id);
    magic_function();
    // set the texture wrapping parameters
    magic_function();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT); // set texture wrapping to GL_REPEAT (default wrapping method)
    magic_function();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    magic_function();
    // set texture filtering parameters
    magic_function();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    magic_function();
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    magic_function();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    magic_function();
    glGenerateMipmap(GL_TEXTURE_2D);
    magic_function();
}

const Pixel& Texture::operator[](unsigned int index) const
{
    return pixels[index];
}

void Texture::update(std::function<void(std::vector<Pixel>& pixels)> updater)
{
    updater(pixels);

    magic_function();
    glBindTexture(GL_TEXTURE, opengl_texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    glGenerateMipmap(GL_TEXTURE_2D);
    magic_function();
}

unsigned int Texture::get_opengl_texture_id() const
{
    return opengl_texture_id;
}

unsigned int Texture::get_width() const
{
    return width;
}

unsigned int Texture::get_height() const
{
    return height;
}

const std::vector<Pixel>& Texture::get_pixels() const
{
    return pixels;
}