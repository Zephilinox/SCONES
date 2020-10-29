#pragma once

//LIBS
#include <GL/gl.h>
#include <glad/glad.h>
#include <spdlog/spdlog.h>

namespace paperbag
{
    inline void magic_function()
    {
        auto err = glGetError();
        if (err != GL_NO_ERROR)
        {
            spdlog::info("OpenGL error - {:x}", err);
            int totes_important = 0;
        }
    }

}