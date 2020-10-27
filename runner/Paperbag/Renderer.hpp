#pragma once

//STD
#include <iostream>

//LIBS

//SELF
#include "Window.hpp"
#include "Pixel.hpp"
#include "Texture.hpp"

namespace paperbag
{

class Texture;

//todo: base class and shit
class Renderer
{
public:
    Renderer(Window* window);
    ~Renderer();

    SDL_GLContext& get_context();

    std::unique_ptr<Texture> make_texture(std::vector<Pixel> p, unsigned int w, unsigned int h) const;
    void render(Texture& texture);
    void end();

private:
    Window* window = nullptr;
    SDL_GLContext context = nullptr;

    unsigned int VBO;
    unsigned int VAO;
    unsigned int EBO;

    bool first_frame = true;
    unsigned int texture_count_last_frame = 0;
    std::vector<Texture*> textures;

    //https://learnopengl.com/code_viewer_gh.php?code=includes/learnopengl/shader.h
    class Shader
    {
    public:
        Shader(const char* vertex_shader, const char* fragment_shader)
        {
            // compile shaders
            unsigned int vertex;
            unsigned int fragment;
            // vertex shader
            vertex = glCreateShader(GL_VERTEX_SHADER);
            glShaderSource(vertex, 1, &vertex_shader, NULL);
            glCompileShader(vertex);
            check_compile_errors(vertex, "VERTEX");
            // fragment Shader
            fragment = glCreateShader(GL_FRAGMENT_SHADER);
            glShaderSource(fragment, 1, &fragment_shader, NULL);
            glCompileShader(fragment);
            check_compile_errors(fragment, "FRAGMENT");
            // shader Program
            ID = glCreateProgram();
            glAttachShader(ID, vertex);
            glAttachShader(ID, fragment);
            glLinkProgram(ID);
            check_compile_errors(ID, "PROGRAM");
            // delete the shaders as they're linked into our program now and no longer necessery
            glDeleteShader(vertex);
            glDeleteShader(fragment);
        }

        void use()
        {
            glUseProgram(ID);
        }

        unsigned int ID;

    private:
        void check_compile_errors(GLuint shader, std::string type)
        {
            GLint success;
            GLchar infoLog[1024];
            if (type != "PROGRAM")
            {
                glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
                if (!success)
                {
                    glGetShaderInfoLog(shader, 1024, NULL, infoLog);
                    std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n"
                              << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
                }
            }
            else
            {
                glGetProgramiv(shader, GL_LINK_STATUS, &success);
                if (!success)
                {
                    glGetProgramInfoLog(shader, 1024, NULL, infoLog);
                    std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n"
                              << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
                }
            }
        }
    };

    std::unique_ptr<Shader> shader;
};

}