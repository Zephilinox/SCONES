#pragma once

#include "Uniform.hpp"

#include <cstdint>
#include <string>
#include <memory>
#include <unordered_map>

namespace paperbag
{
    enum class ShaderType
    {
        VERTEX,
        FRAGMENT
    };
    
    class ShaderData;
    
    class Shader
    {
    public:
        Shader(const std::string& id);
        virtual ~Shader();
        
        //Static stuff
        static Shader getShader(const std::string* id);
        static void cleanUpShaders();
        
        //Dirty flag things
        void dirty() { m_dirty = true; }
        void clearDirty() { m_dirty = false; }
        bool isDirty() const { return m_dirty; }
        
        const std::string& getId() const;

    protected:
        std::unordered_map<std::string, std::uint32_t> m_attributes;
        std::unordered_map<std::string, std::pair<UniformType, std::uint32_t>> m_uniforms;
        
    private:
        void loadAttributesAndUniforms();
        
        std::string m_id;
        std::unique_ptr<ShaderData> m_shaderData = nullptr;
        bool m_dirty = true;
        
        static std::unordered_map<std::string, Shader*> sm_loadedShaders;
    };
    
    class ShaderData
    {
    public:
        ShaderData(Shader* shader);
        virtual ~ShaderData();
        
        virtual void cleanUp() {};
        
    protected:
        template <typename T>
        T* getShader() const { return reinterpret_cast<T*>(m_shader); }
        
        const std::unordered_map<std::string, std::uint32_t>& getAttributes() const;
        
        void setAttributIndex(const std::string& id, std::uint32_t index);
        void setUniformTypeAndLocation(const std::string& id, UniformType type, uint32_t location);
    private:
        Shader* m_shader;
    };
}