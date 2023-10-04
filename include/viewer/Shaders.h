#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

class Shader {
public:
    // Constructor that takes vertex and fragment shader file paths
    Shader(const char* vertexPath, const char* fragmentPath) {
        // 1. Read shader source code from files
        std::string vertexCode, fragmentCode;
        std::ifstream vertexShaderFile, fragmentShaderFile;

        // Ensure exceptions can be thrown
        vertexShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        fragmentShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

        try {
            // Open the shader files
            vertexShaderFile.open(vertexPath);
            fragmentShaderFile.open(fragmentPath);

            std::stringstream vertexShaderStream, fragmentShaderStream;

            // Read file content into streams
            vertexShaderStream << vertexShaderFile.rdbuf();
            fragmentShaderStream << fragmentShaderFile.rdbuf();

            // Close file handlers
            vertexShaderFile.close();
            fragmentShaderFile.close();

            // Convert streams to strings
            vertexCode = vertexShaderStream.str();
            fragmentCode = fragmentShaderStream.str();

            // std::cout << "vertexCode: " << vertexCode << std::endl;
            // std::cout << "FragmentCode: " << fragmentCode << std::endl;

        }
        catch (const std::ifstream::failure& e) {
            std::cout << vertexPath << std::endl;
            std::cerr << "Shader file read error: " << e.what() << std::endl;
        }

        const char* vertexShaderCode = vertexCode.c_str();
        const char* fragmentShaderCode = fragmentCode.c_str();

        // 2. Compile shaders
        unsigned int vertex, fragment;
        int success;
        char infoLog[512];

        // Vertex Shader
        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vertexShaderCode, nullptr);
        glCompileShader(vertex);

        glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(vertex, 512, nullptr, infoLog);
            std::cerr << "Vertex shader compilation error: " << infoLog << std::endl;
        }

        // Fragment Shader
        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fragmentShaderCode, nullptr);
        glCompileShader(fragment);

        glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(fragment, 512, nullptr, infoLog);
            std::cerr << "Fragment shader compilation error: " << infoLog << std::endl;
        }

        // Shader Program
        ID = glCreateProgram();
        glAttachShader(ID, vertex);
        glAttachShader(ID, fragment);
        glLinkProgram(ID);

        glGetProgramiv(ID, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(ID, 512, nullptr, infoLog);
            std::cerr << "Shader program linking error: " << infoLog << std::endl;
        }

        // Delete shaders as they are linked into the program and no longer needed
        glDeleteShader(vertex);
        glDeleteShader(fragment);
    }

    Shader(const char* vertexPath, const char* fragmentPath, const char* geometryPath) {
        // 1. Read shader source code from files
        std::string vertexCode, fragmentCode, geometryCode, WFGeometryCode;
        std::ifstream vertexShaderFile, fragmentShaderFile, geometryShaderFile, WFGeometryShaderFile;

        // Ensure exceptions can be thrown
        vertexShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        fragmentShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        geometryShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

        try {
            // Open the shader files
            vertexShaderFile.open(vertexPath);
            fragmentShaderFile.open(fragmentPath);
            geometryShaderFile.open(geometryPath);

            std::stringstream vertexShaderStream, fragmentShaderStream, geometryShaderStream, WFGeometryShaderStream;

            // Read file content into streams
            vertexShaderStream << vertexShaderFile.rdbuf();
            fragmentShaderStream << fragmentShaderFile.rdbuf();
            geometryShaderStream << geometryShaderFile.rdbuf();

            // Close file handlers
            vertexShaderFile.close();
            fragmentShaderFile.close();
            geometryShaderFile.close();

            // Convert streams to strings
            vertexCode = vertexShaderStream.str();
            fragmentCode = fragmentShaderStream.str();
            geometryCode = geometryShaderStream.str();

        }
        catch (const std::ifstream::failure& e) {
            std::cerr << "Shader file read error: " << e.what() << std::endl;
        }

        const char* vertexShaderCode = vertexCode.c_str();
        const char* fragmentShaderCode = fragmentCode.c_str();
        const char* geometryShaderCode = geometryCode.c_str();

        // 2. Compile shaders
        unsigned int vertex, fragment, geometry;
        int success;
        char infoLog[512];

        // Vertex Shader
        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vertexShaderCode, nullptr);
        glCompileShader(vertex);

        glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(vertex, 512, nullptr, infoLog);
            std::cerr << "Vertex shader compilation error: " << infoLog << std::endl;
        }

        // Fragment Shader
        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fragmentShaderCode, nullptr);
        glCompileShader(fragment);

        glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(fragment, 512, nullptr, infoLog);
            std::cerr << "Fragment shader compilation error: " << infoLog << std::endl;
        }
        // Geometry Shader
        geometry = glCreateShader(GL_GEOMETRY_SHADER);
        glShaderSource(geometry, 1, &geometryShaderCode, nullptr);
        glCompileShader(geometry);

        glGetShaderiv(geometry, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(geometry, 512, nullptr, infoLog);
            std::cerr << "Geometry shader compilation error: " << infoLog << std::endl;
        }


        // Shader Program
        ID = glCreateProgram();
        glAttachShader(ID, vertex);
        glAttachShader(ID, fragment);
        glAttachShader(ID, geometry);
        glLinkProgram(ID);

        glGetProgramiv(ID, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(ID, 512, nullptr, infoLog);
            std::cerr << "Shader program linking error: " << infoLog << std::endl;
        }
        // Delete shaders as they are linked into the program and no longer needed
        glDeleteShader(vertex);
        glDeleteShader(fragment);
        glDeleteShader(geometry);
    }

    // Use the shader program
    void Use() {
        glUseProgram(ID);
    }

    // Set a 4x4 matrix as a uniform in the shader
    void SetMat4(const std::string& name, const glm::mat4& matrix) {
        glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(matrix));
    }

    // Function to set a glm::vec4 uniform
    void SetVec4(const std::string& name, const glm::vec4& value) const {
        glUniform4fv(glGetUniformLocation(ID, name.c_str()), 1, glm::value_ptr(value));
    }

    void SetVec3(const std::string& name, const glm::vec3& value) const {
        glUniform4fv(glGetUniformLocation(ID, name.c_str()), 1, glm::value_ptr(value));
    }
    // Set uniform variables
    void SetBool(const std::string& name, bool value) const {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), static_cast<int>(value));
    }

    void SetInt(const std::string& name, int value) const {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
    }

    void SetFloat(const std::string& name, float value) const {
        glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
    }

    void SetLineColor(const glm::vec4& color) const{
        glUniform4fv(glGetUniformLocation(ID, "lineColor"), 1, &color[0]);
    }

    // Function to get the location of the "position" attribute
    GLint GetPositionAttribLocation() const{
        return glGetAttribLocation(ID, "position");
    }

    GLuint GetID() const {
        return ID;
    }
private:
    unsigned int ID;
};
