#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <glm/glm.hpp> 
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "Shaders.h"

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 texCoords;
};

struct Face {
    int vertexIndices[3];
};


// Define the base class MeshBase
class MeshBase {
public:
    virtual void Draw(const glm::mat4& model, const glm::mat4& view, const glm::mat4& projection, const glm::vec4& color, int renderMode) = 0;
    virtual ~MeshBase() {} // Virtual destructor for proper cleanup
};

class TriMesh : public MeshBase {
public:
    // Constructor with default shader file paths
    TriMesh(const std::string& filePath): shader("shaders/mesh_vertex_shader.glsl", "shaders/mesh_fragment_shader.glsl") {
        LoadModel(filePath);
    }

    void PrintVerticesAndFaces() const {
        for (const auto& vertex : vertices) {
            std::cout << "Vertex: " << vertex.position.x << ", " << vertex.position.y << ", " << vertex.position.z << std::endl;
        }

        for (const auto& face : faces) {
            std::cout << "Face: " << face.vertexIndices[0] << ", " << face.vertexIndices[1] << ", " << face.vertexIndices[2] << std::endl;
        }
    }

    GLuint GetVAO() const {
        return VAO;
    }

    GLuint GetVBO() const {
        return VBO;
    }
    
void Draw(const glm::mat4& modelMatrix, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix, const glm::vec4& color, int render_mode) override {
    // Activate the shader program
    shader.Use();

    // Set transformation matrices (model, view, projection) as uniforms in the shader
    shader.SetMat4("model", modelMatrix);
    shader.SetMat4("view", viewMatrix);
    shader.SetMat4("projection", projectionMatrix);

    // Define material properties here or use shader uniforms
    glm::vec4 diffuseColor = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);  // Set your desired diffuse color
    glm::vec4 specularColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f); // Set your desired specular color

    // Set shader uniforms for material properties
    shader.SetVec4("diffusecol", diffuseColor);
    shader.SetVec4("specularcol", specularColor);

    shader.SetVec4("InputColor", color); 

    glBindVertexArray(VAO);

    if (render_mode == 0) {
        // Render as Smooth Mesh
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(faces.size() * 3), GL_UNSIGNED_INT, 0);
    } else if (render_mode == 1) {
        // Render as Wireframe
        glLineWidth(1.0f);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(faces.size() * 3), GL_UNSIGNED_INT, 0);
    } else if (render_mode == 2) {
        // Render as Point Cloud
        glPointSize(2.5f);
        glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(faces.size() * 3), GL_UNSIGNED_INT, 0);
    } else if (render_mode == 3) {
        // Render as a combination of all three modes (Smooth Mesh, Wireframe, Point Cloud)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(faces.size() * 3), GL_UNSIGNED_INT, 0);

        glPointSize(2.5f);
        glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(faces.size() * 3), GL_UNSIGNED_INT, 0);

        glLineWidth(2.5f);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(faces.size() * 3), GL_UNSIGNED_INT, 0);
    }

    glBindVertexArray(0);
}


private:
    std::vector<Vertex> vertices;
    std::vector<Face> faces;
    unsigned int VAO, VBO, EBO;
    Shader shader;

    void LoadModel(const std::string& filePath) {
        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFile(filePath, aiProcess_Triangulate | aiProcess_FlipUVs);

        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            std::cerr << "Assimp error: " << importer.GetErrorString() << std::endl;
            return;
        }

        ProcessNode(scene->mRootNode, scene);
    }

    void ProcessNode(aiNode* node, const aiScene* scene) {
        for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            ProcessMesh(mesh, scene);
        }

        for (unsigned int i = 0; i < node->mNumChildren; ++i) {
            ProcessNode(node->mChildren[i], scene);
        }
    }

    void ProcessMesh(aiMesh* mesh, const aiScene* scene) {
        for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
            Vertex vertex;
            vertex.position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
            vertex.normal = glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
            if (mesh->mTextureCoords[0]) {
                vertex.texCoords = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
            } else {
                vertex.texCoords = glm::vec2(0.0f, 0.0f);
            }
            vertices.push_back(vertex);
        }

        for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
            aiFace face = mesh->mFaces[i];
            if (face.mNumIndices != 3) {
                std::cerr << "Only triangle faces are supported." << std::endl;
                continue;
            }
            Face f;
            f.vertexIndices[0] = face.mIndices[0];
            f.vertexIndices[1] = face.mIndices[1];
            f.vertexIndices[2] = face.mIndices[2];
            faces.push_back(f);
        }

        // create vao, vbo, ebo and buffer vertices and faces
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);
        glGenBuffers(1, &VBO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size() * 8, vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)0);
        glEnableVertexAttribArray(0);

        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * faces.size() * 3, faces.data(), GL_STATIC_DRAW);
        glBindVertexArray(0);
    }
};
