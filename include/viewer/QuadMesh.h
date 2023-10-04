#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <glm/glm.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "Shaders.h"
#include "Mesh.h"

struct Quad {
    int vertexIndices[4];
};

class QuadMesh : public MeshBase {
public:
    QuadMesh(const std::string& filePath)
        : shader1("shaders/quadMesh_vertex_shader.glsl", "shaders/quadMesh_fragment_shader.glsl", "shaders/quadMesh_geometry_shader.glsl"),
          shader2("shaders/quadMesh_vertex_shader.glsl", "shaders/quadMesh_fragment_shader.glsl", "shaders/quadMesh_wf_geometry_shader.glsl") {
        LoadModel(filePath);  
    }

    void PrintVerticesAndQuads() const {
        for (const auto& vertex : vertices) {
            std::cout << "Vertex: " << vertex.position.x << ", " << vertex.position.y << ", " << vertex.position.z << std::endl;
        }

        for (const auto& quad : quads) {
            std::cout << "Quad: " << quad.vertexIndices[0] << ", " << quad.vertexIndices[1] << ", "
                      << quad.vertexIndices[2] << ", " << quad.vertexIndices[3] << std::endl;
        }
    }

    GLuint GetVAO() const {
        return VAO;
    }

    GLuint GetVBO() const {
        return VBO;
    }

    std::vector<Vertex> GetVertices() const {
        return vertices;

    }

    std::vector<Quad> GetQuads() const {
        return quads;
    }

    void Draw(const glm::mat4& modelMatrix, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix, const glm::vec4& color, int render_mode) override {

        if (render_mode == 0) {
            // Render as Smooth Mesh
            shader1.Use();
            shader1.SetMat4("model", modelMatrix);
            shader1.SetMat4("view", viewMatrix);
            shader1.SetMat4("projection", projectionMatrix);
            shader1.SetVec4("inputColor", color);         
            glBindVertexArray(VAO);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDrawElements(GL_LINES_ADJACENCY, static_cast<GLsizei>(quads.size() * 4), GL_UNSIGNED_INT, 0);
        } else if (render_mode == 1) {
            // Render as Wireframe
            shader2.Use();
            shader2.SetMat4("model", modelMatrix);
            shader2.SetMat4("view", viewMatrix);
            shader2.SetMat4("projection", projectionMatrix);
            shader2.SetVec4("inputColor", color);
            glBindVertexArray(VAO);
            glLineWidth(1.0f);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDrawElements(GL_LINES_ADJACENCY, static_cast<GLsizei>(quads.size() * 4), GL_UNSIGNED_INT, 0);
        } else if (render_mode == 2) {
            // Render as Point Cloud
            shader1.Use();
            shader1.SetMat4("model", modelMatrix);
            shader1.SetMat4("view", viewMatrix);
            shader1.SetMat4("projection", projectionMatrix);
            shader1.SetVec4("inputColor", color);
            glBindVertexArray(VAO);
            glPointSize(2.5f);
            glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
            glDrawElements(GL_LINES_ADJACENCY, static_cast<GLsizei>(quads.size() * 4), GL_UNSIGNED_INT, 0);
        } else if (render_mode == 3) {
            // Render as a combination of all three modes (Smooth Mesh, Wireframe, Point Cloud)
            shader1.Use();
            shader1.SetMat4("model", modelMatrix);
            shader1.SetMat4("view", viewMatrix);
            shader1.SetMat4("projection", projectionMatrix);
            shader1.SetVec4("inputColor", color);
            glBindVertexArray(VAO);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDrawElements(GL_LINES_ADJACENCY, static_cast<GLsizei>(quads.size() * 4), GL_UNSIGNED_INT, 0);

            glPointSize(2.5f);
            glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
            glDrawElements(GL_LINES_ADJACENCY, static_cast<GLsizei>(quads.size() * 4), GL_UNSIGNED_INT, 0);

            shader2.Use();
            shader2.SetMat4("model", modelMatrix);
            shader2.SetMat4("view", viewMatrix);
            shader2.SetMat4("projection", projectionMatrix);
            shader2.SetVec4("inputColor", glm::vec4 (0.0, 0.0, 0.0, 1.0));
            glBindVertexArray(VAO);
            glLineWidth(2.5f);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDrawElements(GL_LINES_ADJACENCY, static_cast<GLsizei>(quads.size() * 4), GL_UNSIGNED_INT, 0);
        }

        glBindVertexArray(0);
    }

private:
    std::vector<Vertex> vertices;
    std::vector<Quad> quads;
    unsigned int VAO, VBO, EBO;
    Shader shader1;
    Shader shader2;

    void LoadModel(const std::string& filePath) {
        Assimp::Importer importer;
        // Add the aiProcess_Triangulate flag with the value false to prevent triangulation
        const aiScene* scene = importer.ReadFile(filePath, aiProcess_FlipUVs | aiProcess_JoinIdenticalVertices);

        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            std::cerr << "Assimp error: " << importer.GetErrorString() << std::endl;
            return;
        }

        // Initialize the vertexOffset to 0
        unsigned int vertexOffset = 0;

        // Process the nodes and meshes
        ProcessNode(scene->mRootNode, scene, vertexOffset);
        // Create VAO, VBO, EBO, and buffer vertices and quads
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);
        glGenBuffers(1, &VBO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size() * 8, vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * quads.size() * 4, quads.data(), GL_STATIC_DRAW);
        glBindVertexArray(0);
    }


    void ProcessNode(aiNode* node, const aiScene* scene, unsigned int& vertexOffset) {
        for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            ProcessMesh(mesh, scene, vertexOffset);
        }

        for (unsigned int i = 0; i < node->mNumChildren; ++i) {
            ProcessNode(node->mChildren[i], scene, vertexOffset);
        }
    }

    void ProcessMesh(aiMesh* mesh, const aiScene* scene, unsigned int& vertexOffset) {
        // Store the offset for this mesh
        unsigned int currentOffset = vertices.size();

        for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
            Vertex vertex;
            vertex.position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);

            if (mesh->mNormals) {
                vertex.normal = glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
            } else {
                vertex.normal = glm::vec3(0.0f, 0.0f, 0.0f);
            }

            if (mesh->mTextureCoords[0]) {
                vertex.texCoords = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
            } else {
                vertex.texCoords = glm::vec2(0.0f, 0.0f);
            }
            
            // Add the adjusted vertex to the vertices vector
            vertices.push_back(vertex);
        }

        for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
            aiFace face = mesh->mFaces[i];
            if (face.mNumIndices == 4) {
                Quad q;
                for (unsigned int j = 0; j < 4; ++j) {
                    // Adjust indices using the current offset
                    q.vertexIndices[j] = currentOffset + face.mIndices[j];
                }
                quads.push_back(q);
            } else {
                // Handle other face types if needed.
                std::cerr << "Warning: Encountered a face with " << face.mNumIndices << " indices, which is not supported. Ignoring this face." << std::endl;
            }
        }
    }

};


