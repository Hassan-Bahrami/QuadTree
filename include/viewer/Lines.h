#include <GL/glew.h>
#include <vector>
#include "Shaders.h" 

class Lines {
public:
    Lines(const std::vector<glm::vec3>& vertices);
    ~Lines();

    void Draw(const glm::mat4& modelMatrix, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix, const glm::vec4& lineColor);

private:
    GLuint VAO, VBO;
    Shader shader;
    std::vector<glm::vec3> lineVertices;

    void SetupBuffers();
};

Lines::Lines(const std::vector<glm::vec3>& vertices) : lineVertices(vertices), shader("shaders/line_vertex_shader.glsl", "shaders/line_fragment_shader.glsl") {
    SetupBuffers();
}

Lines::~Lines() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
}

void Lines::SetupBuffers() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, lineVertices.size() * sizeof(glm::vec3), lineVertices.data(), GL_STATIC_DRAW);

    // Define vertex attribute pointers
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glBindVertexArray(0);
}

void Lines::Draw(const glm::mat4& modelMatrix, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix, const glm::vec4& lineColor = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)) {
    shader.Use();
    shader.SetMat4("model", modelMatrix);
    shader.SetMat4("view", viewMatrix);
    shader.SetMat4("projection", projectionMatrix);
    shader.SetVec4("lineColor", lineColor); // Set the line color using the provided or default color

    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, lineVertices.size());
    glBindVertexArray(0);
}

