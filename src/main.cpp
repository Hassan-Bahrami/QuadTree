#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <filesystem>
#include "viewer/Mesh.h"
#include "viewer/QuadMesh.h"
#include "viewer/Lines.h"
#include "viewer/Shaders.h"
#include "viewer/Camera.h"

#include "MotorCycleGraph/MotorCycle.h"

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <ImFileBrowser.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// Callback function to receive OpenGL debug messages
void GLAPIENTRY OpenGLDebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam) {
// Output or log the debug message
std::cout << "OpenGL Debug Message: " << message << std::endl;
}

// Window Dimensions
float WinHeight = 600;
float WinWidth = 800;

// Create an instance of the OrbitCamera
OrbitCamera orbitCamera(20.0f); // Set an initial radius

// Define camera parameters
glm::vec3 cameraPosition = glm::vec3(0.0f, 0.0f, 50.0f); // Initial camera position
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);   // Initial camera front direction
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);       // Up vector
float aspectRatio = static_cast<float>(WinWidth) / WinHeight; // Adjust with your window size

// Create an FPS camera
FPSCamera fpsCamera(cameraPosition, cameraFront, cameraUp, aspectRatio);

// Function to handle window resizing
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

// Function to handle user input (e.g., escape key to close the window)
void processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

double prevX = 0.0;
double prevY = 0.0;
bool isRotating = false;
// Callback function for mouse movement
void orbitCamera_cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    if (isRotating) {
        // If left mouse button is pressed, update camera rotation
        double dx = xpos - prevX;
        double dy = ypos - prevY;
        orbitCamera.rotate(dx, dy);
        prevX = xpos;
        prevY = ypos;
    }
}


// Mouse button callback
void orbitCamera_mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            // Start rotating when left mouse button is pressed
            isRotating = true;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); // Hide cursor while dragging
            glfwGetCursorPos(window, &prevX, &prevY); // Initialize prevX and prevY
        } else if (action == GLFW_RELEASE) {
            isRotating = false; // Stop rotating when left mouse button is released
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL); // Show cursor when not dragging
        }
    }
}



// Mouse scroll callback
void orbitCamera_scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    // Adjust the camera's radius based on the mouse wheel input (yoffset)
    orbitCamera.zoom(yoffset);
}

double lastX = WinWidth / 2.0;
double lastY = WinHeight / 2.0;
// Callback function for mouse movement
void FPSCamera_mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    // Calculate the delta between the current and last mouse positions
    double xoffset = xpos - lastX;
    double yoffset = lastY - ypos;  // Reversed since y-coordinates are inverted

    // Update the last mouse position
    lastX = xpos;
    lastY = ypos;

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        fpsCamera.mouse_control(xoffset, yoffset);
    }
}

void FPSCamera_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    static double lastTime = glfwGetTime();
    double currentTime = glfwGetTime();
    float deltaTime = static_cast<float>(currentTime - lastTime);
    lastTime = currentTime;

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) {
        case GLFW_KEY_W:
            fpsCamera.move(FPSCamera::FORWARD, deltaTime);
            break;
        case GLFW_KEY_S:
            fpsCamera.move(FPSCamera::BACKWARD, deltaTime);
            break;
        case GLFW_KEY_A:
            fpsCamera.move(FPSCamera::LEFT, deltaTime);
            break;
        case GLFW_KEY_D:
            fpsCamera.move(FPSCamera::RIGHT, deltaTime);
            break;
        case GLFW_KEY_SPACE:
            fpsCamera.move(FPSCamera::UP, deltaTime);
            break;
        case GLFW_KEY_LEFT_SHIFT:
            fpsCamera.move(FPSCamera::DOWN, deltaTime);
            break;
        }
    }
}


// Callback function for mouse scroll
void FPSCamera_scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    fpsCamera.scroll_control(yoffset);
}

// Imgui Implementation
// ImGui variables
bool showFileBrowser = false;
std::string selectedObjFilePath;

void ImGuiInit(GLFWwindow* window) {
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    // Initialize ImGui
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

}


void ImGuiShutdown() {
    // Cleanup ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void RenderMesh(MeshBase& mesh, const glm::vec4& color, int renderMode, Camera camera) {
    // Define your model matrix (e.g., translation, rotation, scaling)
    glm::mat4 model = glm::mat4(1.0f); // Initialize as an identity matrix (no transformation)

    // Get the view matrix from the camera
    glm::mat4 viewMatrix = camera.view();

    // Get the projection matrix from the camera
    glm::mat4 ProjectionMatrix = camera.projection();

    // // Convert ImGui color to glm color
    // glm::vec4 glmColor(color.x, color.y, color.z, color.w);
    mesh.Draw(model, viewMatrix, ProjectionMatrix, color, renderMode);
}

ImGui::FileBrowser fileDialog(ImGuiFileBrowserFlags_CreateNewDir);
bool meshLoaded = false; // Flag to track whether the mesh is loaded
bool useQuadMesh = false; // Flag to determine which type of mesh to use
bool useTriangularMesh = false; // Flag to determine which type of mesh to use
QuadMesh *quadMesh = nullptr;
TriMesh* triangularMesh = nullptr;
// FPSCamera* fpsCamera = nullptr;
// OrbitCamera* orbitCamera = nullptr;
Camera* currentCamera = &orbitCamera;
glm::vec4 meshColor = glm::vec4(0.5f, 0.5f, 0.5f, 1.0f); // Light gray color
int renderMode = 3; // Default to Mesh
bool useFPSCamera =false; // Flag to use FPS camera
bool useOrbitCamera = false; // Flag to use Orbit camera

void UI_Content() {
    const char* renderModes[] = { "Smooth", "Wire Frame", "Point Cloud", "Mesh" };

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Open", NULL)) {
                ImGui::OpenPopup("Open File");
                fileDialog.SetTitle("Open File");
                fileDialog.SetTypeFilters({ ".obj", ".ply", ".off" });
                fileDialog.Open();
            }
            if (ImGui::MenuItem("Quit", NULL)) {
                exit(0);
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }

    // ImGui window for settings
    ImGui::Begin("Settings");

    // Mesh type section
    ImGui::Text("Mesh type");
    ImGui::Checkbox("Quad", &useQuadMesh);
    ImGui::Checkbox("Triangular", &useTriangularMesh);
    // Inside your ImGui "Settings" window
    ImGui::ColorPicker4("Mesh Color", (float*)&meshColor, ImGuiColorEditFlags_Float);

    // Render mode checkboxes
    ImGui::Text("Render mode");
    bool renderModeFlags[4] = { (renderMode == 0), (renderMode == 1), (renderMode == 2), (renderMode == 3) };
    for (int i = 0; i < 4; i++) {
        if (ImGui::Checkbox(renderModes[i], &renderModeFlags[i])) {
            renderMode = i;
        }
    }

    // // Camera type
    // ImGui::Text("Camera");
    // ImGui::Checkbox("FPS camera", &useFPSCamera);
    // ImGui::Checkbox("Orbit camera", &useOrbitCamera);

    // End settings window
    ImGui::End();

    // if (useFPSCamera) {
    //     currentCamera = &fpsCamera;
    // } else if (useOrbitCamera){
    //     currentCamera = &orbitCamera;
    // }

    // Show the file dialog
    fileDialog.Display();
    if (fileDialog.HasSelected()) {
        selectedObjFilePath = fileDialog.GetSelected();
        meshLoaded = true;
        fileDialog.Close();
    }
    if (meshLoaded){
        if (useQuadMesh) {
            quadMesh = new QuadMesh(selectedObjFilePath);
            RenderMesh(*quadMesh, meshColor, renderMode, *currentCamera);
        }
        if (useTriangularMesh) {
            triangularMesh = new TriMesh(selectedObjFilePath);
            RenderMesh(*triangularMesh, meshColor, renderMode, *currentCamera);
        }   
    } 
}

void ImGui_cleanup(){
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}


void RenderImGui() { // Pass fileDialog as an argument
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    UI_Content();
    // Rendering ImGui
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void GL_init(){
            // Clear the screen
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}


int main() {
        
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Set GLFW to use OpenGL 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

    // Create a GLFW window
    GLFWwindow* window = glfwCreateWindow(WinWidth, WinHeight, "Geo Viewer", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Make the window's context current
    glfwMakeContextCurrent(window);

    // Set the callback function for window resizing
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Register mouse input callbacks
    glfwSetMouseButtonCallback(window, orbitCamera_mouse_button_callback);
    glfwSetCursorPosCallback(window, orbitCamera_cursor_position_callback);
    glfwSetScrollCallback(window, orbitCamera_scroll_callback);
    // glfwSetKeyCallback(window, FPSCamera_key_callback);

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    // Initialize ImGui
    ImGuiInit(window);

    // Example vertices and line indices
    std::vector<glm::vec3> vertices = {
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(1.0f, 1.0f, 1.0f),
        glm::vec3(2.0f, 2.0f, 2.0f)
    };

    std::vector<unsigned int> lineIndices = { 0, 1, 1, 2 };

    // Create an instance of the Lines class with the vertices
    Lines lines(vertices);

    // QuadMesh mesh1("../models/100478_rem_p0_0_quadrangulation_smooth.obj")
    // MotorcycleGraph(mesh1, )

    QuadMesh mesh1("../models/MyMesh.obj");
    // Create an instance of MotorcycleGraph
    MotorcycleGraph motorcycleGraph(mesh1);

    // Print the motorcycle edges
    const std::vector<int>& extraordinaryVertices = motorcycleGraph.getEOV();
    std::cout << "extraordinaryVertices:" << extraordinaryVertices.size() << std::endl;
    // for (const int& vertexInx : extraordinaryVertices) {
    //     std::cout << vertexInx << std::endl;
    // }

    // Main rendering loop
    while (!glfwWindowShouldClose(window)) {
        // Process user input and update camera
        processInput(window);

        GL_init();

        // // Enable OpenGL debug output
        // glEnable(GL_DEBUG_OUTPUT);
        // glDebugMessageCallback(OpenGLDebugCallback, nullptr);

        // Render ImGui elements
        RenderImGui();

        // // Initialize as an identity matrix (no transformation)
        // glm::mat4 model = glm::mat4(1.0f);

        // // Get the view matrix from the camera
        // glm::mat4 viewMatrix = camera.view();

        // // Get the projection matrix from the camera
        // glm::mat4 ProjectionMatrix = camera.projection();

        // // Convert ImGui color to glm color
        // glm::vec4 glmColor(color.x, color.y, color.z, color.w);
        // std::cout << "I reached before drawing!" << std::endl;
        // mesh1.Draw(model, viewMatrix, ProjectionMatrix, glm::vec4 (0.0, 0.0, 1.0, 1.0), 0);

        // Render the lines
        // lines.Draw(model, viewMatrix, camera.projection());


        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup and exit
    ImGui_cleanup();
    glfwTerminate();

    return 0;

}
