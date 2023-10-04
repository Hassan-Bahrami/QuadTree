cmake_minimum_required(VERSION 3.3)
project(SurfDev)

# Set the C++ standard to C++17 or later
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})


option(LIBIGL_GLFW                "Build target igl::glfw"                ON)
# option(LIBIGL_IMGUI               "Build target igl::imgui"               ON)
option(LIBIGL_OPENGL              "Build target igl::opengl"              ON)


# Libigl
include(libigl)

# Finding Gurobi
# find_package(GUROBI REQUIRED)

# Finding Assimp
find_package(assimp REQUIRED)

# # Enable the target igl::glfw
# igl_include(glfw)
igl_include(imgui)
# igl_include(embree)


set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/extern/imgui)

# Add your project files
file(GLOB SRC_FILES src/main_new.cpp src/QuadTree/*.cpp)
#src/viewer/Camera.cpp src/MotorCycleGraph/Particle.cpp src/MotorCycleGraph/MotorCycle.cpp ${IMGUI_DIR}/*.cpp
file(GLOB PROJECT_SHADERS shaders/*.glsl)
add_executable(${PROJECT_NAME} ${SRC_FILES})


# linking libraries
target_link_libraries(${PROJECT_NAME} PUBLIC igl::glfw igl::imgui igl::opengl ${GUROBI_LIBRARY} /opt/gurobi1002/linux64/lib/libgurobi_c++.a ${ASSIMP_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} PUBLIC igl::glfw igl::imgui igl::opengl ${ASSIMP_LIBRARIES})

# Manually link GLEW and OpenGL
set(GLEW_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/extern/glew-2.1.0/lib)
set(GLEW_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/extern/glew-2.1.0/include)
set(GLM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/glm)
set(FREEGLUT_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/extern/freeglut/lib/libglut.a)
set(FREEGLUT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/freeglut/include)
set(IMGUIFILEDIALOG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/ImGuiFileDialog)
set(IMGUI_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/imgui/)


target_link_libraries(${PROJECT_NAME} PUBLIC ${GLEW_LIBRARY_DIR}/libGLEW.a GL ${FREEGLUT_LIBRARY_DIR})

# include directories
include_directories(${PROJECT_NAME} PRIVATE ./include ./extern/GeoLibrary ${GLEW_INCLUDE_DIR} ${GLM_INCLUDE_DIR} ${FREEGLUT_INCLUDE_DIR})
# ${IMGUI_INCLUDE_DIR}

# Copy the shaders folder into the build directory
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/shaders" DESTINATION "${CMAKE_BINARY_DIR}")

# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/extern/ImGuiFileDialog/)
