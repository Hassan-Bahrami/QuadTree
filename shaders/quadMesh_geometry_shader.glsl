#version 330 core

layout(lines_adjacency) in;
layout(triangle_strip, max_vertices = 4) out;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
in vec4 vPosition[4];

out vec3 gNormal;

void emit(int index)
{
    gl_Position = gl_in[index].gl_Position;
    EmitVertex();
}


void main()
{
    emit(0); emit(1); emit(3); emit(2);
    EndPrimitive();
}
