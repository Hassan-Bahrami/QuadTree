#version 330 core

layout(lines_adjacency) in;
layout(line_strip, max_vertices = 5) out;

// uniform bool wireframe;

void emit(int index) {
    gl_Position = gl_in[index].gl_Position;
    EmitVertex(); 
}

void main() {
    // emit(0); emit(1); emit(3); emit(2);
    emit(0); emit(1); emit(2); emit(3); emit(0);
    EndPrimitive();
}