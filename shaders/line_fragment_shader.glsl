#version 330 core
out vec4 FragColor;

uniform vec4 lineColor; // Customize the line color here

void main() {
    FragColor = lineColor;
}
