#version 430

in vec4 vColor;

out vec3 color;

void main() {
    color = vColor.xyz;
}
