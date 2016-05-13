#version 430

layout(location=1) uniform mat4 view;
layout(location=2) uniform mat4 projection;

layout(location=1) in vec4 color;
layout(location=2) in vec3 position;

out vec4 vColor;

void main() {
  vColor = color;

  mat4 mvp = projection * view;

  gl_Position = mvp * vec4(position, 1);
}
