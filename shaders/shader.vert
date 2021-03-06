#version 430

layout(location=1) uniform mat4 view;
layout(location=2) uniform mat4 projection;

layout(location=1) in vec3 color;
layout(location=2) in vec3 position;
layout(location=3) in vec3 normal;

out vec3 vPosition;
out vec3 vColor;
out vec3 vNormal;

void main() {
  vPosition = position;
  vColor = color;
  vNormal = normal;

  mat4 mvp = projection * view;

  gl_Position = mvp * vec4(position, 1);
}
