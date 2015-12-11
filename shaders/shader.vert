#version 430

uniform mat4 view;
uniform mat4 projection;

in vec3 position;
in vec3 normal;
in vec3 color;

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
