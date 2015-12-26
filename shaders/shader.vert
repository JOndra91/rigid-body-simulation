#version 430

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform vec3 color;

in vec3 normal;

out vec3 vPosition;
out vec3 vColor;
out vec3 vNormal;

void main() {
  vPosition = (model * vec4(position, 1)).xyz;
  vColor = color;
  vNormal = normal;

  mat4 mvp = projection * view * model;

  gl_Position = mvp * vec4(position, 1);
}
