#version 430

uniform vec3 sunPosition = vec3(150.0, 100.0, -100.0);
uniform vec4 sunColor = vec4(1.0, 1.0, 1.0, 0.9); // a component is for intensity

in vec3 vColor;
in vec3 vNormal;
out vec3 vPosition;

out vec3 color;

void main() {
    vec3 sunNormal = normalize(sunPosition - vPosition);
    float theta = max(0.2,dot(vNormal, sunNormal));
    color = vColor * sunColor.rgb * sunColor.a * theta;
}
