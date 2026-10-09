#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;
layout(location = 2) in vec3 inNormal;

layout(push_constant) uniform CameraData {
    mat4 viewProjection;
    mat4 lightViewProjection;
} camera;

layout(location = 0) out vec3 color;
layout(location = 1) out vec3 normal;
layout(location = 2) out vec4 lightPosition;

void main() {
    vec4 worldPosition = vec4(inPosition, 1.0);
    gl_Position = camera.viewProjection * worldPosition;
    color = inColor;
    normal = inNormal;
    lightPosition = camera.lightViewProjection * worldPosition;
}