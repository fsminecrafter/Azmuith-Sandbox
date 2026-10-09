#version 450

layout(location = 0) in vec3 inPosition;

layout(push_constant) uniform ShadowCamera {
    mat4 lightViewProjection;
} shadowCamera;

void main() {
    gl_Position = shadowCamera.lightViewProjection * vec4(inPosition, 1.0);
}