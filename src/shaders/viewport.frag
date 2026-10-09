#version 450

layout(location = 0) in vec3 color;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec4 lightPosition;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D shadowDepth;

float filteredShadow(vec3 normalDirection) {
    vec3 projected = lightPosition.xyz / lightPosition.w;
    vec2 uv = projected.xy * 0.5 + 0.5;
    if (projected.z < 0.0 || projected.z > 1.0 || any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0))))
        return 1.0;
    float bias = max(0.0007 * (1.0 - abs(dot(normalDirection, normalize(vec3(0.45, 0.82, 0.35))))), 0.00012);
    vec2 texel = 1.0 / vec2(textureSize(shadowDepth, 0));
    float visibility = 0.0;
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            float storedDepth = texture(shadowDepth, uv + vec2(x, y) * texel).r;
            visibility += projected.z - bias <= storedDepth ? 1.0 : 0.0;
        }
    }
    return visibility / 9.0;
}

void main() {
    vec3 normalDirection = normalize(normal);
    vec3 lightDirection = normalize(vec3(0.45, 0.82, 0.35));
    float diffuse = abs(dot(normalDirection, lightDirection));
    float visibility = filteredShadow(normalDirection);
    outColor = vec4(color * (0.28 + 0.72 * diffuse) * (0.34 + 0.66 * visibility), 1.0);
}