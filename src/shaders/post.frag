#version 450

layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D sceneColor;
layout(set = 0, binding = 1) uniform sampler2D sceneDepth;

layout(push_constant) uniform Effects {
    vec4 switchesAndQuality;
    vec2 inverseExtent;
} effects;

float luma(vec3 value) {
    return dot(value, vec3(0.299, 0.587, 0.114));
}

void main() {
    vec3 result = texture(sceneColor, uv).rgb;
    if (effects.switchesAndQuality.x > 0.5) {
        float centerDepth = texture(sceneDepth, uv).r;
        float occlusion = 0.0;
        const vec2 directions[8] = vec2[8](
            vec2(1.0, 0.0), vec2(-1.0, 0.0), vec2(0.0, 1.0), vec2(0.0, -1.0),
            vec2(0.707, 0.707), vec2(-0.707, 0.707), vec2(0.707, -0.707), vec2(-0.707, -0.707));
        if (centerDepth < 0.9999) {
            for (int index = 0; index < 8; ++index) {
                vec2 sampleUv = uv + directions[index] * effects.inverseExtent * effects.switchesAndQuality.w;
                float sampleDepth = texture(sceneDepth, sampleUv).r;
                float depthDelta = centerDepth - sampleDepth;
                occlusion += smoothstep(0.0015, 0.035, depthDelta);
            }
            result *= 1.0 - effects.switchesAndQuality.z * occlusion / 8.0;
        }
    }

    if (effects.switchesAndQuality.y > 0.5) {
        vec3 north = texture(sceneColor, uv + vec2(0.0, -effects.inverseExtent.y)).rgb;
        vec3 south = texture(sceneColor, uv + vec2(0.0, effects.inverseExtent.y)).rgb;
        vec3 east = texture(sceneColor, uv + vec2(effects.inverseExtent.x, 0.0)).rgb;
        vec3 west = texture(sceneColor, uv + vec2(-effects.inverseExtent.x, 0.0)).rgb;
        float centerLuma = luma(result);
        float low = min(centerLuma, min(min(luma(north), luma(south)), min(luma(east), luma(west))));
        float high = max(centerLuma, max(max(luma(north), luma(south)), max(luma(east), luma(west))));
        if (high - low > max(0.035, high * 0.10)) {
            result = mix(result, (north + south + east + west) * 0.25, 0.42);
        }
    }
    outColor = vec4(result, 1.0);
}