#version 330 core
out vec4 FragColor;
in vec3 color;
uniform int renderMode;


in vec3 fragPos;
void main() {
    if (renderMode == 0) {
        float intensity = clamp(0.3 - clamp(length(fragPos), 0, 60) / 60.0, 0.0, 1.0);
        FragColor = vec4(intensity, intensity, intensity, intensity);
        return;
    }

    FragColor = vec4(color, 1.0);
    return;


}