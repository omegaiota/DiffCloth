#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in  vec2 texCoord;
layout (location = 2) in vec3 velocity;
layout (location = 3) in float useCustomColor;
layout (location = 4) in vec3 color;
layout (location = 5) in vec3 normalIn;
layout (location = 6) in float isCollide;
layout (location = 7) in float deform;

out vec2 texCo;
out vec3 vel;
out float overrideColor;
out vec3 customColor;
out vec3 normal;
out vec3 fragPos;
out vec3 fragPosLocal;
out float collides;
out float deformation;

uniform vec3 systemCenter;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    texCo = texCoord;
    vel = velocity;
    customColor = color;
    overrideColor = useCustomColor;
    normal = normalIn;
    collides = isCollide;
    fragPos = vec3(model * vec4(aPos, 1.0));
    fragPosLocal = vec3(model * vec4(aPos-systemCenter, 1.0));
    deformation = deform;
}