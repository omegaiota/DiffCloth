#version 330 core
layout (location = 0) in vec3 aPos;
in vec2 texCoord;
out vec2 texCo;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    texCo = texCoord;
}