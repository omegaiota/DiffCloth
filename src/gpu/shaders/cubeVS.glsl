#version 330 core
layout (location = 0) in vec3 aPos;
//layout (location = 1) in vec3 aColor;
layout (location = 1) in vec2 aTexCoord;
layout (location = 2) in vec3 aNormal;

out vec3 ourColor;
out vec2 TexCoord;
out vec3 Normal;
out vec3 FragPos;

uniform vec3 viewPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat3 normalMat;


void main()
{
    gl_Position =  projection * view * model * vec4(aPos, 1.0);
    //gl_Position = vec4(aPos, 1.0f);    
    //ourColor = aColor;
    FragPos = vec3(model * vec4(aPos, 1.0));
    TexCoord = aTexCoord;

    Normal = normalMat * aNormal;
//    Normal = vec3(model2 * vec4(aNormal, 1.0));
//    Normal = aNormal;

}