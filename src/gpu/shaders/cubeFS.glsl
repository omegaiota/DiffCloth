#version 330 core
in vec3 Normal;
in vec2 TexCoord;
in vec3 FragPos;

out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 lightColor;

uniform vec3 viewPos;
uniform sampler2D ourTexture;

vec3 phongLighting(float specularStrength, float ambientStrength, vec3 normal, vec3 fragPos, vec3 camPos, vec3 lightPos, vec3 lightColor) {
    vec3 norm = normalize(normal);
    vec3 lightDir = normalize(lightPos - fragPos);
    vec3 viewDir = normalize(camPos - fragPos);
    vec3 reflectDir = reflect(-lightDir, norm);

    vec3 ambientLight = ambientStrength * lightColor;

    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuseLight = diff * lightColor;

    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);

    vec3 specularLight = specularStrength * spec * lightColor;
    vec3 result = (ambientLight + diffuseLight + specularLight);
    return result;
}
void main()
{
    float speFac = 0.5;
    float ambFac = 0.2;

    vec3 objectColor = texture(ourTexture, TexCoord).rgb;
    vec3 result = phongLighting(speFac, ambFac, Normal, FragPos, viewPos, lightPos, lightColor) * objectColor;

    FragColor = vec4(result, 1.0);

}