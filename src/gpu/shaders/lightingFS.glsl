#version 330 core
struct Material {
    sampler2D diffuse;
    sampler2D specular;
    float shininess;
};


struct Light {
    vec3 position;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

uniform Light light;
in vec3 Normal;
in vec2 TexCoord;
in vec3 FragPos;

out vec4 FragColor;
uniform vec3 viewPos;
//uniform sampler2D ourTexture;
uniform Material material;

vec3 phongLighting(Material material, vec3 normal, vec3 fragPos, vec3 camPos, Light light, vec2 TexCoord) {
    vec3 norm = normalize(normal);
    vec3 lightDir = normalize(light.position - fragPos);
    vec3 viewDir = normalize(camPos - fragPos);
    vec3 reflectDir = reflect(-lightDir, norm);

    vec3 ambientLight  = light.ambient * texture(material.diffuse, TexCoord).rgb;

    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuseLight = light.diffuse * diff * texture(material.diffuse, TexCoord).rgb;

    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);

    vec3 specularLight = (texture(material.specular, TexCoord).rgb * spec) * light.specular;
    vec3 result = (ambientLight + diffuseLight + specularLight);

    return result;
}

void main()
{
    vec3 result = phongLighting(material, Normal, FragPos, viewPos, light, TexCoord);

    FragColor = vec4(result, 1.0);

}
