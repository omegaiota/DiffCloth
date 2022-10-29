#version 330 core
uniform vec3 lightColor;
uniform vec3 spotLightPos;
uniform vec3 camPos;
uniform vec3 color1;
uniform vec3 color2;
uniform vec3 color3;
uniform vec3 colorFabric;
uniform vec3 colorWireframe;
uniform vec3 lightPos;
uniform vec3 lightDir;
uniform vec3 systemCenter;
uniform int renderMode;
uniform bool visualizeCollision;
uniform bool shading;
uniform bool lighting;
uniform float wireframeThickness;
in vec2 texCo;
in vec3 vel;
in float overrideColor;
in vec3 customColor;
out vec4 FragColor;

in vec3 normal;
in vec3 fragPos;
in vec3 fragPosLocal;
in float collides;
in float deformation;



vec3 calcPointLight(vec3 lightPos, vec3 lightColor, vec3 normal, vec3 fragPos) {
    vec3 norm = normalize(normal);
    vec3 lightDir = normalize(lightPos - fragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    float ambientStrength = 0.07;
    vec3 ambient = ambientStrength * lightColor;
    vec3 diffuse = diff * lightColor;
    return ambient + diffuse;
}

float calcSpotLight(vec3 lightPos, vec3 pointPos, vec3 lightColor, vec3 fragPos) {
    vec3 fragDir = normalize(lightPos - fragPos);
    vec3 lightDir = normalize(pointPos - lightPos);
    float theta     = dot(fragDir, -lightDir);
    float lightCutoff = 0.91;
    float lightOuterCutoff = 0.7;
    float epsilon   = lightCutoff - lightOuterCutoff;
    float intensity = clamp((theta - lightOuterCutoff) / epsilon, 0.0, 1.0);
    return intensity;
}


vec3 calcDirectionalLight(vec3 lightDir, vec3 lightColor, vec3 normal) {
    vec3 norm = normalize(normal);
    float diffuseStrength = max(dot(-norm, lightDir), 0.0);
    float ambientStrength = 0.15;
    vec3 outLight = min(diffuseStrength * (1.0 - ambientStrength), 3.0) * lightColor;
    return outLight;
}

vec3 calcAmbientLight(vec3 lightColor) {
    float ambientStrength = 0.15;
    vec3 outLight = ambientStrength * lightColor;
    return outLight;
}

void main()
{
  int FABRIC_COLOR = 0, SOLID_CUSTOM = 1, WIREFRAME = 2, WIREFRAME_COLORED = 3, SOLID_DOUBLESIDES = 4, DEFORMATION = 5, NORMAL_MAP = 6, RANDOM_COLOR = 7;

    vec3 lookDir = normalize(fragPos - camPos);
    bool lookatBack = false;
    if (dot(normal, lookDir) < 0.0) {
     lookatBack = true;
    }
    float distance = length(lightPos - fragPos);
    float lightConst = 1.0;
    float lightLinearConst =  0.027;
    float lightQuadratic =  0.0028;
    float attenuation = 1.0 / (lightConst + lightLinearConst * distance +
        		    lightQuadratic * (distance * distance));


    vec3 lightContrib = vec3(0.0,0.0,0.0);
    if (lighting) {
        float spotLightIntensity =  calcSpotLight(lightPos, spotLightPos, lightColor, fragPos);
        lightContrib += calcAmbientLight(lightColor);
        lightContrib += calcDirectionalLight(lightDir, lightColor, normal) * spotLightIntensity;

    } else {
      if (shading)
       lightContrib =  length(normal) * 0.8 + vec3(1.0,1.0,1.0) * 0.2;
      else
        lightContrib = vec3(1.0);
    }


    vec3 objColor;
    vec3 colorGreen = vec3(0.191,0.738,0.3359);

    float alpha = 1.0;
    bool collision = false &&  (collides > 0.99);
    vec3 collisionColor = vec3(0.191,0.738,0.9359);
    if (renderMode == NORMAL_MAP) { // normal map

    float thickness = wireframeThickness;
   if (texCo.x < thickness || texCo.y < thickness || abs(texCo.x + texCo.y - 1.0 ) < thickness ) {
        objColor = vec3(0.0,0.0,0.0);
      alpha = 1.0;
   } else {
               objColor = (normal + 1.0) / 2.0;
   }
    FragColor = vec4(objColor, 1.0);
                  return;
    }

    if (renderMode == WIREFRAME) { //wireframe
                   float thickness = wireframeThickness;
                   if (texCo.x < thickness || texCo.y < thickness || abs(texCo.x + texCo.y - 1.0 ) < thickness ) {
                      if (overrideColor > 0.99) { // override color
                         objColor = customColor;
                      } else
                        objColor = colorWireframe;
                      alpha = 1.0;
                   } else {
                      alpha = 0.0;
                      objColor = vec3(1,1,1);
                   }

                   if (alpha < 0.1)
                      discard;
                   FragColor = vec4(objColor, alpha);
                   return;
   }

   if (overrideColor > 0.99) { // override color
             objColor = customColor;
             alpha = 1.0;
             FragColor = vec4(objColor, alpha);
   }  else
    if (collision ) { // collision color
         objColor =  collisionColor;
         alpha = 1.0;
         FragColor = vec4(objColor, alpha);
         return;
    } else {

      if (renderMode == FABRIC_COLOR) {
         objColor = colorFabric;

      } else if (renderMode == SOLID_CUSTOM) { // solid color
         objColor = color1;

     } else if (renderMode == WIREFRAME_COLORED) {//wireframe
                float thickness = 0.05;
                if (texCo.x < thickness || texCo.y < thickness || abs(texCo.x + texCo.y - 1.0 ) < thickness ) {
                   objColor = vec3(0,0,0);
                } else {
                   objColor = color1;
                }

                FragColor = vec4(objColor, 1.0);
                return;
    } else if (renderMode == SOLID_DOUBLESIDES) { // solid two sides
        objColor = vec3(255.0,115.0,0.0) ; //orange
        if (lookatBack)
            objColor = vec3(49.0, 189.0, 117.0); // purple
        objColor = vec3(objColor.r / 255.0, objColor.g / 255.0, objColor.b / 255.0);
           objColor = color2 ; //orange
                if (lookatBack)
                    objColor = color3; // purple
     } else if (renderMode == DEFORMATION) {
         float clampedVal = (clamp(deformation, 0.0, 4.0) - 1.0) / 3.0; // 0 ~ 3
         float curved = pow(10.0, clampedVal) / 10.0;
         objColor = vec3(curved, 0.0, 1.0-curved);
         FragColor = vec4(objColor, 1.0);
         return;
     }
    }

    float gamma = 2.2;
    objColor = pow(objColor, vec3(gamma)); // first convert color from sRGB to linearRGB

    vec3 result = lightContrib * objColor; // lighting calculation
// gamma correction
    result = pow(result, vec3(1.0/gamma));  // convert linearRGB back to sRGB

    FragColor = vec4(result, alpha);






 }