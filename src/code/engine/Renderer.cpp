//
// Created by Yifei Li on 3/4/21.
// Email: liyifei@csail.mit.edu
//

#include "Renderer.h"

unsigned int Renderer::gridVAO = -1;
unsigned int Renderer::gridVBO = -1;
unsigned int Renderer::gridIBO = -1;
unsigned int Renderer::fixedPointMeshVAO = -1;
int Renderer::gridLength = 0;
bool Renderer::gridInitialized = false;
bool Renderer::fixedPointInitialized = false;


static Vec3d posToHashColor(Vec3d in) {
  in *= 27644437;
  in += Vec3d(28497324, 123874, 192384);

  Vec3i out((int) in[0], (int) in[1], (int) in[2]);
  out[0] %= 6691;
  out[1] %= 6763;
  out[2] %= 7723;

  return Vec3d(out[0] * 1.0 / 6691, out[1] * 1.0 / 6763, out[2] * 1.0 / 7723);
}

void Renderer::useClothShaderAndSetUniforms(Simulation *msSystem, Shader *clothShader, Viewer &window) {
  clothShader->use();
  clothShader->setVec3("objectColor", 1.0f, 0.3f, 0.31f);
  clothShader->setVec3("colorFabric", msSystem->sceneConfig.fabric.color);
  clothShader->setVec3("lightPos", msSystem->lightPos);
  clothShader->setVec3("spotLightPos", msSystem->particles[0].pos);
  clothShader->setVec3("lightDir", msSystem->lightDir);
  clothShader->setBool("shading", true);
  clothShader->setBool("lighting", true);
  clothShader->setFloat("wireframeThickness", window.wireframeThickness);

  clothShader->setVec3("color1", window.color_custom);
  clothShader->setVec3("color2", window.color_doubleside_1);
  clothShader->setVec3("color3", window.color_doubleside_2);
  clothShader->setVec3("colorWireframe", 1.0f, 1.0f, 1.0f);
  clothShader->setMat4("view", window.camera.getViewMat());
  clothShader->setMat4("projection", window.getProjectionMat());
  clothShader->setMat4("model",
                       glm::translate(glm::mat4(1.0f),
                                      glm::vec3(msSystem->systemCenter[0], msSystem->systemCenter[1],
                                                msSystem->systemCenter[2])));
  clothShader->setInt("renderMode", window.myRendermode);
  clothShader->setVec3("lightColor", glm::vec3(1.0, 1.0, 1.0));
  clothShader->setVec3("camPos", window.camera.getCamPos());
  clothShader->setVec3("systemCenter",
                       glm::vec3(msSystem->systemCenter[0], msSystem->systemCenter[1], msSystem->systemCenter[2]));
  clothShader->setBool("visualizeCollision", window.visualizeCollision);
}

void Renderer::useSimpleShaderAndSetUniforms(Simulation *msSystem, Shader *simpleShader, Viewer &window) {
  simpleShader->use();
  simpleShader->setMat4("view", window.camera.getViewMat());
  simpleShader->setMat4("projection", window.getProjectionMat());
  simpleShader->setMat4("model",
                        glm::translate(glm::mat4(1.0f),
                                       glm::vec3(msSystem->systemCenter[0], msSystem->systemCenter[1],
                                                 msSystem->systemCenter[2])));
  simpleShader->setInt("renderMode", 0);
}

void
Renderer::fillTriangleToVertexArr(Triangle &m, float *vertexArr, int offset, int numFields, Vec3d color, Vec3d center,
                                  bool useVertexNormal) {
  // {pos(3),texCorrd(2),velocity(3),override?(1),color(3),normal(3)}
  Particle *vs[] = {m.p0(), m.p1(), m.p2()};
  for (int j = 0; j < 3; j++) {
    int startIdx = offset + j * numFields;
    Vec3d p = vs[j]->pos;
    fillArrayWithVec3(vertexArr, p + center, startIdx + 0); // 0-2
    vertexArr[startIdx + 3] = (j == 1) ? 1.0 : 0.0; // (0, 0 ), (1, 0), (0, 1)
    vertexArr[startIdx + 4] = (j >= 2) ? 1.0 : 0.0;
    fillArrayWithVec3(vertexArr, vs[j]->velocity, startIdx + 5); // 5-7
    vertexArr[startIdx + 8] = m.overrideColor;
    fillArrayWithVec3(vertexArr, color, startIdx + 9); // 9-11
    if (useVertexNormal)
      fillArrayWithVec3(vertexArr, vs[j]->normal, startIdx + 12); // 9-11
    else
      fillArrayWithVec3(vertexArr, m.normal, startIdx + 12);

  }
}

void Renderer::renderMesh(Simulation *msSystem, Shader *clothShader, Viewer &window, Simulation::FileMesh &model,
                          Vec3d color,
                          std::vector<Vec3d> center, Vec3d initialDir, Vec3d pointDir, bool shading, bool lighting) {
  useClothShaderAndSetUniforms(msSystem, clothShader, window);

  clothShader->setBool("shading", shading);
  clothShader->setBool("lighting", lighting);

  glm::mat4 rotationXYZ = glm::mat4(1.0f);

  if ((pointDir - initialDir).norm() > 1e-5) {
    Vec3d perp = initialDir.cross(pointDir);
    double angle = std::acos(pointDir.dot(initialDir) / pointDir.norm() / initialDir.norm());
    Vec3d rotationDegrees = dirVecToAngleDegrees(pointDir) - dirVecToAngleDegrees(initialDir);
    rotationXYZ = glm::rotate(glm::mat4(1.0f), (float) angle, glm::vec3(perp[0], perp[1], perp[2]));
  }

  int fieldNum = 16;
  int totalTriangles = model.triangles.size();
  float vertexArr[3 * fieldNum * totalTriangles];

  for (int i = 0; i < model.triangles.size(); i++) {
    // {pos(3),texCorrd(2),velocity(3),override?(1),color(3),normal(3)}
    for (int j = 0; j < 3; j++) {
      int startIdx = i * (fieldNum * 3) + j * fieldNum;
      Vec3d p = model.points[model.triangles[i][j]];
      fillArrayWithVec3(vertexArr, p, startIdx + 0); // 0-2
      vertexArr[startIdx + 3] = (j == 1) ? 1.0 : 0.0; // (0, 0 ), (1, 0), (0, 1)
      vertexArr[startIdx + 4] = (j >= 2) ? 1.0 : 0.0;
      fillArrayWithVec3(vertexArr, Vec3d(0, 0, 0), startIdx + 5); // 5-7
      vertexArr[startIdx + 8] = 1;
      fillArrayWithVec3(vertexArr, color, startIdx + 9); // 9-11
      fillArrayWithVec3(vertexArr, model.normals[model.triangles[i][j]], startIdx + 12);

    }
  }

  VertexArrayObject vertexVAO = VertexArrayObject((float *) vertexArr,
                                                  3 * totalTriangles,
                                                  fieldNum * sizeof(float));
  std::vector<int> fieldComponents = {3, 2, 3, 1, 3, 3, 1};
  vertexArrAddAtrribInfo(vertexVAO, fieldComponents);


  for (Vec3d &c : center) {
    vertexVAO.use();
    Vec3d totalTranslate = msSystem->systemCenter + c;
    glm::mat4 translation = glm::translate(glm::mat4(1.0f),
                                           glm::vec3(totalTranslate[0], totalTranslate[1], totalTranslate[2]));
    clothShader->setMat4("model", translation * rotationXYZ);

    glDrawArrays(GL_TRIANGLES, 0, 3 * totalTriangles);
    glBindVertexArray(0);
  }
}


void Renderer::renderNormals(Simulation *msSystem, Shader *clothShader, Viewer &window, Simulation::FileMesh &model, std::vector<std::pair<Vec3d, Vec3d>> &pointsAndDirs, Vec3d initialDir) {
  useClothShaderAndSetUniforms(msSystem, clothShader, window);

  clothShader->setBool("shading", false);
  clothShader->setBool("lighting", false);



  int fieldNum = 16;
  int totalTriangles = model.triangles.size();
  float vertexArr[3 * fieldNum * totalTriangles];

  for (int i = 0; i < model.triangles.size(); i++) {
    // {pos(3),texCorrd(2),velocity(3),override?(1),color(3),normal(3)}
    for (int j = 0; j < 3; j++) {
      int startIdx = i * (fieldNum * 3) + j * fieldNum;
      Vec3d p = model.points[model.triangles[i][j]];
      fillArrayWithVec3(vertexArr, p, startIdx + 0); // 0-2
      vertexArr[startIdx + 3] = (j == 1) ? 1.0 : 0.0; // (0, 0 ), (1, 0), (0, 1)
      vertexArr[startIdx + 4] = (j >= 2) ? 1.0 : 0.0;
      fillArrayWithVec3(vertexArr, Vec3d(0, 0, 0), startIdx + 5); // 5-7
      vertexArr[startIdx + 8] = 0; // use color1
      fillArrayWithVec3(vertexArr, Vec3d(0,0,1), startIdx + 9); // 9-11
      fillArrayWithVec3(vertexArr, model.normals[model.triangles[i][j]], startIdx + 12);

    }
  }

  VertexArrayObject vertexVAO = VertexArrayObject((float *) vertexArr,
                                                  3 * totalTriangles,
                                                  fieldNum * sizeof(float));
  std::vector<int> fieldComponents = {3, 2, 3, 1, 3, 3, 1};
  vertexArrAddAtrribInfo(vertexVAO, fieldComponents);


  for (std::pair<Vec3d, Vec3d> &instance : pointsAndDirs) {
    Vec3d c = instance.first;
    Vec3d pointDir = instance.second;
    pointDir = pointDir.normalized();

    glm::mat4 rotationXYZ = glm::mat4(1.0f);
    if ((pointDir - initialDir).norm() > 1e-5) {
      Vec3d perp = initialDir.cross(pointDir);
      double angle = std::acos(pointDir.dot(initialDir) / pointDir.norm() / initialDir.norm());
      Vec3d rotationDegrees = dirVecToAngleDegrees(pointDir) - dirVecToAngleDegrees(initialDir);
      rotationXYZ = glm::rotate(glm::mat4(1.0f), (float) angle, glm::vec3(perp[0], perp[1], perp[2]));
    }

    vertexVAO.use();
    Vec3d totalTranslate = msSystem->systemCenter + c;
    glm::mat4 translation = glm::translate(glm::mat4(1.0f),
                                           glm::vec3(totalTranslate[0], totalTranslate[1], totalTranslate[2]));
    clothShader->setMat4("model", translation * rotationXYZ);
    clothShader->setInt("renderMode", 1);

    Vec3d color = (pointDir + Vec3d::Ones()) / 2.0;
    clothShader->setVec3("color1", color);
    clothShader->use();

    glDrawArrays(GL_TRIANGLES, 0, 3 * totalTriangles);
    glBindVertexArray(0);
  }
}


void Renderer::renderFixedPoints(Simulation *msSystem, Shader *clothShader, Viewer &window, std::vector<Vec3d> &points,
                                 Vec3d color, bool renderClips, bool useHashColor) {
  if (renderClips) {
    std::vector<Vec3d> pointsWithOffset;
    for (Vec3d &p : points) {
      pointsWithOffset.emplace_back(p + Vec3d(0, msSystem->clip.dim * 0.5, 0));
    }
    renderMesh(msSystem, clothShader, window, msSystem->clip, COLOR_WOOD,
               pointsWithOffset, Vec3d(0, 1, 0), Vec3d(0, 1, 0), true, true);
  }

  useClothShaderAndSetUniforms(msSystem, clothShader, window);
  clothShader->setBool("shading", false);
  clothShader->setBool("lighting", false);

  int fieldNum = 16;
  float vertexArr[3 * fieldNum * msSystem->sphereForFixedPointRender.mesh.size()];

  for (int i = 0; i < msSystem->sphereForFixedPointRender.mesh.size(); i++) {
    // {pos(3),texCorrd(2),velocity(3),override?(1),color(3),normal(3)}
    fillTriangleToVertexArr(msSystem->sphereForFixedPointRender.mesh[i], vertexArr, i * (fieldNum * 3), fieldNum,
                            color, Vec3d(0, 0, 0), true); // TODO: should this be vertex normal?
  }

  VertexArrayObject fixedPointMeshVAO = VertexArrayObject((float *) vertexArr,
                                                          1 * 3 * msSystem->sphereForFixedPointRender.mesh.size(),
                                                          fieldNum * sizeof(float));
  std::vector<int> fieldComponents = {3, 2, 3, 1, 3, 3, 1};
  vertexArrAddAtrribInfo(fixedPointMeshVAO, fieldComponents);
  fixedPointMeshVAO.use();

  for (Vec3d &c : points) {
    glBindVertexArray(fixedPointMeshVAO.VAO);
    Vec3d totalTranslate = msSystem->systemCenter + c;
    glm::mat4 translation = glm::translate(glm::mat4(1.0f),
                                           glm::vec3(totalTranslate[0], totalTranslate[1], totalTranslate[2]));
    clothShader->setMat4("model", translation);

    glDrawArrays(GL_TRIANGLES, 0, msSystem->sphereForFixedPointRender.mesh.size() * 3);
    glBindVertexArray(0);
  }
}

void Renderer::renderLines(Simulation *msSystem, Shader *simpleShader, Viewer &window,
                           const std::vector<std::pair<Vec3d, Vec3d>> &lines ) {
  if (lines.empty())
    return;

  useSimpleShaderAndSetUniforms(msSystem, simpleShader, window);
  simpleShader->setInt("renderMode", 1); // SOCK

  std::vector<glm::vec3> vertices;
  std::vector<glm::uvec2> indices;

  int count = 0;
  for (const std::pair<Vec3d, Vec3d> &line : lines) {
    vertices.emplace_back(line.first[0], line.first[1], line.first[2]);
    vertices.emplace_back(COLOR_WOOD[0], COLOR_WOOD[1], COLOR_WOOD[2]); // SOCK POINT

    vertices.emplace_back(line.second[0], line.second[1], line.second[2]);
    Vec3d color = posToHashColor(line.second);
    vertices.emplace_back(color[0], color[1], color[2]); // FOOT POINT

    indices.emplace_back(count, count + 1);
    count += 2;
  }

  unsigned int IBO;
  unsigned int VAO, VBO;

//  VertexArrayObject vertexVAO = VertexArrayObject(vertices,
//                                                  lines.size() * 2,
//                                                  2 * sizeof(glm::vec3));
//    std::vector<int> fieldComponents = {3, 3};
//    vertexArrAddAtrribInfo(vertexVAO, fieldComponents);

  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);

  glGenBuffers(1, &VBO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), glm::value_ptr(vertices[0]), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), nullptr);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), (void *) (sizeof(glm::vec3)));
  glEnableVertexAttribArray(1);

  glGenBuffers(1, &IBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(glm::uvec2), glm::value_ptr(indices[0]),
               GL_STATIC_DRAW);


  glBindVertexArray(0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glDepthMask(GL_FALSE);
//    vertexVAO.use();

  glBindVertexArray(VAO);
  glDrawElements(GL_LINES, (GLuint) indices.size() * 2, GL_UNSIGNED_INT, NULL);
  glBindVertexArray(0);
  glDepthMask(GL_TRUE);

  glDeleteBuffers(1, &VBO);
  glDeleteBuffers(1, &IBO);
  glDeleteVertexArrays(1, &VAO);
}


void Renderer::renderLines(Simulation *msSystem, Shader *simpleShader, Viewer &window,
                           const std::vector<std::pair<Vec3d, Vec3d>> &lines, Vec3d color) {
  if (lines.empty())
    return;

  useSimpleShaderAndSetUniforms(msSystem, simpleShader, window);
  simpleShader->setInt("renderMode", 1); // SOCK

  std::vector<glm::vec3> vertices;
  std::vector<glm::uvec2> indices;

  int count = 0;
  for (const std::pair<Vec3d, Vec3d> &line : lines) {
    vertices.emplace_back(line.first[0], line.first[1], line.first[2]);
    vertices.emplace_back(color[0], color[1], color[2]); // SOCK POINT

    vertices.emplace_back(line.second[0], line.second[1], line.second[2]);
    vertices.emplace_back(color[0], color[1], color[2]); // FOOT POINT

    indices.emplace_back(count, count + 1);
    count += 2;
  }

  unsigned int IBO;
  unsigned int VAO, VBO;

//  VertexArrayObject vertexVAO = VertexArrayObject(vertices,
//                                                  lines.size() * 2,
//                                                  2 * sizeof(glm::vec3));
//    std::vector<int> fieldComponents = {3, 3};
//    vertexArrAddAtrribInfo(vertexVAO, fieldComponents);

  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);

  glGenBuffers(1, &VBO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), glm::value_ptr(vertices[0]), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), nullptr);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), (void *) (sizeof(glm::vec3)));
  glEnableVertexAttribArray(1);

  glGenBuffers(1, &IBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(glm::uvec2), glm::value_ptr(indices[0]),
               GL_STATIC_DRAW);


  glBindVertexArray(0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glDepthMask(GL_FALSE);
//    vertexVAO.use();

  glBindVertexArray(VAO);
  glDrawElements(GL_LINES, (GLuint) indices.size() * 2, GL_UNSIGNED_INT, NULL);
  glBindVertexArray(0);
  glDepthMask(GL_TRUE);

  glDeleteBuffers(1, &VBO);
  glDeleteBuffers(1, &IBO);
  glDeleteVertexArrays(1, &VAO);
}


void Renderer::renderGrid(Simulation *msSystem, Shader *simpleShader, Viewer &window) {
  useSimpleShaderAndSetUniforms(msSystem, simpleShader, window);
  simpleShader->setInt("renderMode", 0); // ground grid

  if (!gridInitialized) {
    int slices = 50;
    std::vector<glm::vec3> vertices;
    std::vector<glm::uvec2> indices;

    for (int j = 0; j <= slices; ++j) {
      for (int i = 0; i <= slices; ++i) {
        double spacing = 1.2;

        float x = (i - slices / 2.0) * spacing;
        float y = -2;
        float z = (j - slices / 2.0) * spacing;
        vertices.emplace_back(x, y, z);
      }
    }

    for (int j = 0; j < slices; ++j) {
      for (int i = 0; i < slices; ++i) {
        int row1 = j * (slices + 1);
        int row2 = (j + 1) * (slices + 1);
        // horizontal
        indices.emplace_back(row1 + i, row1 + i + 1);
        //vertical
        indices.emplace_back(row1 + i, row2 + i);
      }
    }

    glGenVertexArrays(1, &gridVAO);
    glBindVertexArray(gridVAO);

    glGenBuffers(1, &gridVBO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), glm::value_ptr(vertices[0]), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    glGenBuffers(1, &gridIBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gridIBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(glm::uvec2), glm::value_ptr(indices[0]),
                 GL_STATIC_DRAW);


    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    gridLength = (GLuint) indices.size() * 2;
    gridInitialized = true;
  }

  glDepthMask(GL_FALSE);
  glBindVertexArray(gridVAO);
  glDrawElements(GL_LINES, gridLength, GL_UNSIGNED_INT, NULL);
  glBindVertexArray(0);
  glDepthMask(GL_TRUE);

//      glDeleteBuffers(1, &gridVBO);
//      glDeleteBuffers(1, &gridIBO);
//      glDeleteVertexArrays(1, &gridVAO);
}

void Renderer::renderSplines(Simulation *msSystem, Shader *clothShader, Viewer &window, std::vector<Spline> &splines,
                             Vec3d color) {
  useClothShaderAndSetUniforms(msSystem, clothShader, window);
  glm::vec3 lightColor;
  Camera &cam = window.camera;
  clothShader->setBool("shading", false);
  clothShader->setBool("lighting", false);
  Mat3x3d rotMax;
  rotMax = Eigen::AngleAxis<double>(0, Vec3d::UnitX())
           * Eigen::AngleAxis<double>(0, Vec3d::UnitY())
           * Eigen::AngleAxis<double>(0.5 * 3.14159, Vec3d::UnitZ());


  for (Spline &s : splines) {
    std::vector<Vec3d> samplePoints;
    std::vector<Vec3d> upPoints;
    int SAMPLE_NUM = 20;
    for (int i = 0; i < SAMPLE_NUM; i++) {
      Vec3d p = s.evalute(i * 1.0 / (SAMPLE_NUM - 1));
      Vec3d tangant = s.evalute(i * 1.0 / (SAMPLE_NUM - 1), 1);
      tangant.normalize();
      Vec3d normal = rotMax * tangant;
      samplePoints.emplace_back(p);
      upPoints.emplace_back(p + normal * 0.2);
    }

    int numFields = 15;
    int TOTAL_TRIANGLES = (samplePoints.size() - 1) * 2;
    float vertexArr[TOTAL_TRIANGLES * 3 * numFields];


    for (int i = 0; i < TOTAL_TRIANGLES; i++) {
      Vec3d p0, p1, p2;

      int idx = i / 2;
      if (i % 2 == 0) {
        p0 = samplePoints[idx];
        p1 = samplePoints[idx + 1];
        p2 = upPoints[idx];
      } else {
        p0 = upPoints[idx];
        p1 = samplePoints[idx + 1];
        p2 = upPoints[idx + 1];
      }

      Vec3d vs[] = {p0, p1, p2};
      for (int j = 0; j < 3; j++) {
        int startIdx = i * (numFields * 3) + j * numFields;
        fillArrayWithVec3(vertexArr, vs[j], startIdx + 0);
        vertexArr[startIdx + 3] = (j == 1) ? 1.0 : 0.0; // (0, 0 ), (1, 0), (0, 1)
        vertexArr[startIdx + 4] = (j >= 2) ? 1.0 : 0.0;
        vertexArr[startIdx + 8] = true; // override color
        fillArrayWithVec3(vertexArr, color, startIdx + 9); // override with orange color
      }
    }


    VertexArrayObject vertexVAO = VertexArrayObject((float *) vertexArr,
                                                    TOTAL_TRIANGLES * 3,
                                                    numFields * sizeof(float));

    int fieldComponents[6] = {3, 2, 3, 1, 3, 3};
    int count = 0;
    for (int i = 0; i < 6; i++) {
      vertexVAO.addVertexAttrib(i, fieldComponents[i], count * sizeof(float));
      count += fieldComponents[i];
    }

//        std::vector<int> fieldComponents = {3, 2, 3, 1, 3, 3};
//        vertexArrAddAtrribInfo(vertexVAO, fieldComponents);
    vertexVAO.use();

    glDrawArrays(GL_TRIANGLES, 0, TOTAL_TRIANGLES * 3);
    glBindVertexArray(0);

  }
}

void Renderer::renderPrimitive(Primitive& p, Shader* shader, Vec3d center) {
  shader->use();
//  std::printf("render primitive of type %d center (%2.f, %.2f, %.2f)\n", p.type, center[0], center[1], center[2]);

  int numFields = 15;
  int TRIANGLE_NUM = p.mesh.size();
  float vertexArr[3 * TRIANGLE_NUM * numFields];
  for (int i = 0; i < TRIANGLE_NUM; i++) {
    Triangle &m = p.mesh[i];
    fillTriangleToVertexArr(m, vertexArr, i * (numFields * 3), numFields, m.color,
                            center, true);
  }

  VertexArrayObject vertexVAO = VertexArrayObject((float *) vertexArr,
                                                  TRIANGLE_NUM * 3,
                                                  numFields * sizeof(float));
  std::vector<int> fieldComponents = {3, 2, 3, 1, 3, 3};
  vertexArrAddAtrribInfo(vertexVAO, fieldComponents);
  vertexVAO.use();
  glDrawArrays(GL_TRIANGLES, 0, TRIANGLE_NUM * 3);
  glBindVertexArray(0);
}

void Renderer::renderPrimitives(std::vector<Primitive*>& primitives, Shader* clothShader,  int recordIdx, Vec3d center = Vec3d(0,0,0)) {
    for (Primitive *p : primitives) {
      if (p->isEnabled && p->simulationEnabled) {
        if (p->isPrimitiveCollection) {
//          std::printf("p is collection with center (%.2f, %.2f, %.2f) accumulated (%.2f %.2f %.2f) record %d/%zu\n",
//                      p->forwardRecords[recordIdx][0],
//                      p->forwardRecords[recordIdx][1],
//                      p->forwardRecords[recordIdx][2],
//                      center[0], center[1], center[2],
//                      recordIdx, p->forwardRecords.size()
//                      );
          renderPrimitives(p->primitives, clothShader, recordIdx, center + p->forwardRecords[recordIdx]);
        } else {
//          std::printf("p is single with center (%.2f, %.2f, %.2f) accumulated center (%.2f, %.2f, %.2f) record %d/%zu\n",
//                      p->forwardRecords[recordIdx][0],
//                      p->forwardRecords[recordIdx][1],
//                      p->forwardRecords[recordIdx][2],
//                      center[0],
//                      center[1], center[2],
//                      recordIdx, p->forwardRecords.size()
//          );
          renderPrimitive(*p, clothShader, p->forwardRecords[recordIdx] + center);
        }
      }
    }
}
void
Renderer::stepAndRenderSystem(Simulation *msSystem, Shader *clothShader, Shader *simpleShader, Viewer &viewer,
                              std::vector<Simulation::ForwardInformation> &recordToUse, int recordIdx,
                              bool isGroundTruthSystem) {

  Simulation::ForwardInformation record;
  // TODO: change to atomic
  if (recordIdx >= recordToUse.size()) {
    std::printf("want to render step %d but only %zu records are available\n", recordIdx, recordToUse.size());
    return;
  } else {
    record = recordToUse[recordIdx];
  }

  if (!isGroundTruthSystem) {
    if (viewer.renderGrid)
      renderGrid(msSystem, simpleShader, viewer);
  }


  useClothShaderAndSetUniforms(msSystem, clothShader, viewer);
  clothShader->setVec3("spotLightPos", (Vec3d) record.x.segment(0, 3));


  if (isGroundTruthSystem) {
    clothShader->use();
    clothShader->setInt("renderMode", Viewer::RenderMode::WIREFRAME);
    nanogui::Color color(1.0, 0.0, 0.0, 1.0f);
    clothShader->setVec3("colorWireframe", color);
  }
  VecXd x_now, v_now;


  x_now = record.x;
  v_now = record.v;


  bool vertexCollision[msSystem->particles.size()];
  for (int i = 0; i < msSystem->particles.size(); i++) {
    vertexCollision[i] = false;
  }

  for (const Simulation::SelfCollisionInformation &info : record.collisionInfos.first.second) {
    vertexCollision[info.particleId1] = true;
    vertexCollision[info.particleId2] = true;
  }

  for (const Simulation::PrimitiveCollisionInformation &info : record.collisionInfos.first.first) {
    vertexCollision[info.particleId] = true;
  }

  {
    msSystem->updateParticleNormals(x_now);
    int numFields = 17;
    bool useVelocity = false;

    int EMIT_SIZE = 200; // Number of Triangles in a single draw emit
    float vArr[3 * numFields * EMIT_SIZE];
    int curIdx = -1;
    for (int triIdx = 0; triIdx < msSystem->mesh.size(); triIdx++) {
      Triangle &m = msSystem->mesh[triIdx];
      double deformation = m.getDeformation(x_now);
      curIdx++;
      Particle *vs[] = {m.p0(), m.p1(), m.p2()};
      // {pos(3),texCorrd(2),velocity(3),override?(1),color(3),normal(3), collision(1), deformation(1)}
      fillTriangleToVertexArr(m, vArr, curIdx * 3 * numFields, numFields, Vec3d(0, 0, 0), Vec3d(0, 0, 0), true);

      Vec3d randColor;
      srand(triIdx);

      srand(rand());

      randColor.setRandom();
      randColor = randColor + Vec3d::Ones();
      randColor *= 0.5;
      for (int vIdx = 0; vIdx < 3; vIdx++) {
        Vec3d pos = x_now.segment(vs[vIdx]->idx * 3, 3);
        Vec3d vel = v_now.segment(vs[vIdx]->idx * 3, 3);
        int startIdx = (curIdx * 3 + vIdx) * numFields;
        fillArrayWithVec3(vArr, pos, startIdx + 0); // 0-2
        fillArrayWithVec3(vArr, vel, startIdx + 5); // 5-7
        if (viewer.triangleId == triIdx) {
          fillArrayWithFloat(vArr, 1, startIdx + 8);
          fillArrayWithVec3(vArr, Vec3d(1, 0, 0), startIdx + 9);
        } else {
          fillArrayWithFloat(vArr, m.overrideColor, startIdx + 8);
        }
        if (viewer.myRendermode == Viewer::RenderMode::RANDOM_COLOR) {
          fillArrayWithFloat(vArr, 1, startIdx + 8);
          fillArrayWithVec3(vArr, randColor, startIdx + 9);
        }
        fillArrayWithFloat(vArr, vertexCollision[vs[vIdx]->idx], startIdx + 15);
        fillArrayWithFloat(vArr, deformation, startIdx + 16);
      }
      int currentTriangleCount = curIdx + 1;
      if ((currentTriangleCount == EMIT_SIZE) || (triIdx + 1 == msSystem->mesh.size())) {
        VertexArrayObject vertexVAO = VertexArrayObject((float *) vArr,
                                                        1 * 3 * currentTriangleCount,
                                                        numFields * sizeof(float));
        std::vector<int> fieldComponents = {3, 2, 3, 1, 3, 3, 1, 1};
        vertexArrAddAtrribInfo(vertexVAO, fieldComponents);
        vertexVAO.use();
        glDrawArrays(GL_TRIANGLES, 0, currentTriangleCount * 3);
        glBindVertexArray(0);

        curIdx = -1;
      }

    }

    renderPrimitives(msSystem->allPrimitivesToRender, clothShader, recordIdx);
  }

  // TODO: needs rewrite for robotics task
  if (viewer.splineVisualization) {
    if (msSystem->sceneConfig.trajectory != TrajectoryConfigs::PER_STEP_TRAJECTORY)
    renderSplines(msSystem, clothShader, viewer, record.splines, COLOR_YELLOW);

  }

  if (viewer.perStepTrajectoryVisualization) {
    if (msSystem->sceneConfig.trajectory == TrajectoryConfigs::PER_STEP_TRAJECTORY) {
      std::vector<std::pair<Vec3d, Vec3d>> lines;
      for (int i = 0; i < msSystem->sysMat[msSystem->currentSysmatId].fixedPoints.size(); i++) {
        lines.clear();
        for (int pIdx = 0; pIdx <  ((int)  msSystem->perstepTrajectory.size())-1; pIdx++) {
          lines.emplace_back(msSystem->perstepTrajectory[pIdx].segment(i * 3, 3), msSystem->perstepTrajectory[pIdx+1].segment(i * 3, 3));
        }
        if (!lines.empty())
          renderLines(msSystem, simpleShader, viewer, lines, Vec3d(1, 1, 0));

      }
    }

  }

  if (viewer.perStepGradientVisualization) {


    std::vector<std::pair<Vec3d, Vec3d>> pointsAndDirs;

       for (int i = 0; i < msSystem->sysMat[msSystem->currentSysmatId].fixedPoints.size(); i++) {
         for (int pIdx = 0; pIdx <  ((int)  recordToUse.size())-1; pIdx++) {
            Vec3d loc = recordToUse[pIdx].x_fixedpoints.segment(i * 3, 3);
           bool skip = false;
           if (!pointsAndDirs.empty()) {
             Vec3d locLast = pointsAndDirs[pointsAndDirs.size()-1].first;
             skip = (locLast-loc).norm() < 0.1;

           }
           if ((!skip) && (msSystem->perStepGradient.size() > pIdx)) {
             pointsAndDirs.emplace_back(loc, -msSystem->perStepGradient[pIdx].segment(i * 3, 3)); /* render rnegative gradient -> descent direction */

           }
         }
      }

        if (!pointsAndDirs.empty())
        renderNormals(msSystem, clothShader, viewer, msSystem->arrow3, pointsAndDirs, Vec3d(1, 0, 0));


  }

  std::vector<Vec3d> fixedPoints;
  for (FixedPoint &p : msSystem->sysMat[record.sysMatId].fixedPoints)
    fixedPoints.emplace_back(record.x_fixedpoints.segment(p.idx * 3, 3));
  renderFixedPoints(msSystem, clothShader, viewer, fixedPoints, COLOR_EGGPLANT, true);


  if ((viewer.particleId * 3 < record.x.rows()) && (viewer.particleId >= 0)) {
    Vec3d highlightParticlePos = record.x.segment(viewer.particleId * 3, 3);
    std::vector<Vec3d> arr = {highlightParticlePos};
    renderFixedPoints(msSystem, clothShader, viewer, arr, COLOR_WOOD, false);
  }

  if (viewer.visualizeCollision) {
  {
    std::vector<Vec3d> points;
    std::vector<std::pair<Vec3d, Vec3d>> pointsAndDirs;
      for (const Simulation::PrimitiveCollisionInformation &info : record.collisionInfos.first.first) {
        pointsAndDirs.emplace_back(record.x.segment(info.particleId * 3, 3), info.normal);
        points.emplace_back(record.x.segment(info.particleId * 3, 3));
      }


    if (viewer.collisionShowNormal)
      renderNormals(msSystem, clothShader, viewer, msSystem->arrow2, pointsAndDirs, Vec3d(1, 0, 0));
      renderFixedPoints(msSystem, clothShader, viewer, points, COLOR_IBM_ULTRAMARINE40, false, false);

    }
  }


  if (viewer.visualizeSelfCollisionLayers) {
    std::vector<std::pair<Vec3d, Vec3d>> pointsAndDirs;
    std::vector<Vec3d> points;
    std::vector<std::pair<Vec3d, Vec3d>> lines;
    for (const Simulation::SelfCollisionInformation &info : record.collisionInfos.first.second) {
      if ((viewer.selfCollisionLayerVisId != -1) && (info.layerId != viewer.selfCollisionLayerVisId))
        continue;
      Vec3d p1 = record.x.segment(info.particleId1 * 3, 3);
      Vec3d p2 = record.x.segment(info.particleId2 * 3, 3);
      lines.emplace_back(p1,p2);
      pointsAndDirs.emplace_back(p1, info.normal);
      pointsAndDirs.emplace_back(p2, -info.normal);
      points.emplace_back(p1);
      points.emplace_back(p2);
    }
    renderFixedPoints(msSystem, clothShader, viewer, points, COLOR_IBM_MAGENTA50, false, false);
    if (viewer.collisionShowNormal)
      renderNormals(msSystem, clothShader, viewer, msSystem->arrow2, pointsAndDirs, Vec3d(1, 0, 0));
    renderLines(msSystem, simpleShader, viewer, lines, COLOR_IBM_MAGENTA50);

  }

  {
    std::vector<std::pair<Vec3d, Vec3d>> lines;
    Vec3d min = msSystem->sceneConfig.sceneBbox.min;
    Vec3d max = msSystem->sceneConfig.sceneBbox.max;
    Vec3d frontLeftDown = Vec3d(min[0], min[1], min[2]);
    Vec3d frontRightDown = Vec3d(max[0], min[1], min[2]);
    Vec3d frontRightUp = Vec3d(max[0], max[1], min[2]);
    Vec3d frontLeftUp = Vec3d(min[0], max[1], min[2]);
    lines.emplace_back(frontLeftDown, frontRightDown); // front face
    lines.emplace_back(frontLeftUp, frontRightUp);
    lines.emplace_back(frontLeftDown, frontLeftUp);
    lines.emplace_back(frontRightDown, frontRightUp);

    for (int i = 0; i < 4; i++) {
      lines.emplace_back(lines[i]); // back face
      lines[lines.size() - 1].first[2] = lines[lines.size() - 1].second[2] = max[2];
    }

    for (int i = 0; i < 2; i++) { //lines connecting front and back face
      Vec3d leftP = lines[i].first;
      Vec3d rightP = lines[i].second;
      lines.emplace_back(leftP, leftP);
      lines.emplace_back(rightP, rightP);
      lines[lines.size() - 1].second[2] = lines[lines.size() - 2].second[2] = max[2];
    }

    renderLines(msSystem, simpleShader, viewer, lines, Vec3d(1, 0, 0));

  }
  if (!isGroundTruthSystem) {
    if (viewer.renderAxis) {
      // render axis
      for (int i = 0; i < 3; i++) {
        std::vector<Vec3d> points;
        for (int dist = 0; dist < 15; dist++) {
          points.emplace_back(Vec3d((i == 0) * dist, (i == 1) * dist, (i == 2) * dist));
        }
        Vec3d color(i == 0, i == 1, i == 2);
        renderFixedPoints(msSystem, clothShader, viewer, points, color);

      }
    }

    if (viewer.renderAABB) {
      std::vector<std::pair<Vec3d, Vec3d>> lines;
      Vec3d min = msSystem->sceneConfig.sceneBbox.min;
      Vec3d max = msSystem->sceneConfig.sceneBbox.max;
      Vec3d frontLeftDown = Vec3d(min[0], min[1], min[2]);
      Vec3d frontRightDown = Vec3d(max[0], min[1], min[2]);
      Vec3d frontRightUp = Vec3d(max[0], max[1], min[2]);
      Vec3d frontLeftUp = Vec3d(min[0], max[1], min[2]);
      lines.emplace_back(frontLeftDown, frontRightDown); // front face
      lines.emplace_back(frontLeftUp, frontRightUp);
      lines.emplace_back(frontLeftDown, frontLeftUp);
      lines.emplace_back(frontRightDown, frontRightUp);

      for (int i = 0; i < 4; i++) {
        lines.emplace_back(lines[i]); // back face
        lines[lines.size() - 1].first[2] = lines[lines.size() - 1].second[2] = max[2];
      }

      for (int i = 0; i < 2; i++) { //lines connecting front and back face
        Vec3d leftP = lines[i].first;
        Vec3d rightP = lines[i].second;
        lines.emplace_back(leftP, leftP);
        lines.emplace_back(rightP, rightP);
        lines[lines.size() - 1].second[2] = lines[lines.size() - 2].second[2] = max[2];
      }

      renderLines(msSystem, simpleShader, viewer, lines, Vec3d(1, 0, 0));

    }

    Vec3d greenColor(0, 1, 0);
    if (viewer.renderPosPairs) {
      if (!msSystem->debugPointTargetPos.empty()) {
        std::vector<Simulation::CorresPondenceTargetInfo> &pairs = msSystem->debugPointTargetPos;
        std::vector<std::pair<Vec3d, Vec3d>> lines;
        std::vector<Vec3d> posesSock, posesFoot;

        for (Simulation::CorresPondenceTargetInfo &pair : pairs) {

          if (pair.frameIdx != recordIdx)
            continue;
          posesFoot.emplace_back(pair.targetPos);
          std::vector single = {pair.targetPos};
          renderFixedPoints(msSystem, clothShader, viewer, single, posToHashColor(pair.targetPos), false);

          for (int id : pair.particleIndices) {
            Vec3d sockPoint = record.x.segment(id * 3, 3);
            posesSock.emplace_back(sockPoint);
            lines.emplace_back(sockPoint, pair.targetPos);
          }
        }

        renderFixedPoints(msSystem, clothShader, viewer, posesSock, COLOR_WOOD, false);
        renderLines(msSystem, simpleShader, viewer, lines);
      }

      if (!msSystem->debugLoopPoints.empty()) {
        std::vector<Vec3d> points;
        for (int pIdx : msSystem->debugLoopPoints) {
          points.emplace_back(record.x.segment(pIdx * 3, 3));
        }
        renderFixedPoints(msSystem, clothShader, viewer, points, COLOR_IBM_MAGENTA50, false, false);

      }

      if (!msSystem->debugShapeTargetPos.empty()) {
        useClothShaderAndSetUniforms(msSystem, clothShader, viewer);
        clothShader->use();
         clothShader->setVec3("systemCenter",
                             glm::vec3(0,0,0));
        for (std::pair<int, VecXd> target : msSystem->debugShapeTargetPos) {
          if (target.first != recordIdx)
            continue;
           int numFields = 17;

            clothShader->setInt("renderMode", Viewer::RenderMode::WIREFRAME);

            nanogui::Color color(0.0, 0.0, 1.0, 1.0f);
            clothShader->setVec3("colorWireframe", color);


          int EMIT_SIZE = 200; // Number of Triangles in a single draw emit
          float vArr[3 * numFields * EMIT_SIZE];
          int curIdx = -1;

          for (int triIdx = 0; triIdx < msSystem->mesh.size(); triIdx++) {
            Triangle &m = msSystem->mesh[triIdx];
            double deformation = m.getDeformation(x_now);
            curIdx++;
            Particle *vs[] = {m.p0(), m.p1(), m.p2()};
            fillTriangleToVertexArr(m, vArr, curIdx * 3 * numFields, numFields, Vec3d(0, 0, 0), Vec3d(0, 0, 0), true);
            for (int vIdx = 0; vIdx < 3; vIdx++) {
              Vec3d pos = target.second.segment(vs[vIdx]->idx * 3, 3);
              Vec3d vel = v_now.segment(vs[vIdx]->idx * 3, 3);
              int startIdx = (curIdx * 3 + vIdx) * numFields;
              fillArrayWithVec3(vArr, pos, startIdx + 0); // 0-2
              fillArrayWithVec3(vArr, vel, startIdx + 5); // 5-7
              if (viewer.triangleId == triIdx) {
                fillArrayWithFloat(vArr, 1, startIdx + 8);
                fillArrayWithVec3(vArr, Vec3d(1, 0, 0), startIdx + 9);
              } else {
                fillArrayWithFloat(vArr, m.overrideColor, startIdx + 8);
              }
              fillArrayWithFloat(vArr, vertexCollision[vs[vIdx]->idx], startIdx + 15);
              fillArrayWithFloat(vArr, deformation, startIdx + 16);
            }
            int currentTriangleCount = curIdx + 1;
            if (((currentTriangleCount == EMIT_SIZE) || (triIdx + 1 == msSystem->mesh.size()))) {
              VertexArrayObject vertexVAO = VertexArrayObject((float *) vArr,
                                                              1 * 3 * currentTriangleCount,
                                                              numFields * sizeof(float));
              std::vector<int> fieldComponents = {3, 2, 3, 1, 3, 3, 1, 1};
              vertexArrAddAtrribInfo(vertexVAO, fieldComponents);
              vertexVAO.use();


              glDrawArrays(GL_TRIANGLES, 0, currentTriangleCount * 3);
              glBindVertexArray(0);


              curIdx = -1;
            }

          }


        }
      }

    }

   }


}
