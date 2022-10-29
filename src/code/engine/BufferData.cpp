//
// Created by liyifei@csail.mit.edu on 1/6/19.
//

#include "BufferData.h"

const float BufferData::cubeVertices[];
const float BufferData::triangleVertices[];
//const float BufferData::triangleDownVertices[];
//const float BufferData::triangleUpVertices[];

VertexArrayObject BufferData::drawLightSetupVAO() {
  VertexArrayObject lightVAO = VertexArrayObject((float *) BufferData::cubeVertices, 36, 8 * sizeof(float));
  lightVAO.addVertexAttrib(0, 3, 0);
  return lightVAO;
}

void BufferData::drawRectangle(VertexArrayObject &vao, IndexArrayObject &iao, Shader shader) {
  shader.use();
  vao.use();
  iao.use();
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0); // 6 indices
}


VertexArrayObject BufferData::drawTriangleSetup(glm::vec3 a, glm::vec3 b, glm::vec3 c) {
  float vertices[] = {
          a[0], a[1], a[2],
          b[0], b[1], b[2],
          c[0], c[1], c[2],
  };

  VertexArrayObject obj = VertexArrayObject(vertices, 3, 3 * sizeof(float));
  obj.addVertexAttrib(0, 3, 0);
  return obj;
}

VertexArrayObject BufferData::drawTriangleSetup() {
  float vertices[] = {
          -0.5f, -0.5f, 0.0f,
          0.5f, -0.5f, 0.0f,
          0.0f, 0.5f, 0.0f
  };

  VertexArrayObject obj = VertexArrayObject(vertices, 3, 3 * sizeof(float));
  obj.addVertexAttrib(0, 3, 0);
  return obj;
}

VertexArrayObject BufferData::drawRectangleSetupVAO() {
  const float width = 0.3f;
  const float height = 0.3f;
  float vertices[] = {
          1.0f,  1.0f, 0.0f,  // top right
          1.0f, 1.0f-height, 0.0f,  // bottom right
          1.0f-width, 1.0f-height, 0.0f,  // bottom left
          1.0f-width,  1.0f, 0.0f   // top left
  };
  VertexArrayObject obj = (VertexArrayObject(vertices, 4, 3 * sizeof(float)));
  obj.addVertexAttrib(0, 3, 0);
  return obj;
}


VertexArrayObject BufferData::textureVAOSetup() {
  float vertices[] = {
          // positions          // colors           // textureID coords
          0.5f,  0.5f, 0.0f,   1.0f, 0.0f, 0.0f,   1.0f, 1.0f,   // top right
          0.5f, -0.5f, 0.0f,   0.0f, 1.0f, 0.0f,   1.0f, 0.0f,   // bottom right
          -0.5f, -0.5f, 0.0f,   0.0f, 0.0f, 1.0f,   0.0f, 0.0f,   // bottom left
          -0.5f,  0.5f, 0.0f,   1.0f, 1.0f, 0.0f,   0.0f, 1.0f    // top left
  };

  VertexArrayObject VAO = VertexArrayObject(vertices, 4, 8 * sizeof(float));
  VAO.addVertexAttrib(0, 3, 0);
  VAO.addVertexAttrib(1, 3, 3 * sizeof(float));
  VAO.addVertexAttrib(2, 2, 6 * sizeof(float));

  return VAO;
}


VertexArrayObject BufferData::drawcubeSetupVAO() {
  VertexArrayObject VAO = VertexArrayObject((float *) BufferData::cubeVertices , 36, 8 * sizeof(float));
  VAO.addVertexAttrib(0, 3, 0);
  VAO.addVertexAttrib(1, 2, 3 * sizeof(float));
  VAO.addVertexAttrib(2, 3, 5 * sizeof(float));

  return VAO;
}


IndexArrayObject BufferData::drawRectangleSetupIAO() {
  Index indices[] = {  // note that we start from 0!
          0, 1, 3,   // first triangle
          1, 2, 3    // second triangle
  };
  return  (IndexArrayObject(indices, 6));

}
void BufferData::drawTriangle(Shader triangleShader, VertexArrayObject& VAOobj) {
  triangleShader.use();
  triangleShader.setFloat("grassColor", 1.0f);
  VAOobj.use();
  glDrawArrays(GL_TRIANGLES, 0, 3);
}
