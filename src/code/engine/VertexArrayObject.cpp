#include "VertexArrayObject.h"

VertexArrayObject::VertexArrayObject(float * dataSource, int vCount, int vStride)
{
	vertexCount = vCount;
	vertexStride = vStride;
  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);

  glGenBuffers(1, &VBO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, vertexStride * vertexCount, &dataSource[0], GL_STATIC_DRAW);


}


VertexArrayObject::VertexArrayObject(std::vector<glm::vec3> &array, int vCount, int vStride) {
  vertexCount = vCount;
  vertexStride = vStride;
  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);

  glGenBuffers(1, &VBO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, vertexStride * vertexCount, glm::value_ptr(array[0]), GL_STATIC_DRAW);


}


void VertexArrayObject::addVertexAttrib(int index, int componentCount, int offset) {
  //vertex shader attributes
  glVertexAttribPointer(index, componentCount, GL_FLOAT, GL_FALSE, vertexStride, (void *) offset);
  glEnableVertexAttribArray(index);
}
