

#ifndef OMEGAENGINE_VERTEXARRAYOBJECT_H
#define OMEGAENGINE_VERTEXARRAYOBJECT_H

#include "Macros.h"
class VertexArrayObject {
public:
    VertexArrayObject() {
      VAO = -1;
      VBO = -1;
      vertexStride = -1;
      vertexCount = 0;
    }

    VertexArrayObject(float *dataSource, int vCount, int vStride);

    VertexArrayObject(std::vector<glm::vec3> &array, int vCount, int vStride);

    void addVertexAttrib(int index, int componentCount, int offset);

    ~VertexArrayObject() {
      glDeleteBuffers(1, &VBO);
      glDeleteVertexArrays(1, &VAO);
    };

    void use() {
      glBindVertexArray(VAO);
    }

public:
    unsigned int VAO;
    unsigned int VBO;
    unsigned int vertexStride;
    unsigned int vertexCount;

};

#endif 