#include "IndexArrayObject.h"

IndexArrayObject::IndexArrayObject(Index * dataSource, int indexCount)
{
	glGenBuffers(1, &IAO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IAO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexCount * sizeof(int), &dataSource[0], GL_STATIC_DRAW);
}
