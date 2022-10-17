
#ifndef OMEGAENGINE_INDEXARRAYOBJECT_H
#define OMEGAENGINE_INDEXARRAYOBJECT_H

#include "Macros.h"
class IndexArrayObject
{
public:
	IndexArrayObject(Index* dataSource, int indexCount);
	~IndexArrayObject() {};
	void use() {
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IAO);
	}

private:
	unsigned int IAO;
};


#endif 


