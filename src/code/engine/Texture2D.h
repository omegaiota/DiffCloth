#ifndef OMEGAENGINE_TEXTURE2D_H
#define OMEGAENGINE_TEXTURE2D_H

#include "Macros.h"
class Texture2D
{
public:
	Texture2D(std::string name);
	~Texture2D() {};

	unsigned int getTexture() {
		return textureID;
	}

	void use(int slot) {
		glActiveTexture((GLenum) (GL_TEXTURE0 + slot));
		glBindTexture(GL_TEXTURE_2D, textureID);
	}
private:
	int width;
	int height;
	int nrComponents;
	int nChannels;
	std::string name;
	unsigned int textureID;

};


#endif