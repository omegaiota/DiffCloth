//
// Created by liyifei@csail.mit.edu on 7/28/18.
//

#ifndef OMEGAENGINE_SIMPLEDRAW_H
#define OMEGAENGINE_SIMPLEDRAW_H


#include "Shader.h"

class SimpleDraw {
public:
    SimpleDraw(Shader shaderProgram) : shaderProgram(shaderProgram) {
      setup();
    }
    void render();
private:
    unsigned int rectangleVAO;
    unsigned int triangleVAO;
    Shader shaderProgram;
    unsigned int EBO;
    void setup();

};


#endif //OMEGAENGINE_SIMPLEDRAW_H
