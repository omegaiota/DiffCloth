//
// Created by liyifei@csail.mit.edu on 5/9/21.
//

#ifndef OMEGAENGINE_RENDERLOOP_H
#define OMEGAENGINE_RENDERLOOP_H

#include "Renderer.h"
#include "Macros.h"
#include "Viewer.h"
#include "Shader.h"
#include "../simulation/Simulation.h"

class RenderLoop {
public:
    static Shader* clothShader;
    static Shader* simpleShader;
    static bool initialized;


    static Viewer viewer;
    static GLFWwindow *glfwWindow;

    static void
    renderRecordsForSystem(Simulation *system, std::vector<Simulation::ForwardInformation> &forwardRecords,
                           bool renderPosPairs = false, bool exitOnLastFrame = false,
                           std::string text = "You can set text for your convenience");

    ~RenderLoop() {
      delete clothShader;
      delete simpleShader;
    }

};




#endif //OMEGAENGINE_RENDERLOOP_H
