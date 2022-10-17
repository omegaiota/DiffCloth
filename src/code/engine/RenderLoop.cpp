//
// Created by liyifei@csail.mit.edu on 6/18/21.
//


#include "RenderLoop.h"


bool RenderLoop::initialized = false;
Shader* RenderLoop::clothShader = nullptr;
Shader* RenderLoop::simpleShader = nullptr;
Viewer RenderLoop::viewer = Viewer(true);
GLFWwindow* RenderLoop::glfwWindow = nullptr;
void RenderLoop::renderRecordsForSystem(Simulation *system, std::vector<Simulation::ForwardInformation> &forwardRecords,
                                        bool renderPosPairs, bool exitOnLastFrame, std::string text) {

  if (!initialized) {
    viewer.perStepTrajectoryVisualization = true;
    viewer.renderPosPairs = renderPosPairs;

    Simulation::SceneConfiguration &sceneConfig = system->sceneConfig;
    viewer.camera.setCamPos(glm::vec3(sceneConfig.camPos[0], sceneConfig.camPos[1], sceneConfig.camPos[2]));

    viewer.camera.setLookAt(system->getLookAtPos(system, sceneConfig));
    viewer.addSystems({system});
    glfwWindow = viewer.init();

    viewer.splineVisualization = true;
    viewer.perStepGradientVisualization = true;
    viewer.renderAxis = true;
    viewer.simulationInfoTextBox->setValue(text);

    clothShader = new Shader("clothVS", "clothFS");
    simpleShader = new Shader("simplebufferV", "simplebufferF");
    initialized = true;

  } else {
    glfwRestoreWindow(glfwWindow);
    glfwFocusWindow(glfwWindow);
  }








  int N = forwardRecords.size();
  int currentFrame = 0;
  bool escPressed = false;
  bool isLastFrame = false;
  bool spacePressed = false;

  int visible = glfwGetWindowAttrib(glfwWindow, GLFW_VISIBLE);
  while (!((exitOnLastFrame || escPressed) && isLastFrame)) {
    viewer.updateOnScreenInfos();
    viewer.processInput();

    if (glfwGetKey(glfwWindow, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
      escPressed = true;
    }

    if (glfwGetKey(glfwWindow, GLFW_KEY_SPACE) == GLFW_PRESS) {
      currentFrame = 0;
    }

    glClearColor(viewer.color_background[0], viewer.color_background[1], viewer.color_background[2], 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Renderer::stepAndRenderSystem(system, clothShader, simpleShader, viewer, forwardRecords, currentFrame);

    viewer.screen->drawWidgets();


    glfwSwapBuffers(glfwWindow);

    glfwPollEvents();

    currentFrame = std::min(currentFrame+1, N-1);
    isLastFrame = currentFrame == N-1;

  }
   glfwIconifyWindow(glfwWindow);
}