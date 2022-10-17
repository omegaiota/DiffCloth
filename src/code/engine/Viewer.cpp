//
// Created by liyifei@csail.mit.edu on 7/18/18.
//
#include "Viewer.h"
#include "UtilityFunctions.h"

GLFWwindow *Viewer::init() {
  if (!glfwInit()) {
      std::printf("ERROR: glfwInit failed\n");
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_DEPTH_BITS, 32);
  glfwWindowHint(GL_DEPTH_BITS, 32);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

  // enabled MSAA
  glfwWindowHint(GLFW_SAMPLES, 8);


  std::string windowName = "OmegaEngine";
  windowName.append(std::to_string(width)).append(" ").append(std::to_string(height));
  glfWwindow = glfwCreateWindow(width, height, windowName.c_str(), NULL, NULL);


  if (glfWwindow == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return nullptr;
  }
  glfwSetWindowUserPointer(glfWwindow, this);
  glfwSetCursorPosCallback(glfWwindow, [](GLFWwindow *window, double x, double y) {
      Viewer *viewer = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
      viewer->mouse_callback(x, y);
  });
  glfwMakeContextCurrent(glfWwindow);


  glfwSetInputMode(glfWwindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
  initalizeGLAD();
  setViewport();
  screen = new nanogui::Screen();
  screen->initialize(glfWwindow, true);
  setupNanogui();
  std::printf("%s\n", glGetString(GL_VERSION));

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


  glEnable(GL_MULTISAMPLE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);

  glDepthFunc(GL_LESS);
//  glDepthRange(0,1);
  GLenum err;
  if ((err = glGetError()) != GL_NO_ERROR) {
    std::string error;
    switch (err) {
      case GL_INVALID_ENUM:
        error = "INVALID_ENUM";
        break;
      case GL_INVALID_VALUE:
        error = "INVALID_VALUE";
        break;
      case GL_INVALID_OPERATION:
        error = "INVALID_OPERATION";
        break;
      case GL_STACK_OVERFLOW:
        error = "STACK_OVERFLOW";
        break;
      case GL_STACK_UNDERFLOW:
        error = "STACK_UNDERFLOW";
        break;
      case GL_OUT_OF_MEMORY:
        error = "OUT_OF_MEMORY";
        break;
      case GL_INVALID_FRAMEBUFFER_OPERATION:
        error = "INVALID_FRAMEBUFFER_OPERATION";
        break;
    }
    // Process/log the error.
    std::printf("glfw init error: %d %s\n", err, error.c_str());
  }
  return glfWwindow;
}


GLFWwindow *Viewer::newWindow() {
  if (!glfwInit()) {
    std::printf("ERROR: glfwInit failed\n");
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_DEPTH_BITS, 32);
  glfwWindowHint(GL_DEPTH_BITS, 32);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

  // enabled MSAA
  glfwWindowHint(GLFW_SAMPLES, 8);


  std::string windowName = "OmegaEngine";
  windowName.append(std::to_string(width)).append(" ").append(std::to_string(height));
  glfWwindow = glfwCreateWindow(width, height, windowName.c_str(), NULL, NULL);


  if (glfWwindow == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return nullptr;
  }
  glfwSetWindowUserPointer(glfWwindow, this);
  glfwSetCursorPosCallback(glfWwindow, [](GLFWwindow *window, double x, double y) {
      Viewer *viewer = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
      viewer->mouse_callback(x, y);
  });
  glfwMakeContextCurrent(glfWwindow);


  glfwSetInputMode(glfWwindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
  initalizeGLAD();
  setViewport();


  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


  glEnable(GL_MULTISAMPLE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);

  glDepthFunc(GL_LESS);
//  glDepthRange(0,1);
  GLenum err;
  if ((err = glGetError()) != GL_NO_ERROR) {
    std::string error;
    switch (err) {
      case GL_INVALID_ENUM:
        error = "INVALID_ENUM";
        break;
      case GL_INVALID_VALUE:
        error = "INVALID_VALUE";
        break;
      case GL_INVALID_OPERATION:
        error = "INVALID_OPERATION";
        break;
      case GL_STACK_OVERFLOW:
        error = "STACK_OVERFLOW";
        break;
      case GL_STACK_UNDERFLOW:
        error = "STACK_UNDERFLOW";
        break;
      case GL_OUT_OF_MEMORY:
        error = "OUT_OF_MEMORY";
        break;
      case GL_INVALID_FRAMEBUFFER_OPERATION:
        error = "INVALID_FRAMEBUFFER_OPERATION";
        break;
    }
    // Process/log the error.
    std::printf("glfw init error: %d %s\n", err, error.c_str());
  }
  return glfWwindow;
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);


  Viewer *viewer = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
  viewer->updateProjectionMatrix();
  viewer->width = width;
  viewer->height = height;

  viewer->updateCameraDependentWidgetPoses();
  viewer->screen->resizeCallbackEvent(width, height);


  if (!viewer->isSimpleViewer)
    viewer->updateCameraDependentWidgetPoses();

}

void Viewer::mouse_callback(double xpos, double ypos) {
  if (firstMouse) {
    lastX = xpos;
    lastY = ypos;
    firstMouse = false;
  }

  float xoffset = (float) xpos - lastX;
  float yoffset = lastY - (float) ypos; // reversed since y-coordinates range from bottom to top
  lastX = (float) xpos;
  lastY = (float) ypos;

  float sensitivity = 0.05f;
  xoffset *= sensitivity;
  yoffset *= sensitivity;


}

int Viewer::initalizeGLAD() {
  if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }
  glfwSetFramebufferSizeCallback(glfWwindow, framebuffer_size_callback);
  return 0;
}

void Viewer::setViewport() {
#ifdef USING_MACBOOK
  // TODO: I don't know why on my mac using width and height just renders to lower left corner of the screen..
  glViewport(0, 0, width * 2, height * 2);
#else
  glViewport(0, 0, width, height);
#endif
}


void Viewer::processInput() {

  bool cameraMoved = false;

  if (glfwGetKey(glfWwindow, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(glfWwindow, true);
  float moveDelta = 5.0;

  if (glfwGetKey(glfWwindow, GLFW_KEY_C) == GLFW_PRESS) {
    camera.addPitchAndYaw(0.0, moveDelta);
    cameraMoved = true;
  }
  if (glfwGetKey(glfWwindow, GLFW_KEY_V) == GLFW_PRESS) {
    camera.addPitchAndYaw(0.0, -moveDelta);
    cameraMoved = true;

  }
  if (glfwGetKey(glfWwindow, GLFW_KEY_W) == GLFW_PRESS) {
    camera.moveY(moveDelta * 0.01);
    cameraMoved = true;

  }

  if (glfwGetKey(glfWwindow, GLFW_KEY_S) == GLFW_PRESS) {
    camera.moveNegY(moveDelta * 0.01);
    cameraMoved = true;

  }

  if (glfwGetKey(glfWwindow, GLFW_KEY_A) == GLFW_PRESS) {
    cameraMoved = true;
    camera.moveNegX(moveDelta * 0.01);

  }

  if (glfwGetKey(glfWwindow, GLFW_KEY_D) == GLFW_PRESS) {
    cameraMoved = true;
    camera.moveX(moveDelta * 0.01);

  }


  if (glfwGetKey(glfWwindow, GLFW_KEY_Z) == GLFW_PRESS) {
    camera.zoom(moveDelta * 0.1);
    cameraMoved = true;

  }

  if (glfwGetKey(glfWwindow, GLFW_KEY_X) == GLFW_PRESS) {
    camera.zoom(-moveDelta * 0.1);
    cameraMoved = true;

  }

  // Rotation
  if (glfwGetKey(glfWwindow, GLFW_KEY_RIGHT) == GLFW_PRESS) {
    glm::vec3 dir = camera.getCamPos() - camera.lookAtPos;
    Vec3d dirEigen = Vec3d(dir[0], dir[1], dir[2]);

    Vec3d tangent = dirEigen.normalized().cross(Vec3d(0, 1, 0)) * 1.0;
    glm::vec3 move = camera.getCamPos() + glm::vec3(tangent[0], tangent[1], tangent[2]) * 0.5f;
    camera.setCamPos(move);
    cameraMoved = true;

  }

  if (glfwGetKey(glfWwindow, GLFW_KEY_LEFT) == GLFW_PRESS) {
    glm::vec3 dir = camera.getCamPos() - camera.lookAtPos;
    Vec3d dirEigen = Vec3d(dir[0], dir[1], dir[2]);

    Vec3d tangent = dirEigen.normalized().cross(Vec3d(0, 1, 0)) * 1.0;
    glm::vec3 move = camera.getCamPos() - glm::vec3(tangent[0], tangent[1], tangent[2]) * 0.5f;
    camera.setCamPos(move);
    cameraMoved = true;

  }


  if (glfwGetKey(glfWwindow, GLFW_KEY_UP) == GLFW_PRESS) {
    cameraMoved = true;
    camera.moveLookAtDir(moveDelta);

  }

  if (glfwGetKey(glfWwindow, GLFW_KEY_DOWN) == GLFW_PRESS) {
    cameraMoved = true;
    camera.moveLookAtDir(-moveDelta);

  }


//  if (glfwGetKey(glfWwindow, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
//    double xpos, ypos;
//    glfwGetCursorPos(glfWwindow, &xpos, &ypos);
//  }


  if (cameraMoved) {
    updateCameraDependentWidgetPoses();
  }
}


void Viewer::startMainLoop() {

  while (!glfwWindowShouldClose(glfWwindow)) {
    //input
    processInput();

    //rendering commands
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    //check and call events and swap buffers
    glfwSwapBuffers(glfWwindow);
    glfwPollEvents();
  }

  glfwTerminate();
}



std::vector<nanogui::Vector2i> Viewer::modelToScreen(std::vector<Vec3d> &poses) {
  mat4 Vmax = camera.getViewMat();
  mat4 Pmax = getProjectionMat();
  mat4 PV = Pmax * Vmax;
  std::vector<nanogui::Vector2i> res;
  for (const Vec3d &s : poses) {
    glm::vec4 center = glm::vec4(s[0], s[1], s[2], 1);
    glm::vec4 transformed = PV * center;
    double x = (transformed.x / transformed.w);
    double y = (transformed.y / transformed.w);
    double z = (transformed.z / transformed.w);
    res.emplace_back(nanogui::Vector2i((int) ((x * 0.5 + 0.5) * width), (int) (((-y) * 0.5 + 0.5) * height)));
  }

  return res;
}

void Viewer::updateOnScreenInfos() {

  // Update FPS
  float currentFrame = glfwGetTime();
  float deltaTime = currentFrame - lastFrame;
  lastFrame = currentFrame;
  frameNum++;
  if (currentFrame - lastTime >= 1.0f) {
    // Update simulated frames

    if (!isSimpleViewer)
        fpsTextBox->setValue("FPS: " + std::to_string(frameNum));
    frameNum = 0;
    lastTime = currentFrame;
  }
  // TODO: move to other plaeces
  if (isforward && (!isSimpleViewer)) {
    simulationInfoTextBox->setValue(
            "Total Timestep: " + std::to_string(simSystems[0]->forwardRecords.size()));
  }


}

void Viewer::updateCameraDependentWidgetPoses() {
//  std::vector<nanogui::Vector2i> res = modelToScreen(symPosCenters);
//  for (int i = 0; i < simSystems.size(); i++) {
//    nanogui::TextBox *textBox = simSystemNameBoard[i];
//    textBox->setPosition(res[i]);
//  }
    if (!isSimpleViewer) {
      std::vector<nanogui::Vector2i> res = modelToScreen(symPosCenters);
      for (int i = 0; i < simSystems.size(); i++) {
        nanogui::TextBox *textBox = simSystemNameBoard[i];
        textBox->setPosition(res[i]);
      }


    }




}

void Viewer::resetAllSimulation() {
  std::printf("Reset All Simulation Called\n");
  for (Simulation *s : simSystems) { s->resetSystem(); }
  playBackId = 0;
  forwardFrameSliderText.slider->setValue(0);
  forwardFrameSliderText.slider->callback()(forwardFrameSliderText.slider->value());
}

void Viewer::setupNanogui() {
  using namespace nanogui;
  if (isSimpleViewer) {
    simulationInfoTextBox = new nanogui::TextBox(screen);
    simulationInfoTextBox->setValue("Here is a textbox for use blahblahblahblahblahblah");
    simulationInfoTextBox->setFixedSize(nanogui::Vector2i(600, 30));
    simulationInfoTextBox->setFontSize(15);
    simulationInfoTextBox->setAlignment(nanogui::TextBox::Alignment::Left);

    simulationInfoTextBox->setPosition(Vec2i(30, 30));
    simulationInfoTextBox->setEnabled(true);
    simulationInfoTextBox->setVisible(true);
  } else {
    for (Simulation *sim : simSystems) {
      symPosCenters.emplace_back(sim->systemCenter + sim->sphere5.center + Vec3d(0, 21, 0));
    }

    std::vector<Vector2i> res = modelToScreen(symPosCenters);
    SettingsControl = new nanogui::FormHelper(screen);
    addSceneControlWidgets();
    addReinitWidgets();
    addCameraControlWidgets();
    addTextBoxWidgets();
    addForwardSimulationControlWidets();
    backwardInfo = simSystems[0]->backwardInfoDefault;
    backwardInfo.dL_dsplines = finitediffGradientInfo.dL_dsplines = simSystems[0]->backwardInfoDefault.dL_dsplines;
    addForwardSimulationInfoWidets("Forward Info", forwardInfoWidgets, forwardInfo, forwardInfoWindow, 300);
    addBackwardSimulationInfoWidets("Backward Gradient Log", gradientInfoWidgets, backwardInfo, backwardSimInfoWindow,
                                    500, false);
    addBackwardSimulationInfoWidets("Finite Difference Gradient", finiteDiffInfoWidgets, finitediffGradientInfo,
                                    backwardSimFiniteDiffInfoWindow, 900, true);
    addBackwardSimulationControlWidets();

    addParticleInfoWidgets();
    screen->setVisible(true);
    screen->performLayout();


    glfwSetCursorPosCallback(glfWwindow,
                             [](GLFWwindow *window, double x, double y) {
                                 Viewer *viewer = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
                                 viewer->screen->cursorPosCallbackEvent(x, y);
                             }
    );

    glfwSetMouseButtonCallback(glfWwindow,
                               [](GLFWwindow *window, int button, int action, int modifiers) {
                                   Viewer *viewer = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
                                   viewer->screen->mouseButtonCallbackEvent(button, action, modifiers);
                               }
    );

    glfwSetKeyCallback(glfWwindow,
                       [](GLFWwindow *window, int key, int scancode, int action, int mods) {
                           Viewer *viewer = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
                           viewer->screen->keyCallbackEvent(key, scancode, action, mods);
                           viewer->updateCameraDependentWidgetPoses();

                       }
    );

    glfwSetCharCallback(glfWwindow,
                        [](GLFWwindow *window, unsigned int codepoint) {
                            Viewer *viewer = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
                            viewer->screen->charCallbackEvent(codepoint);
                        }
    );

    glfwSetDropCallback(glfWwindow,
                        [](GLFWwindow *window, int count, const char **filenames) {
                            Viewer *viewer = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
                            viewer->screen->dropCallbackEvent(count, filenames);
                        }
    );

    glfwSetScrollCallback(glfWwindow,
                          [](GLFWwindow *window, double x, double y) {
                              Viewer *viewer = static_cast<Viewer *>(glfwGetWindowUserPointer(window));
                              viewer->screen->scrollCallbackEvent(x, y);
                          }
    );

    std::printf("finished...");

  }
}

void floatToIdxAndText(double val, int totalSize, int &idx, std::string &text) {
  idx = (totalSize <= 1) ? 0 : std::round(val * (totalSize - 1.0));
  text = std::to_string(idx + 1) + "/" + std::to_string(totalSize);
//  std::printf("float %.2f withtotal %d converted to idx %d and text %s\n", val, totalSize, idx, text.c_str());

}

// selectedIdx = 0, val = 0, size = 1

void idxToFloatAndText(int idx, int totalSize, double &floatVal, std::string &text) {
  // idx:[0,totalSize-1]
  // floatVal: [0,1]
  // textVal: [1,totalSize] = idx+1
  floatVal = (totalSize <= 1) ? 0 : (idx / (totalSize - 1.0));
  text = std::to_string(idx + 1) + "/" + std::to_string(totalSize);
//  std::printf("int %.2f withtotal %d converted to val %d and text %s\n", idx, totalSize, floatVal, text.c_str());


}


void Viewer::addTextBoxWidgets() {
  int textBoxHeight = 25;
  int textBoxWidth = 60;
  fpsTextBox = new nanogui::TextBox(screen);
  fpsTextBox->setValue("0");
  fpsTextBox->setFixedSize(nanogui::Vector2i(textBoxWidth, textBoxHeight));
  fpsTextBox->setFontSize(15);
  fpsTextBox->setPosition(Vec2i(30, 30));
  fpsTextBox->setAlignment(nanogui::TextBox::Alignment::Left);
  fpsTextBox->setEditable(false);


  infoTextBox = new nanogui::TextBox(forwardSimControlWindow);
  infoTextBox->setValue("0");
  infoTextBox->setFixedSize(nanogui::Vector2i(textBoxWidth, textBoxHeight));
  infoTextBox->setFontSize(15);
  infoTextBox->setAlignment(nanogui::TextBox::Alignment::Left);
  infoTextBox->setEditable(false);
  infoTextBox->setPosition(Vec2i(50, 100));
}

template<typename T>
void formWidgetSetSpinnableWithIncrement(nanogui::detail::FormWidget<T> *widget, T inc) {
  widget->setSpinnable(true);
  widget->setValueIncrement(inc);
}

void Viewer::addSceneControlWidgets() {
  mainConfigWindow = SettingsControl->addWindow(Eigen::Vector2i(30, 0), "Cloth Simulation");
  SettingsControl->addGroup("Experiment Name");
  {
    nanogui::TextBox* nameBox = new nanogui::TextBox(mainConfigWindow);
    nameBox->setEditable(true);
    nameBox->setFontSize(12);
    nameBox->setCallback([=](const std::string& s) {return true;});
    SettingsControl->addWidget("name", nameBox);
  }

  SettingsControl->addGroup("Scene Control");
  if (simSystems.size() > 0) {
    std::vector<Primitive *> prims = simSystems[0]->primitives;
    for (int idx = 0; idx < prims.size(); idx++) {
      Primitive *p = prims[idx];
      nanogui::Widget *panel = new nanogui::Widget(mainConfigWindow);
      panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                     nanogui::Alignment::Middle, 0, 10));
      SettingsControl->addWidget(p->primitiveTypeStrings[p->type].c_str(), panel);

      SettingsControl->addVariable<bool>("On ",
                                         [=](const bool &v) {
                                             for (int i = 0; i < simSystems.size(); i++) {
                                               simSystems[i]->primitives[idx]->setEnabled(v);
                                             }
                                         },
                                         [=]() -> bool {
                                             return simSystems[0]->primitives[idx]->isEnabled;
                                         }, panel
      );

      SettingsControl->addVariable<bool>("Vis",
                                         [=](const bool &v) {
                                             for (int i = 0; i < simSystems.size(); i++) {
                                               simSystems[i]->primitives[idx]->setSimulationEnabled(v);
                                             }
                                         },
                                         [=]() -> bool {
                                             return simSystems[0]->primitives[idx]->simulationEnabled;
                                         }, panel
      );

      nanogui::detail::FormWidget<float> *muControl = SettingsControl->addVariable<float>("mu",
                                                                                          [=](const float &v) {
                                                                                              for (int i = 0; i <
                                                                                                              simSystems.size(); i++) {
                                                                                                simSystems[i]->primitives[idx]->setMu(
                                                                                                        v);
                                                                                              }
                                                                                          },
                                                                                          [=]() -> float {
                                                                                              return simSystems[0]->primitives[idx]->mu;
                                                                                          }, panel);
      formWidgetSetSpinnableWithIncrement(muControl, 0.1f);


    }
  }


  SettingsControl->addGroup("Simulation Control");
  nanogui::Widget *panel = new nanogui::Widget(mainConfigWindow);
  panel->setLayout(new nanogui::GridLayout(nanogui::Orientation::Horizontal, 6, nanogui::Alignment::Middle,
                                           0, 0));
  SettingsControl->addWidget("", panel);

  SettingsControl->addVariable<bool>("gravity",
                                     [&](const bool &v) { Simulation::gravityEnabled = v; },
                                     [&]() -> bool { return Simulation::gravityEnabled; }, panel
  );

  SettingsControl->addVariable<bool>("wind",
                                     [&](const bool &v) { Simulation::windEnabled = v; },
                                     [&]() -> bool { return Simulation::windEnabled; }, panel
  );

  SettingsControl->addVariable<bool>("bending",
                                     [&](const bool &v) { Simulation::bendingEnabled = v; },
                                     [&]() -> bool { return Simulation::bendingEnabled; }, panel
  );
  SettingsControl->addVariable<bool>("contact",
                                     [&](const bool &v) { Simulation::contactEnabled = v; },
                                     [&]() -> bool { return Simulation::contactEnabled; }, panel
  );
  SettingsControl->addVariable<bool>("self-collision",
                                     [&](const bool &v) { Simulation::selfcollisionEnabled = v; },
                                     [&]() -> bool { return Simulation::selfcollisionEnabled; }, panel
  );
  SettingsControl->addGroup("Rendering Control");

  panel = new nanogui::Widget(mainConfigWindow);
  panel->setLayout(new nanogui::GridLayout(nanogui::Orientation::Horizontal, 6, nanogui::Alignment::Middle,
                                           0, 0));
  SettingsControl->addWidget("", panel);

  SettingsControl->addVariable<bool>("collision",
                                     [&](const bool &v) {
                                         visualizeCollision = v;
                                     },
                                     [&]() -> bool {
                                         return visualizeCollision;
                                     }, panel
  );


  SettingsControl->addVariable<bool>("grid ",
                                     [&](const bool &v) {
                                         renderGrid = v;
                                     },
                                     [&]() -> bool {
                                         return renderGrid;
                                     }, panel
  );

  SettingsControl->addVariable<bool>("AABB ",
                                     [&](const bool &v) {
                                         renderAABB = v;
                                     },
                                     [&]() -> bool {
                                         return renderAABB;
                                     }, panel
  );


  SettingsControl->addVariable<bool>("spline",
                                     [&](const bool &v) {
                                         splineVisualization = v;
                                     },
                                     [&]() -> bool {
                                         return splineVisualization;
                                     }, panel
  );

  SettingsControl->addVariable<bool>("axis",
                                     [&](const bool &v) {
                                         renderAxis = v;
                                     },
                                     [&]() -> bool {
                                         return renderAxis;
                                     }, panel
  );

  SettingsControl->addVariable<bool>("lights",
                                     [&](const bool &v) {
                                         renderLights = v;
                                     },
                                     [&]() -> bool {
                                         return renderLights;
                                     }, panel
  );

  SettingsControl->addVariable<bool>("pairs",
                                     [&](const bool &v) {
                                         renderPosPairs = v;
                                     },
                                     [&]() -> bool {
                                         return renderPosPairs;
                                     }, panel
  );

  SettingsControl->addVariable<bool>("self-collisionLayer",
                                     [&](const bool &v) {
                                         visualizeSelfCollisionLayers = v;
                                     },
                                     [&]() -> bool {
                                         return visualizeSelfCollisionLayers;
                                     }, panel
  );
  SettingsControl->addVariable<bool>("collisionNormal",
                                     [&](const bool &v) {
                                         collisionShowNormal = v;
                                     },
                                     [&]() -> bool {
                                         return collisionShowNormal;
                                     }, panel
  );

  nanogui::detail::FormWidget<int> * layerIdWidget = SettingsControl->addVariable<int>("layerId",
                                    [&](const int &v) {selfCollisionLayerVisId = v;},
                                    [&]() -> int { return selfCollisionLayerVisId;}, panel);
  layerIdWidget->setMinValue(-1);
  formWidgetSetSpinnableWithIncrement(layerIdWidget, 1);

  SettingsControl->addVariable("Render Mode", myRendermode, true)->setItems(renderModeStrings);
  SettingsControl->addVariable("WindConfig", simSystems[0]->sceneConfig.windConfig, true)->setItems(windConfigStrings);

  panel = new  nanogui::Widget(mainConfigWindow);
  panel->setLayout(new nanogui::GridLayout(nanogui::Orientation::Horizontal, 4, nanogui::Alignment::Middle,0, 0));

  SettingsControl->addWidget("", panel);

  timeStepSetter = SettingsControl->addVariable<int>("timeStep",
                                                     [&](const int &v) {
                                                         for (Simulation *s : simSystems) {
                                                           s->sceneConfig.timeStep = 1.0 / v;
                                                           s->initializePrefactoredMatrices(); }
                                                     },
                                                     [&]() -> int {

                                                         return (int) (1.0 /
                                                                      simSystems[0]->sceneConfig.timeStep);
                                                     }, panel
  );
  timeStepSetter->setFixedWidth(60);

  precisionSetter = SettingsControl->addVariable<int>("forward_thresh",
                                                      [&](const int &v) {
                                                          Simulation::forwardConvergenceThreshold =
                                                                  std::pow(10, -v);
                                                      },
                                                      [&]() -> int {

                                                          return (int) (-std::log10(
                                                                  Simulation::forwardConvergenceThreshold));
                                                      }, panel);
  precisionSetter->setFixedWidth(60);
  formWidgetSetSpinnableWithIncrement(precisionSetter, 1);

  formWidgetSetSpinnableWithIncrement(timeStepSetter, 5);

  stepLimit = SettingsControl->addVariable<int>("stepNum",
                                                [&](const int &v) {
                                                    for (Simulation* s : simSystems) {
                                                      s->sceneConfig.stepNum = v;
                                                    }

                                                },
                                                [&]() -> int {

                                                    return simSystems[0]->sceneConfig.stepNum;
                                                }, panel
  );
  stepLimit->setFixedWidth(60);
  formWidgetSetSpinnableWithIncrement(stepLimit, 5);




  SettingsControl->addVariable("custom color", color_custom)
          ->setFinalCallback([](const nanogui::Color &c) {
              std::cout << "color_custom: ["
                        << c.r() << ", "
                        << c.g() << ", "
                        << c.b() << ", "
                        << c.w() << "]" << std::endl;
          });

  SettingsControl->addVariable("color_2sides_1", color_doubleside_1)
          ->setFinalCallback([](const nanogui::Color &c) {
              std::cout << "color_doubleside_1: ["
                        << c.r() << ", "
                        << c.g() << ", "
                        << c.b() << ", "
                        << c.w() << "]" << std::endl;
          });

  SettingsControl->addVariable("color_2sides_2", color_doubleside_2)
          ->setFinalCallback([](const nanogui::Color &c) {
              std::cout << "color_doubleside_2: ["
                        << c.r() << ", "
                        << c.g() << ", "
                        << c.b() << ", "
                        << c.w() << "]" << std::endl;
          });

  SettingsControl->addVariable("color_background", color_background)
          ->setFinalCallback([](const nanogui::Color &c) {
              std::cout << "color_background: ["
                        << c.r() << ", "
                        << c.g() << ", "
                        << c.b() << ", "
                        << c.w() << "]" << std::endl;
          });
  nanogui::detail::FormWidget<float> *wireframe = SettingsControl->addVariable<float>("wireframe thickness",
                                                                                        [&](const float &v) {
                                                                                             wireframeThickness = v;
                                                                                        },
                                                                                        [&]() -> float {

                                                                                            return wireframeThickness;
                                                                                        }
  );
  formWidgetSetSpinnableWithIncrement(wireframe, 0.01f);
  nanogui::detail::FormWidget<float> *windControl = SettingsControl->addVariable<float>("wind size",
                                                                                        [&](const float &v) {
                                                                                            Simulation::windNorm = v;
                                                                                        },
                                                                                        [&]() -> float {

                                                                                            return Simulation::windNorm;
                                                                                        }
  );

  formWidgetSetSpinnableWithIncrement(windControl, 0.01f);


  windControl = SettingsControl->addVariable<float>("wind frequency", [&](const float &v) {
                                                        Simulation::windFrequency = v;
                                                    },
                                                    [&]() -> float {

                                                        return Simulation::windFrequency;
                                                    }
  );

  formWidgetSetSpinnableWithIncrement(windControl, 0.1f);

  windControl = SettingsControl->addVariable<float>("wind phase", [&](const float &v) {
                                                        Simulation::windPhase = v;
                                                    },
                                                    [&]() -> float {
                                                        return Simulation::windPhase;
                                                    }
  );

  formWidgetSetSpinnableWithIncrement(windControl, 0.1f);


}

void Viewer::addCameraControlWidgets() {
  using namespace nanogui;
  SettingsControl->addGroup("Camera");
  Widget *panel = new Widget(reinitWindow);
  panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                 Alignment::Middle, 0, 0));
  SettingsControl->addWidget("CameraPos", panel);
  for (int i = 0; i < 3; i++) {

    nanogui::detail::FormWidget<float> *widget = SettingsControl->addVariable<float>("",
                                                                                     [=](const float &v) {
                                                                                         glm::vec3 newCamPos = camera.getCamPos();
                                                                                         newCamPos[i] = v;
                                                                                         camera.setCamPos(newCamPos);
                                                                                         updateCameraDependentWidgetPoses();
                                                                                     },
                                                                                     [=]() -> float {
                                                                                         return camera.getCamPos()[i];
                                                                                     }, panel);


    cameraWidgets.push_back(widget);
    widget->setFixedWidth(60);
    formWidgetSetSpinnableWithIncrement(widget, 1.0f);
  }

  nanogui::Button* updateButton = new Button(panel, "update");
  updateButton->setFontSize(12);
  updateButton->setCallback([&](){
      for ( int dim = 0; dim < 3; dim++) {
        cameraWidgets[dim]->setValue(camera.getCamPos()[dim]);
      }
  });

  panel = new Widget(reinitWindow);
  panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                 Alignment::Middle, 0, 0));
  SettingsControl->addWidget("FocusPoint", panel);
  for (int i = 0; i < 3; i++) {

    nanogui::detail::FormWidget<float> *widget = SettingsControl->addVariable<float>("",
                                                                                     [=](const float &v) {
                                                                                         glm::vec3 newCamPos = camera.lookAtPos;
                                                                                         newCamPos[i] = v;
                                                                                         camera.setLookAt(
                                                                                                 newCamPos);

                                                                                     },
                                                                                     [=]() -> float {
                                                                                         return camera.lookAtPos[i];
                                                                                     }, panel);

    cameraWidgets.push_back(widget);
    widget->setFixedWidth(60);

    formWidgetSetSpinnableWithIncrement(widget, 1.0f);
  }
  updateButton = new Button(panel, "update");
  updateButton->setFontSize(12);
  updateButton->setCallback([&]() {
      for (int dim = 3; dim < 6; dim++) {
        cameraWidgets[dim]->setValue(camera.lookAtPos[dim - 3]);
      }
      std::printf("Currrent camLookAtPos: %.3f %.3f %.3f\nCurrrent camPos: %.3f %.3f %.3f\n",
                  camera.lookAtPos[0], camera.lookAtPos[1], camera.lookAtPos[2],
                  camera.getCamPos()[0], camera.getCamPos()[1], camera.getCamPos()[2]
      );
  });

  SettingsControl->addButton("Set to SceneConfig", [&]() {
      camera.setCamPos(toGlm(simSystems[0]->sceneConfig.camPos));
      camera.setLookAt(Simulation::getLookAtPos(simSystems[0], simSystems[0]->sceneConfig));
  });

}

void Viewer::addSystems(std::vector<Simulation *> systems) {
  simSystems = systems;
}

void Viewer::addSystem(Simulation *system) {
  simSystems.push_back(system);
}

void Viewer::backwardSliderChecker() {
  int backwardRecordNum = simSystems[0]->backwardOptimizationRecords.size();


  if (isplaybackSimulationOn) {
    if (backwardPlayGroundtruthSimulationButton->enabled())
      backwardPlayGroundtruthSimulationButton->setEnabled(false);
    if (backwardPlayRecordButton->enabled())
      backwardPlayRecordButton->setEnabled(false);
    if (backwardRecordSliderText.slider->enabled())
      backwardRecordSliderText.slider->setEnabled(false);
  } else {

    if (backwardRecordNum == 0) {
      backwardPlayRecordButton->setEnabled(false);
      if (forwardFrameSliderText.textbox->enabled()) {
        forwardFrameSliderText.textbox->setEnabled(false);
        backwardRecordSliderText.slider->setEnabled(false);
        backwardRecordSliderText.slider->setValue(0);
        backwardRecordSliderText.idxWidget->setEnabled(false);
      }
    } else {
      if (!backwardPlayRecordButton->enabled()) {
        backwardPlayRecordButton->setEnabled(true);
        backwardRecordSliderText.slider->setEnabled(false);
        backwardRecordSliderText.idxWidget->setEnabled(true);

      }

      // update textbox because new records will come in

      if (backwardSelectedRecordId != -1) {
        std::string text;
        double val;
        idxToFloatAndText(backwardSelectedRecordId, backwardRecordNum, val, text);
        if (text != backwardRecordSliderText.textbox->value()) {
          backwardRecordSliderText.slider->setValue(val);
          backwardRecordSliderText.slider->finalCallback()(val);
        }


      } else {
        backwardRecordSliderText.slider->setEnabled(true);
        backwardPlayRecordButton->setEnabled(true);
      }
    }

    if (simSystems[0]->groundTruthForwardRecords.size() == simSystems[0]->sceneConfig.stepNum + 1) {
      backwardPlayGroundtruthSimulationButton->setEnabled(true);
    }
  }


}

void Viewer::checkBackwardPlaybackFinishedCallback() {
  bool canRender = (playGroundtruth) || (playLineSearch) ||  ((!playGroundtruth) && (!playLineSearch) && (backwardSelectedRecordId >= 0) &&
                                         (backwardSelectedRecordId <
                                          simSystems[0]->backwardOptimizationRecords.size()));
  if (!canRender)
    return;


  int totalSize = simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].first.size();
  if (playGroundtruth) {
    totalSize = simSystems[0]->groundTruthForwardRecords.size();
  } else if (playLineSearch) {
    totalSize = simSystems[0]->linesearchRecords.size();
  }

  if (isplaybackSimulationOn) { // play and advance
    if (playBackId < totalSize - 1) {
      playBackId++;
      backwardFrameSliderText.textbox->setValue(std::to_string(playBackId + 1) + "/" + std::to_string(totalSize));
      backwardFrameSliderText.slider->setValue(totalSize <= 1 ? 0.0 : playBackId / (totalSize - 1.0));
    } else {
      isplaybackSimulationOn = false;
    }
  }

}

void Viewer::checkPlaybackFinishedCallback() {
  int totalSize = simSystems[0]->forwardRecords.size();
  forwardFrameSliderText.idxWidget->setMaxValue(totalSize - 1);

  if (simSystems[0]->explosionEncountered) {
    isplaybackSimulationOn = false;
    isSimulationOn = false;
    forwardFrameSliderText.slider->setEnabled(true);
    forwardStopSimulationButton->setEnabled(true);
    forwardStartSimulationButton->setEnabled(true);
    return;
  }
  if (playBackId < totalSize - 1) {
    playBackId++;
    forwardFrameSliderText.slider->setValue((playBackId + 1.0) / totalSize);
    forwardFrameSliderText.textbox->setValue(
            std::to_string(playBackId + 1) + "/" + std::to_string(simSystems[0]->forwardRecords.size()));

    return;
  } else {
    isplaybackSimulationOn = false;
    forwardFrameSliderText.slider->setEnabled(true);
    forwardStopSimulationButton->setEnabled(true);
    forwardStartSimulationButton->setEnabled(true);
  }
}

double roundToDigits(float value, int digits) {
  int a = std::pow(10, digits);
  return std::round(value * a) / a;
};

void Viewer::addForwardSimulationInfoWidets(std::string title, ForwardInfoWidgets &infoWidgets,
                                            Simulation::ForwardInformation &forwardInfo,
                                            nanogui::ref<nanogui::Window> window, int widthOffset) {
  using namespace nanogui;
  window = SettingsControl->addWindow(Eigen::Vector2i(width - widthOffset, height - 500),
                                      title);
  forwardInfo = {};
  infoWidgets.convergeIter = SettingsControl->addVariable("converge iter",
                                                          forwardInfo.convergeIter);
  infoWidgets.accumulateIter = SettingsControl->addVariable("accumulate iter",
                                                            forwardInfo.cumulateIter);

  infoWidgets.selfCollisionNumber = SettingsControl->addVariable("self-collisions",
                                                                 forwardInfo.convergeIter);
  infoWidgets.externalCollisionNumber = SettingsControl->addVariable("external-collisions",
                                                                     forwardInfo.convergeIter);
  infoWidgets.avgDeformation = SettingsControl->addVariable("avgDeformation",
                                                                     forwardInfo.avgDeformation);

  infoWidgets.selfCollisionNumber->setEditable(false);
  infoWidgets.externalCollisionNumber->setEditable(false);
  infoWidgets.accumulateIter->setEditable(false);
  infoWidgets.convergeIter->setEditable(false);
  infoWidgets.avgDeformation->setEditable(false);

  infoWidgets.convergeIter->setFixedWidth(60);
  infoWidgets.accumulateIter->setFixedWidth(60);
  infoWidgets.selfCollisionNumber->setFixedWidth(60);
  infoWidgets.avgDeformation->setFixedWidth(60);
  infoWidgets.externalCollisionNumber->setFixedWidth(60);
  infoWidgets.converged = SettingsControl->addVariable("converged",
                                                       forwardInfo.converged);


  panel = new Widget(window);
  panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                 Alignment::Fill, 0, 0));
  infoWidgets.totalTime = SettingsControl->addVariable("totalRuntime[s]",
                                                       forwardInfo.avgDeformation);
  infoWidgets.iterTime = SettingsControl->addVariable("iterRuntime[s]",
                                                      forwardInfo.avgDeformation);
  SettingsControl->addWidget("iterTimer[ms]", panel);

  infoWidgets.timerReport = new TextBox(panel, "empty");
  infoWidgets.timerReport->setFontSize(15);
  infoWidgets.timerReport->setFixedWidth(150);
  infoWidgets.timerReport->setEditable(false);

  panel = new Widget(window);
  panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                 Alignment::Fill, 0, 0));
  SettingsControl->addWidget("accumTimer[ms]", panel);

  infoWidgets.accumTimerReport = new TextBox(panel, "empty");
  infoWidgets.accumTimerReport->setFontSize(15);
  infoWidgets.accumTimerReport->setFixedWidth(150);
  infoWidgets.accumTimerReport->setEditable(false);


  infoWidgets.timerReportIdxControl = SettingsControl->addVariable<int>("idx",
                                                                        [&](const int &v) {
                                                                            if (v !=
                                                                                std::clamp(v, 0,
                                                                                           (int) forwardInfo.timer.timeMicroseconds.size() -
                                                                                           1)) {
                                                                              return;
                                                                            }
                                                                            infoWidgets.timerId = v;
                                                                            std::string label = forwardInfo.timer.timeMicroseconds[v].first;
                                                                            long long duration = forwardInfo.timer.timeMicroseconds[v].second;
                                                                            long long durationTotal = forwardInfo.timer.totalMicroseconds;
                                                                            int percentage = (duration * 100.0 /
                                                                                              durationTotal);

                                                                            std::ostringstream streamObj;
                                                                            streamObj << std::setprecision(2)
                                                                                      << std::fixed;
                                                                            streamObj
                                                                                    << roundToDigits(duration / 1000.0,
                                                                                                     2);
                                                                            std::string durationStr = streamObj.str();
                                                                            infoWidgets.timerReport->setValue(
                                                                                    label + ":" + durationStr + "|" +
                                                                                    std::to_string(percentage) + "%");

                                                                            double accumDuration = forwardInfo.accumTimer[v].second;
                                                                            double accumTotal = 0;
                                                                            for (std::pair<std::string, long long> &item : forwardInfo.accumTimer) {
                                                                              if ((item.first != "solveIterative") && (item.first != "solveDirect")) {
                                                                                accumTotal += item.second;
                                                                              }
                                                                            }
                                                                            int percentageAccum =(accumDuration / accumTotal) * 100;
                                                                            streamObj.str("");
                                                                            streamObj.clear();
                                                                            streamObj << std::setprecision(2) << std::fixed;

                                                                            streamObj << roundToDigits(
                                                                                    accumDuration / 1000.0, 2);
                                                                            std::string accumDurationStr = streamObj.str();

                                                                            infoWidgets.accumTimerReport->setValue(label + ":" + accumDurationStr + "|" + std::to_string(percentageAccum) + "%");

                                                                        },
                                                                        [&]() -> int {
                                                                            return particleInfoWidgets.neighborId;
                                                                        }, panel);

  infoWidgets.timerReportIdxControl->setValue(0);

  formWidgetSetSpinnableWithIncrement(infoWidgets.timerReportIdxControl, 1);

}



void Viewer::forwardInfoWindowContentUpdate(ForwardInfoWidgets &forwardWidgets,
                                            Simulation::ForwardInformation &forwardInfo) {
  auto roundToDigits = [](float value, int digits) {
      int a = std::pow(10, digits);
      return std::round(value * a) / a;
  };
  forwardWidgets.converged->setValue(forwardInfo.converged);
  forwardWidgets.convergeIter->setValue(forwardInfo.convergeIter);
  forwardWidgets.accumulateIter->setValue(forwardInfo.cumulateIter);
  forwardWidgets.externalCollisionNumber->setValue(forwardInfo.collisionInfos.first.first.size());
  forwardWidgets.selfCollisionNumber->setValue(forwardInfo.collisionInfos.first.second.size());
  forwardWidgets.avgDeformation->setValue(roundToDigits(forwardInfo.avgDeformation, 3));
  forwardWidgets.timerReportIdxControl->callback()(std::to_string(forwardWidgets.timerReportIdxControl->value()));
  forwardWidgets.totalTime->setValue(roundToDigits(forwardInfo.totalRuntime / 1000000.0, 5));
  forwardWidgets.iterTime->setValue(roundToDigits(forwardInfo.timer.totalMicroseconds / 1000000.0, 5));

}

void Viewer::addBackwardSimulationInfoWidets(std::string title, BackwardGradientInfoWidgets &infoWidgets,
                                             Simulation::BackwardInformation &gradientInfo,
                                             nanogui::ref<nanogui::Window> window, int widthOffset, bool isFiniteDiff) {
  using namespace nanogui;
  window = SettingsControl->addWindow(Eigen::Vector2i(width - widthOffset, height - 400),
                                      title);
  for (int i = 0; i < 2; i++) {
    infoWidgets.stiffness[i] = SettingsControl->addVariable((i == 0) ? "dL/dk_stretch" : "dL/dk_bend",
                                                            gradientInfo.dL_dk_pertype[2 + i]);
    infoWidgets.stiffness[i]->setEditable(false);
    infoWidgets.stiffness[i]->setFontSize(15);

  }
  Widget *panel = new Widget(window);
  panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                 Alignment::Minimum, 0, 0));
//  panel->setFixedWidth(150);
  SettingsControl->addWidget("f_ext", panel);
  for (int i = 0; i < 3; i++) {
    infoWidgets.f_ext[i] = SettingsControl->addVariable("", gradientInfo.dL_dfext[i], panel);
    infoWidgets.f_ext[i]->setEditable(false);
    infoWidgets.f_ext[i]->setFontSize(15);
    infoWidgets.f_ext[i]->setFixedWidth(60);
  }

  panel = new Widget(window);
  panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                 Alignment::Minimum, 0, 0));
  SettingsControl->addWidget("f_wind", panel);
  for (int i = 0; i < 5; i++) {
    infoWidgets.f_wind[i] = SettingsControl->addVariable("", gradientInfo.dL_dwind[i], panel);
    infoWidgets.f_wind[i]->setEditable(false);
    infoWidgets.f_wind[i]->setFontSize(15);
    infoWidgets.f_wind[i]->setFixedWidth(60);
  }

  for (int splineIdx = 0; splineIdx < 4; splineIdx++) {
    panel = new Widget(window);
    panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                   Alignment::Minimum, 0, 0));
    SettingsControl->addWidget("spline " + std::to_string(splineIdx), panel);
    infoWidgets.dL_dspline.emplace_back();
    for (int i = 0; i < 9; i++) {
      infoWidgets.dL_dspline[splineIdx].emplace_back(
              SettingsControl->addVariable("", gradientInfo.dL_dsplines[0][0][0], panel));
      infoWidgets.dL_dspline[splineIdx][i]->setEditable(false);
      infoWidgets.dL_dspline[splineIdx][i]->setFontSize(15);
      infoWidgets.dL_dspline[splineIdx][i]->setFixedWidth(40);
    }
  }

  infoWidgets.density = SettingsControl->addVariable("dL/ddensity", gradientInfo.dL_dk_pertype[3]);
  infoWidgets.x0 = SettingsControl->addVariable("||x0||", gradientInfo.dL_dk_pertype[3]);
  infoWidgets.mu = SettingsControl->addVariable("dL/mu", gradientInfo.dL_dk_pertype[3]);

  if (!isFiniteDiff) {
    infoWidgets.loss = SettingsControl->addVariable("loss", gradientInfo.loss);
    {
      panel = new Widget(window);
      panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                     Alignment::Fill, 0, 0));
      SettingsControl->addWidget("convergence", panel);

      infoWidgets.convergeIter = SettingsControl->addVariable("iter",
                                                              backwardInfo.backwardIters, panel);
      infoWidgets.accumConverge = SettingsControl->addVariable("total converged",
                                                               backwardInfo.convergedAccum, panel);
      infoWidgets.rho = SettingsControl->addVariable("rho",
                                                     backwardInfo.rho, panel);
      infoWidgets.convergeIter->setFixedWidth(60);
      infoWidgets.accumConverge->setFixedWidth(60);
      infoWidgets.converged = SettingsControl->addVariable("converged",
                                                           forwardInfo.converged, panel);
      infoWidgets.converged->setEditable(false);
    }

    {
      panel = new Widget(window);
      panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                     Alignment::Fill, 0, 0));
      SettingsControl->addWidget("iter perf", panel);


      infoWidgets.perfTimer[0] = SettingsControl->addVariable("t_other",
                                                              backwardInfo.timer.solvePerfReport.nonSolveTimeMicroseconds,
                                                              panel);
      infoWidgets.perfTimer[1] = SettingsControl->addVariable("t_direct",
                                                              backwardInfo.timer.solvePerfReport.solveDirectMicroseconds,
                                                              panel);
      infoWidgets.perfTimer[2] = SettingsControl->addVariable("t_iter",
                                                              backwardInfo.timer.solvePerfReport.solveIterativeMicroseconds,
                                                              panel);


      panel = new Widget(window);
      panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                     Alignment::Fill, 0, 0));
      SettingsControl->addWidget("accum perf", panel);
      infoWidgets.accumPerfTimer[0] = SettingsControl->addVariable("t_other",
                                                                   backwardInfo.accumSolvePerformanceReport.nonSolveTimeMicroseconds,
                                                                   panel);
      infoWidgets.accumPerfTimer[1] = SettingsControl->addVariable("t_direct",
                                                                   backwardInfo.accumSolvePerformanceReport.solveDirectMicroseconds,
                                                                   panel);
      infoWidgets.accumPerfTimer[2] = SettingsControl->addVariable("t_iter",
                                                                   backwardInfo.accumSolvePerformanceReport.solveIterativeMicroseconds,
                                                                   panel);
      for (int i = 0; i < 3; i++) {
        infoWidgets.perfTimer[i]->setFixedWidth(80);
        infoWidgets.perfTimer[i]->setEditable(false);
        infoWidgets.accumPerfTimer[i]->setEditable(false);
        infoWidgets.accumPerfTimer[i]->setFixedWidth(80);
      }

    }

    {
      panel = new Widget(window);
      panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                     Alignment::Fill, 0, 0));
      SettingsControl->addWidget("iterTimer", panel);
      infoWidgets.timerId = 0;
      infoWidgets.timerReport = new TextBox(panel, "empty");
      infoWidgets.timerReport->setFontSize(15);
      infoWidgets.timerReport->setFixedWidth(200);
      infoWidgets.timerReport->setEditable(false);

      panel = new Widget(window);
      panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                     Alignment::Fill, 0, 0));
      SettingsControl->addWidget("accumTimer", panel);

      infoWidgets.accumTimerReport = new TextBox(panel, "empty");
      infoWidgets.accumTimerReport->setFontSize(15);
      infoWidgets.accumTimerReport->setFixedWidth(200);
      infoWidgets.accumTimerReport->setEditable(false);
      infoWidgets.timerReportIdxControl = SettingsControl->addVariable<int>("idx",
                                                                            [&](const int &v) {
                                                                                if (v !=
                                                                                    std::clamp(v, 0,
                                                                                               (int) gradientInfo.timer.timeMicroseconds.size() -
                                                                                               1)) {
                                                                                  return;
                                                                                }
                                                                                infoWidgets.timerId = v;
                                                                                std::string label = gradientInfo.timer.timeMicroseconds[v].first;
                                                                                long long duration = gradientInfo.timer.timeMicroseconds[v].second;
                                                                                long long durationTotal =
                                                                                        gradientInfo.timer.totalMicroseconds -
                                                                                        gradientInfo.timer.solvePerfReport.solveDirectMicroseconds -
                                                                                        gradientInfo.timer.solvePerfReport.solveIterativeMicroseconds;
                                                                                int percentage = (duration * 100.0 /
                                                                                                  durationTotal);
                                                                                std::ostringstream streamObj;
                                                                                streamObj << std::setprecision(2)
                                                                                          << std::fixed;
                                                                                streamObj << roundToDigits(duration, 2);
                                                                                infoWidgets.timerReport->setValue(
                                                                                        label + ":" + streamObj.str() +
                                                                                        "|" +
                                                                                        std::to_string(percentage) +
                                                                                        "%");


                                                                                double accumDuration = gradientInfo.accumTimer[v].second;
                                                                                double accumTotal = 0;
                                                                                for (std::pair<std::string, long long> &item : gradientInfo.accumTimer) {
                                                                                  if ((item.first != "solveIterative") && (item.first != "solveDirect")) {
                                                                                    accumTotal += item.second;
                                                                                  }
                                                                                }
                                                                                int percentageAccum =(accumDuration / accumTotal) * 100;
                                                                                streamObj.clear();
                                                                                streamObj.str("");
                                                                                streamObj << std::setprecision(2) << std::fixed;
                                                                                streamObj << roundToDigits(accumDuration,2);

                                                                                infoWidgets.accumTimerReport->setValue(label + ":" + streamObj.str() + "|" + std::to_string(percentageAccum) + "%");

                                                                            },
                                                                            [&]() -> int {
                                                                                return particleInfoWidgets.neighborId;
                                                                            }, panel);

      infoWidgets.timerReportIdxControl->setValue(0);

      infoWidgets.totalTime = SettingsControl->addVariable("t_total[s]",
                                                           backwardInfo.totalRuntime, panel);
      formWidgetSetSpinnableWithIncrement(infoWidgets.timerReportIdxControl, 1);
    }
  }


}

void Viewer::gradientInfoWindowContentUpdate(BackwardGradientInfoWidgets &gradientWidgets,
                                             Simulation::BackwardInformation &gradient, bool isFiniteDiff) {

//  std::printf("gradient info content update\n");
  auto roundToDigits = [](float value, int digits) {
      int a = std::pow(10, digits);
      return std::round(value * a) / a;
  };


  for (int i = 0; i < 2; i++)
    gradientWidgets.stiffness[i]->setValue(roundToDigits(gradient.dL_dk_pertype[2 + i], 5));

  for (int i = 0; i < 3; i++)
    gradientWidgets.f_ext[i]->setValue(roundToDigits(gradient.dL_dfext[i], 2));

  for (int i = 0; i < 5; i++)
    gradientWidgets.f_wind[i]->setValue(roundToDigits(gradient.dL_dwind[i], 2));

  int sysMatId = forwardInfo.sysMatId;
  int splineNum = std::min(gradient.dL_dsplines[sysMatId].size(), gradientWidgets.dL_dspline.size());
  for (int i = 0; i < splineNum; i++) {
    for (int paramIdx = 0; paramIdx < gradient.dL_dsplines[sysMatId][i].rows(); paramIdx++) {
      if (paramIdx >= gradientWidgets.dL_dspline[i].size())
        continue;
      gradientWidgets.dL_dspline[i][paramIdx]->setValue(
              roundToDigits(gradient.dL_dsplines[sysMatId][i][paramIdx], 1));
    }
  }
  if (!gradient.dL_dmu.empty())
    gradientWidgets.mu->setValue(roundToDigits(gradient.dL_dmu[0].second, 5));

  gradientWidgets.density->setValue(roundToDigits(gradient.dL_ddensity, 5));
  gradientWidgets.density->setValue(roundToDigits(gradient.dL_dx.norm(), 5));

  if (!isFiniteDiff) {
    gradientWidgets.accumConverge->setValue(gradient.convergedAccum);
    gradientWidgets.loss->setValue(roundToDigits(gradient.loss, 2));
    gradientWidgets.converged->setValue(gradient.converged);
    gradientWidgets.rho->setValue(gradient.rho);
    gradientWidgets.convergeIter->setValue(gradient.backwardIters);
    gradientWidgets.timerReportIdxControl->callback()(std::to_string(gradientWidgets.timerReportIdxControl->value()));

    gradientWidgets.perfTimer[0]->setValue(gradient.timer.solvePerfReport.nonSolveTimeMicroseconds);
    gradientWidgets.perfTimer[1]->setValue(gradient.timer.solvePerfReport.solveDirectMicroseconds);
    gradientWidgets.perfTimer[2]->setValue(gradient.timer.solvePerfReport.solveIterativeMicroseconds);

    gradientWidgets.accumPerfTimer[0]->setValue(gradient.accumSolvePerformanceReport.nonSolveTimeMicroseconds);
    gradientWidgets.accumPerfTimer[1]->setValue(gradient.accumSolvePerformanceReport.solveDirectMicroseconds);
    gradientWidgets.accumPerfTimer[2]->setValue(gradient.accumSolvePerformanceReport.solveIterativeMicroseconds);
    gradientWidgets.totalTime->setValue(roundToDigits(gradient.totalRuntime / 1000.0, 2));

  }
}

nanogui::TextBox *createTextBox(nanogui::Widget *parent, std::string defaultValue, double fontSize) {
  return new nanogui::TextBox(parent);
}

void Viewer::addParticleInfoWidgets() {
  particleInfoWindow = SettingsControl->addWindow(Eigen::Vector2i(300, 0), "Particle&Triangle Info");
  using namespace nanogui;
  SettingsControl->addGroup("Particle");

  panel = new Widget(particleInfoWindow);
  panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                 Alignment::Middle, 0, 0));
  SettingsControl->addWidget("", panel);

  for (int i = 0; i < 10; i++) {
    TextBox *textBox = new TextBox(screen);
    simSystemNameBoard.push_back(textBox);
    textBox->setPosition(Vec2i(0, 0));
    textBox->setValue("99999");
    textBox->setFontSize(13);
    textBox->setAlignment(TextBox::Alignment::Center);
  }
  particleInfoWidgets.particleIdxControl = SettingsControl->addVariable<int>("particleIdx",
                                                                             [&](const int &v) {
                                                                                 particleId = v;

                                                                                 updateParticleInfoContent();

                                                                             },
                                                                             [&]() -> int {

                                                                                 return particleId;
                                                                             }, panel
  );

  formWidgetSetSpinnableWithIncrement(particleInfoWidgets.particleIdxControl, 1);

  Particle &p = simSystems[0]->particles[particleId];
  particleInfoWidgets.mass = SettingsControl->addVariable("mass", p.mass, panel);
  particleInfoWidgets.area = SettingsControl->addVariable("area", p.area, panel);

  Widget *panel = new Widget(particleInfoWindow);
  panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                 Alignment::Middle, 0, 0));
  SettingsControl->addWidget("Triangles", panel);
  particleInfoWidgets.neighborId = 0;
  particleInfoWidgets.triangles = new TextBox(panel, "triIdx");
  particleInfoWidgets.neighborIdxToggle = SettingsControl->addVariable<int>("idx",
                                                                            [&](const int &v) {
                                                                                if (particleId !=
                                                                                    std::clamp(particleId, 0,
                                                                                               (int) simSystems[0]->particles.size() -
                                                                                               1))
                                                                                  return;
                                                                                Particle &p = simSystems[0]->particles[particleId];
                                                                                std::vector<int> &neighbors = simSystems[0]->particleTriangleMap[p.idx];
                                                                                if (v != std::clamp(v, 0,
                                                                                                    (int) neighbors.size() -
                                                                                                    1))
                                                                                  return;
                                                                                particleInfoWidgets.neighborId = v;
                                                                                particleInfoWidgets.triangles->setValue(
                                                                                        std::to_string(neighbors[v]));

                                                                                particleInfoWidgets.triangleNeighborFocus->callback()();
                                                                            },
                                                                            [&]() -> int {
                                                                                return particleInfoWidgets.neighborId;
                                                                            }, panel);
  formWidgetSetSpinnableWithIncrement(particleInfoWidgets.neighborIdxToggle, 1);
  particleInfoWidgets.triangleNeighborFocus = new Button(panel, "", ENTYPO_ICON_AIR);
  particleInfoWidgets.triangleNeighborFocus->setCallback([&]() {
      if (particleId != std::clamp(particleId, 0, (int) simSystems[0]->particles.size() - 1))
        return;
      Particle &p = simSystems[0]->particles[particleId];
      std::vector<int> &neighbors = simSystems[0]->particleTriangleMap[p.idx];
      if (particleInfoWidgets.neighborId != std::clamp(particleInfoWidgets.neighborId, 0, (int) neighbors.size() - 1))
        return;

      int neighborTriId = neighbors[particleInfoWidgets.neighborId];
      triangleInfoWidgets.triangleIdxControl->callback()(std::to_string(neighborTriId));

  });
  particleInfoWidgets.focus = SettingsControl->addButton("focus", [&]() {
      if (std::clamp(particleId, 0, ((int) simSystems[0]->particles.size()) - 1) != particleId)
        return;
      Particle &p = simSystems[0]->particles[particleId];
      Vec3d pos;
      if (isforward) {
        std::vector<Simulation::ForwardInformation> forwardCopy = simSystems[0]->forwardRecords;
        if (std::clamp(playBackId, 0, ((int) forwardCopy.size()) - 1) != playBackId)
          return;
        pos = forwardCopy[playBackId].x.segment(p.idx * 3, 3);
      } else {
        if ((!playGroundtruth)  && (!playLineSearch)) {
          if (std::clamp(backwardSelectedRecordId, 0, ((int) simSystems[0]->backwardOptimizationRecords.size()) - 1) !=
              backwardSelectedRecordId)
            return;
          if (std::clamp(playBackId, 0,
                         ((int) simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].first.size()) -
                         1) !=
              playBackId)
            return;
          pos = simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].first[playBackId].x.segment(
                  p.idx * 3, 3);
        } else if (playGroundtruth) {
          if (std::clamp(playBackId, 0,
                         ((int) simSystems[0]->groundTruthForwardRecords.size()) - 1) !=
              playBackId)
            return;
          pos = simSystems[0]->groundTruthForwardRecords[playBackId].x.segment(p.idx * 3, 3);
        } else if (playLineSearch) {
          if (std::clamp(playBackId, 0,
                         ((int) simSystems[0]->linesearchRecords.size()) - 1) !=
              playBackId)
            return;
          pos = simSystems[0]->linesearchRecords[playBackId].x.segment(p.idx * 3, 3);
        }

      }
      camera.setLookAt(glm::vec3(pos[0], pos[1], pos[2]));

  });
  particleInfoWidgets.triangles->setFontSize(13);


  SettingsControl->addGroup("Triangle");

  panel = new Widget(particleInfoWindow);
  panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                 Alignment::Middle, 0, 0));
  SettingsControl->addWidget("", panel);

  triangleInfoWidgets.triangleIdxControl = SettingsControl->addVariable<int>("triangleIdx",
                                                                             [&](const int &v) {
                                                                                 updateParticleInfoContent();
                                                                                 triangleId = v;
                                                                             },
                                                                             [&]() -> int {
                                                                                 return triangleId;
                                                                             }, panel);
  formWidgetSetSpinnableWithIncrement(triangleInfoWidgets.triangleIdxControl, 1);
  triangleInfoWidgets.triangleIdxControl->setCallback([&](int idx) {
      triangleId = idx;
      updateParticleInfoContent();
  });
  Triangle &t = simSystems[0]->mesh[triangleId];

  triangleInfoWidgets.area = SettingsControl->addVariable("area", t.area_rest, panel);
  panel = new Widget(particleInfoWindow);

  panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                 Alignment::Middle, 0, 0));
  SettingsControl->addWidget("Particles", panel);
  for (int i = 0; i < 3; i++) {
    triangleInfoWidgets.particleId[i] = SettingsControl->addVariable("", t.idxArr[i], panel);
    triangleInfoWidgets.focusButtons[i] = new Button(panel, "", ENTYPO_ICON_AIRCRAFT);
    triangleInfoWidgets.focusButtons[i]->setCallback([this, i]() {
        if (std::clamp(triangleId, 0, (int) simSystems[0]->mesh.size()) != triangleId)
          return;
        Triangle &t = simSystems[0]->mesh[triangleId];
        particleId = t.idxArr[i];
        particleInfoWidgets.particleIdxControl->callback()(std::to_string(particleId));
    });
  }
  triangleInfoWidgets.focus = SettingsControl->addButton("focus", [&]() {
      if (std::clamp(triangleId, 0, (int) simSystems[0]->mesh.size()) != triangleId)
        return;
      Triangle &t = simSystems[0]->mesh[triangleId];
      Vec3d pos;
      if (isforward) {
        if (std::clamp(playBackId, 0, ((int) simSystems[0]->forwardRecords.size()) - 1) != playBackId)
          return;
        pos = simSystems[0]->forwardRecords[playBackId].x.segment(t.p0_idx * 3, 3);
      } else {
        if (!playGroundtruth) {
          if (std::clamp(backwardSelectedRecordId, 0, ((int) simSystems[0]->backwardOptimizationRecords.size()) - 1) !=
              backwardSelectedRecordId)
            return;
          if (std::clamp(playBackId, 0,
                         ((int) simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].first.size()) -
                         1) !=
              playBackId)
            return;
          pos = simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].first[playBackId].x.segment(
                  t.p0_idx * 3, 3);
        } else {
          if (std::clamp(playBackId, 0,
                         ((int) simSystems[0]->groundTruthForwardRecords.size()) - 1) !=
              playBackId)
            return;
          pos = simSystems[0]->groundTruthForwardRecords[playBackId].x.segment(t.p0_idx * 3, 3);
        }

      }
      camera.setLookAt(glm::vec3(pos[0], pos[1], pos[2]));
  });
  updateParticleInfoContent();

}

void Viewer::updateParticleInfoContent() {
  auto roundToDigits = [](float value, int digits) {
      int a = std::pow(10, digits);
      return std::round(value * a) / a;
  };
  Simulation::ForwardInformation currentRecord = {};
  bool proceed = true;

  if (isforward) {
    currentRecord = simSystems[0]->forwardRecords[playBackId];
  } else {
    if ((!playGroundtruth) && (!playLineSearch)) {
      if (std::clamp(backwardSelectedRecordId, 0, ((int) simSystems[0]->backwardOptimizationRecords.size()) - 1) !=
          backwardSelectedRecordId)
        proceed = false;

      if (std::clamp(playBackId, 0,
                     ((int) simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].first.size()) -
                     1) != playBackId)
        proceed = false;
      if (proceed)
        currentRecord = simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].first[playBackId];
    } else if (playGroundtruth) {
      bool proceed = true;

      if (std::clamp(playBackId, 0,
                     ((int) simSystems[0]->groundTruthForwardRecords.size()) - 1) !=
          playBackId)
        proceed = false;
      if (proceed)
        currentRecord = simSystems[0]->groundTruthForwardRecords[playBackId];
    } else if (playLineSearch) {
      bool proceed = true;
      if (std::clamp(playBackId, 0,
                     ((int) simSystems[0]->linesearchRecords.size()) - 1) !=
          playBackId)
        proceed = false;
      if (proceed)
        currentRecord = simSystems[0]->linesearchRecords[playBackId];
    }

  }


  if (particleId != std::clamp(particleId, 0, (int) simSystems[0]->particles.size() - 1))
    return;

  if ((particleId >= 0) && (particleId < simSystems[0]->particles.size())) {
    Particle &p = simSystems[0]->particles[particleId];
    std::printf("selected particle %d: mass:%.3f area: %.3f neighborhoodsize: %zu\n", p.idx, p.mass, p.area,
                simSystems[0]->particleTriangleMap[p.idx].size());
    particleInfoWidgets.mass->setValue(roundToDigits(p.mass, 6));
    particleInfoWidgets.area->setValue(roundToDigits(p.area, 6));

    std::vector<int> &neighbors = simSystems[0]->particleTriangleMap[p.idx];
    if (particleInfoWidgets.neighborId == std::clamp(particleInfoWidgets.neighborId, 0, (int) neighbors.size() - 1)) {
      particleInfoWidgets.triangles->setValue(std::to_string(neighbors[particleInfoWidgets.neighborId]));
    }

    std::set<int> particleNeighborhood;
    for (int triIdx : neighbors) {
      Triangle &t = simSystems[0]->mesh[triIdx];
      for (int vIdx : t.idxArr) {
        particleNeighborhood.emplace(vIdx);
      }
    }
    int count = 0;




  }

  if ((triangleId >= 0) && ((triangleId < simSystems[0]->mesh.size()))) {
    Triangle &t = simSystems[0]->mesh[triangleId];
    triangleInfoWidgets.area->setValue(roundToDigits(t.area_rest, 6));
    for (int i = 0; i < 3; i++) {
      triangleInfoWidgets.particleId[i]->setValue(t.idxArr[i]);
    }
  }


}
void Viewer::addBackwardSimulationControlWidets() {
  using namespace nanogui;
  backwardSimControlWindow = SettingsControl->addWindow(Eigen::Vector2i(width - 350, 200), "Backward Cloth Simulation");

  backwardInfo1 = new TextBox(backwardSimControlWindow);
  backwardInfo1->setValue("Not Solving");
  backwardInfo1->setFontSize(15);
  backwardInfo1->setFixedSize(Vec2i(90, 20));
  backwardInfo1->setAlignment(TextBox::Alignment::Left);
  backwardInfo1->setEnabled(true);
  backwardInfo1->setVisible(true);
  SettingsControl->addWidget("Solving State", backwardInfo1);


  backwardInfo2 = new TextBox(backwardSimControlWindow);
  backwardInfo2->setValue("Not Solving");
  backwardInfo2->setFontSize(15);
  backwardInfo2->setAlignment(TextBox::Alignment::Left);
  backwardInfo2->setEnabled(true);
  backwardInfo2->setVisible(true);
  backwardInfo2->setHeight(5);

  SettingsControl->addWidget("Current Status", backwardInfo2);
  perfResolutionWidget = SettingsControl->addVariable<int>("Perf resoution", [&](const int &v) { perfResolution = v; },
                                                           [&]() -> int { return perfResolution; });
  perfResolutionWidget->setMinValue(0);
  perfResolutionWidget->setMaxValue(3);
  formWidgetSetSpinnableWithIncrement(perfResolutionWidget, 1);

  SettingsControl->addVariable<bool>("Finite diff",
                                     [&](const bool &v) {
                                         useFiniteDiff = v;
                                     },
                                     [&]() -> bool { return useFiniteDiff; }
  );
  SettingsControl->addVariable<bool>("LBfGS",
                                     [&](const bool &v) {
                                         useLBFGS = v;
                                     },
                                     [&]() -> bool { return useLBFGS; }
  );



  SettingsControl->addVariable<bool>("Rand init",
                                     [&](const bool &v) {
                                         randomInit = v;
                                     },
                                     [&]() -> bool { return randomInit; }
  );

 
  Widget *panel = new Widget(backwardSimControlWindow);
  panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                 Alignment::Minimum, 0, 0));
  SettingsControl->addWidget("Record", panel);

  { // Backward select records controls, slider+textbox+two buttons
    backwardRecordSliderText.slider = new Slider(panel);
    backwardRecordSliderText.slider->setValue(0.0f);
    backwardRecordSliderText.slider->setFixedWidth(100);
    backwardRecordSliderText.textbox = new TextBox(panel);
    backwardRecordSliderText.textbox->setFontSize(15);
    backwardRecordSliderText.textbox->setUnits("");
    backwardRecordSliderText.textbox->setValue(simSystems[0]->backwardOptimizationRecords.size() == 0 ? "0/0" : "1/" +
                                                                                                                std::to_string(
                                                                                                                        simSystems[0]->backwardOptimizationRecords.size()));
    backwardRecordSliderText.slider->setFinalCallback([&](float value) {
        std::vector<std::pair<std::vector<Simulation::ForwardInformation>,
                std::vector<Simulation::BackwardInformation >>> &backwardRecords = simSystems[0]->backwardOptimizationRecords;
        std::string recordText;

        floatToIdxAndText(value, backwardRecords.size(), backwardSelectedRecordId, recordText);
        backwardRecordSliderText.idx = backwardSelectedRecordId;
        backwardRecordSliderText.idxWidget->setValue(backwardSelectedRecordId);
        backwardRecordSliderText.idxWidget->setFixedWidth(60);
        backwardRecordSliderText.textbox->setValue(recordText);
        playGroundtruth = false;
        playLineSearch = false;
        isplaybackSimulationOn = false;
        playBackId = 0;

    });
    backwardRecordSliderText.idx = 0;
    backwardRecordSliderText.idxWidget = SettingsControl->addVariable<int>("",
                                                                           [&](const int &v) {
                                                                               int totalRecords = simSystems[0]->backwardOptimizationRecords.size();
                                                                               int prevIdx = backwardSelectedRecordId;
                                                                               backwardSelectedRecordId = std::clamp(v,
                                                                                                                     0,
                                                                                                                     totalRecords -
                                                                                                                     1);
                                                                               backwardRecordSliderText.idx = backwardSelectedRecordId;
                                                                               if (backwardSelectedRecordId !=
                                                                                   prevIdx) {
                                                                                 double sliderVal;
                                                                                 std::string recordText;
                                                                                 idxToFloatAndText(
                                                                                         backwardSelectedRecordId,
                                                                                         totalRecords, sliderVal,
                                                                                         recordText);
                                                                                 backwardRecordSliderText.slider->setValue(
                                                                                         sliderVal);
                                                                                 backwardRecordSliderText.slider->finalCallback()(
                                                                                         sliderVal);
                                                                                 backwardFrameSliderText.slider->setValue(
                                                                                         0);
                                                                                 backwardFrameSliderText.slider->finalCallback()(
                                                                                         0);
                                                                               }
                                                                           },
                                                                           [&]() -> int {
                                                                               return backwardRecordSliderText.idx;
                                                                           }, panel
    );
    formWidgetSetSpinnableWithIncrement(backwardRecordSliderText.idxWidget, 1);
    backwardRecordSliderText.idxWidget->setMinValue(0);
    backwardRecordSliderText.idxWidget->setFixedWidth(60);

  }
  { // Backward select frame control
    panel = new Widget(backwardSimControlWindow);
    panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                   Alignment::Minimum, 0, 0));
    SettingsControl->addWidget("Frame", panel);

    backwardFrameSliderText.slider = new Slider(panel);
    backwardFrameSliderText.slider->setFixedWidth(100);
    backwardFrameSliderText.slider->setValue(0.0);
    backwardFrameSliderText.textbox = new TextBox(panel);
    backwardFrameSliderText.textbox->setFontSize(15);
    backwardFrameSliderText.textbox->setUnits("");
    backwardFrameSliderText.textbox->setValue(simSystems[0]->backwardOptimizationRecords.size() == 0 ? "0/0" : "1/" +
                                                                                                               std::to_string(
                                                                                                                       simSystems[0]->backwardOptimizationRecords[0].first.size()));
    backwardFrameSliderText.slider->setFinalCallback([&](float value) {
        if ((!playGroundtruth) && (!playLineSearch) && (
                (backwardSelectedRecordId >= simSystems[0]->backwardOptimizationRecords.size())))
          return;
        int totalFrames = simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].first.size();
        if (playGroundtruth)
          totalFrames = simSystems[0]->groundTruthForwardRecords.size();
        if (playLineSearch)
          totalFrames = simSystems[0]->linesearchRecords.size();
        int selectedFrame = 0;
        std::string text;
        floatToIdxAndText(value, totalFrames, selectedFrame, text);
        playBackId = selectedFrame;
        backwardFrameSliderText.textbox->setValue(text);
        backwardFrameSliderText.idxWidget->setValue(playBackId);
        backwardFrameSliderText.idx = playBackId;
        if ((!playGroundtruth) && (!playLineSearch)) {
//          std::printf(
//                  "trying to update actual gradient ...  totalBackwardRecordNum: %zu select:%d totalFrames: %zu totalBackwardNum: %zu, selectId: %d\n",
//                  simSystems[0]->backwardOptimizationRecords.size(),
//                  backwardSelectedRecordId,
//                  simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].first.size(),
//                  simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].second.size(),
//                  playBackId - 1
//          );
          // for x0,v0 display the first backward info
          forwardInfo = simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].first[playBackId];
          backwardInfo = simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].second[playBackId];
          gradientInfoWindowContentUpdate(gradientInfoWidgets, backwardInfo, false);
          forwardInfoWindowContentUpdate(forwardInfoWidgets, forwardInfo);

          if (useFiniteDiff) {

            if ((simSystems[0]->backwardRecordsFiniteDiff.size() > backwardSelectedRecordId) &&
                (simSystems[0]->backwardRecordsFiniteDiff[backwardSelectedRecordId].size() > playBackId)) {
              finitediffGradientInfo = simSystems[0]->backwardRecordsFiniteDiff[backwardSelectedRecordId][playBackId];
              gradientInfoWindowContentUpdate(finiteDiffInfoWidgets, finitediffGradientInfo, true);
            } else {
              std::printf("ERROR: want to display finite diff info but there is record number mismatch\n");
              std::printf(
                      "trying to update actual gradient ...  totalFiniteDiffRecordNum: %zu select:%d  , selectId: %d\n",
                      simSystems[0]->backwardRecordsFiniteDiff.size(), backwardSelectedRecordId, playBackId - 1
              );
            }


          }

        } else if (playGroundtruth)  {
          forwardInfo = simSystems[0]->groundTruthForwardRecords[playBackId];
          forwardInfoWindowContentUpdate(forwardInfoWidgets, forwardInfo);
        } else if (playLineSearch) {
          forwardInfo = simSystems[0]->linesearchRecords[playBackId];
          forwardInfoWindowContentUpdate(forwardInfoWidgets, forwardInfo);
        }


    });
    backwardFrameSliderText.idx = 0;
    backwardFrameSliderText.idxWidget =
            SettingsControl->addVariable<int>("",
                                              [&](const int &v) {
                                                  if ((!playGroundtruth) && (!playLineSearch) && (
                                                          (backwardSelectedRecordId >=
                                                           simSystems[0]->backwardOptimizationRecords.size())))
                                                    return;
                                                  int totalSize =  simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].first.size();
                                                  if (playGroundtruth)
                                                    totalSize = simSystems[0]->groundTruthForwardRecords.size();
                                                  if (playLineSearch)
                                                    totalSize = simSystems[0]->linesearchRecords.size();
                                                  int playBackIdBefore = playBackId;
                                                  playBackId = std::clamp(v, 0, totalSize - 1);
                                                  backwardFrameSliderText.idx = v;

                                                  if (playBackIdBefore == playBackId)
                                                    return;
                                                  std::string text;
                                                  double sliderVal;
                                                  idxToFloatAndText(playBackId, totalSize, sliderVal, text);
                                                  if (std::abs(sliderVal - backwardFrameSliderText.slider->value()) >
                                                      0.001) {
                                                    backwardFrameSliderText.slider->setValue(sliderVal);
                                                    backwardFrameSliderText.slider->finalCallback()(sliderVal);
                                                  }
                                              },
                                              [&]() -> int {
                                                  return backwardFrameSliderText.idx;
                                              }, panel
            );
    backwardFrameSliderText.idxWidget->setMinValue(0);
    formWidgetSetSpinnableWithIncrement(backwardFrameSliderText.idxWidget, 1);

  }


  SettingsControl->addButton("Demo Wind", [&]() {
      isSimulationOn = false;
      isforward = false;
      backwardNeedsInitiation = true;
      selectedDemoIdx = Demos::DEMO_WIND;
      isplaybackSimulationOn = false;
      backwardSelectedRecordId = 0;
      forwardStartSimulationButton->setEnabled(false);
      forwardStopSimulationButton->setEnabled(false);
      playbackSimulationButton->setEnabled(false);
      backwardPlayGroundtruthSimulationButton->setEnabled(false);
  });



  SettingsControl->addButton("Demo sphere", [&]() {
      isSimulationOn = false;
      isforward = false;
      backwardNeedsInitiation = true;
      selectedDemoIdx = Demos::DEMO_SPHERE_ROTATE;
      isplaybackSimulationOn = false;
      backwardSelectedRecordId = 0;
      forwardStartSimulationButton->setEnabled(false);
      forwardStopSimulationButton->setEnabled(false);
      playbackSimulationButton->setEnabled(false);
      backwardPlayGroundtruthSimulationButton->setEnabled(false);
  });




  SettingsControl->addButton("Demo Wind T-shirt", [&]() {
      isSimulationOn = false;
      isforward = false;
      backwardNeedsInitiation = true;
      selectedDemoIdx = Demos::DEMO_WIND_TSHIRT;
      isplaybackSimulationOn = false;
      backwardSelectedRecordId = 0;
      forwardStartSimulationButton->setEnabled(false);
      forwardStopSimulationButton->setEnabled(false);
      playbackSimulationButton->setEnabled(false);
      backwardPlayGroundtruthSimulationButton->setEnabled(false);
  });


  SettingsControl->addButton("Demo Dress Twirl", [&]() {
      isSimulationOn = false;
      isforward = false;
      backwardNeedsInitiation = true;
      selectedDemoIdx = Demos::DEMO_DRESS_TWIRL;
      isplaybackSimulationOn = false;
      backwardSelectedRecordId = 0;
      forwardStartSimulationButton->setEnabled(false);
      forwardStopSimulationButton->setEnabled(false);
      playbackSimulationButton->setEnabled(false);
      backwardPlayGroundtruthSimulationButton->setEnabled(false);
  });


  SettingsControl->addButton("Demo Slope Perf", [&]() {
      isSimulationOn = false;
      isforward = false;
      backwardNeedsInitiation = true;
      selectedDemoIdx = Demos::DEMO_SLOPE_PERF;
      isplaybackSimulationOn = false;
      backwardSelectedRecordId = 0;
      perfTest = true;

      forwardStartSimulationButton->setEnabled(false);
      forwardStopSimulationButton->setEnabled(false);
      playbackSimulationButton->setEnabled(false);
      backwardPlayGroundtruthSimulationButton->setEnabled(false);
  });

  SettingsControl->addButton("Demo Wind Perf", [&]() {
      isSimulationOn = false;
      isforward = false;
      backwardNeedsInitiation = true;
      selectedDemoIdx = Demos::DEMO_WIND;
      perfTest = true;
      isplaybackSimulationOn = false;
      backwardSelectedRecordId = 0;
      forwardStartSimulationButton->setEnabled(false);
      forwardStopSimulationButton->setEnabled(false);
      playbackSimulationButton->setEnabled(false);
      backwardPlayGroundtruthSimulationButton->setEnabled(false);
  });

  SettingsControl->addButton("Demo Wear hat", [&]() {
      isSimulationOn = false;
      isforward = false;
      backwardNeedsInitiation = true;
      selectedDemoIdx = Demos::DEMO_WEAR_HAT;
      isplaybackSimulationOn = false;
      backwardSelectedRecordId = 0;
      forwardStartSimulationButton->setEnabled(false);
      forwardStopSimulationButton->setEnabled(false);
      playbackSimulationButton->setEnabled(false);
      backwardPlayGroundtruthSimulationButton->setEnabled(false);
  });





  SettingsControl->addButton("Demo Wear Sock", [&]() {
      isSimulationOn = false;
      isforward = false;
      backwardNeedsInitiation = true;
      selectedDemoIdx = Demos::DEMO_WEAR_SOCK;
      isplaybackSimulationOn = false;
      backwardSelectedRecordId = 0;
      forwardStartSimulationButton->setEnabled(false);
      forwardStopSimulationButton->setEnabled(false);
      playbackSimulationButton->setEnabled(false);
      backwardPlayGroundtruthSimulationButton->setEnabled(false);
  });





  backwardPlayRecordButton = SettingsControl->addButton("Play record", [&]() {
      if (backwardSelectedRecordId < simSystems[0]->backwardOptimizationRecords.size()) {
        isplaybackSimulationOn = true;
        playGroundtruth = false;
        playLineSearch = false;
        playBackId = 0;

        Simulation::BackwardInformation &info = simSystems[0]->backwardOptimizationRecords[backwardSelectedRecordId].second[0];
      }
  });

  backwardPlayRecordButton->setEnabled(false);
  backwardPlayRecordButton->setIcon(ENTYPO_ICON_CONTROLLER_PLAY);
  backwardPlayGroundtruthSimulationButton = SettingsControl->addButton("Play groundtruth", [&]() {
      isplaybackSimulationOn = true;
      playGroundtruth = true;
      playLineSearch = false;
      playBackId = 0;
  });
  backwardPlayGroundtruthSimulationButton->setEnabled(false);
  backwardPlayGroundtruthSimulationButton->setIcon(ENTYPO_ICON_CONTROLLER_PLAY);


  nanogui::Button * playLineSearchButton = SettingsControl->addButton("Play linesearch", [&]() {
      isplaybackSimulationOn = true;
      playLineSearch = true;
      playGroundtruth = false;
      playBackId = 0;
  });

  playLineSearchButton->setEnabled(true);
  playLineSearchButton->setIcon(ENTYPO_ICON_CONTROLLER_PLAY);
  SettingsControl->addVariable<bool>("vis/w/gt",
                                     renderWithGroundTruth, true);


  SettingsControl->addButton("Save Optimization Record", [&]() {
      simSystems[0]->exportOptimizationRecords((Demos) selectedDemoIdx);
  });
}

void Viewer::addForwardSimulationControlWidets() {
  using namespace nanogui;
  forwardSimControlWindow = SettingsControl->addWindow(Eigen::Vector2i(width - 250, 0), "Forward Cloth Simulation");

  simulationInfoTextBox = new TextBox(forwardSimControlWindow);
  simulationInfoTextBox->setValue("Total Timestep: " + std::to_string(simSystems[0]->forwardRecords.size()));
  simulationInfoTextBox->setFontSize(20);
  simulationInfoTextBox->setAlignment(TextBox::Alignment::Left);
  simulationInfoTextBox->setEnabled(true);
  simulationInfoTextBox->setVisible(true);
  SettingsControl->addWidget("", simulationInfoTextBox);

  SettingsControl->addButton("Reset", [&]() {
    std::printf("ResetButton called");
    resetAllSimulation(); })->setTooltip(
          "Resetting the simulation to time 0 without changing configuration");
  forwardStartSimulationButton = SettingsControl->addButton("Start Simulation", [&]() { isSimulationOn = true; });
  forwardStopSimulationButton = SettingsControl->addButton("Stop Simulation", [&]() { isSimulationOn = false; });
  playbackSimulationButton = SettingsControl->addButton("Playback Simulation", [&]() {
      isplaybackSimulationOn = true;
      playBackId = 0;
      forwardFrameSliderText.slider->setEnabled(false);
      forwardStopSimulationButton->setEnabled(false);
      forwardStartSimulationButton->setEnabled(false);

  });


  Widget *panel = new Widget(forwardSimControlWindow);
  panel->setLayout(new BoxLayout(nanogui::Orientation::Horizontal,
                                 Alignment::Middle, 0, 10));
  SettingsControl->addWidget("", panel);

  forwardFrameSliderText.slider = new Slider(panel);
  forwardFrameSliderText.slider->setValue(0.0f);
  forwardFrameSliderText.slider->setFixedWidth(80);
  forwardFrameSliderText.idx = 0;
  forwardFrameSliderText.idxWidget = SettingsControl->addVariable<int>("",
                                                                       [&](const int &v) {
                                                                           int totalSize = ((int) simSystems[0]->forwardRecords.size());
                                                                           double val = 0;
                                                                           std::string text;
                                                                           idxToFloatAndText(v, totalSize, val, text);
                                                                           forwardFrameSliderText.slider->setValue(val);
                                                                           forwardFrameSliderText.slider->callback()(
                                                                                   val);
                                                                       },
                                                                       [&]() -> int {
                                                                           return forwardFrameSliderText.idx;
                                                                       }, panel
  );
  forwardFrameSliderText.idxWidget->setMinValue(0);
  formWidgetSetSpinnableWithIncrement(forwardFrameSliderText.idxWidget, 1);

  forwardFrameSliderText.textbox = new TextBox(panel);
  forwardFrameSliderText.textbox->setFixedSize(Vector2i(60, 25));
  forwardFrameSliderText.textbox->setValue("1/" + std::to_string(simSystems[0]->forwardRecords.size()));
  forwardFrameSliderText.textbox->setUnits("");
  forwardFrameSliderText.slider->setCallback([&](float value) {
      if (!isforward)
        return;
      std::vector<Simulation::ForwardInformation> forwardRecordCopy = simSystems[0]->forwardRecords;

      int frame = 0;
      std::string text;
      floatToIdxAndText(value, forwardRecordCopy.size(), frame, text);
      playBackId = frame;
      forwardFrameSliderText.idx = playBackId;
      forwardFrameSliderText.idxWidget->setValue(playBackId);
      forwardFrameSliderText.textbox->setValue(text);
      if ((playBackId >= 0) && (playBackId < forwardRecordCopy.size())) {
        forwardInfo = forwardRecordCopy[playBackId];
        forwardInfoWindowContentUpdate(forwardInfoWidgets, forwardInfo);
      }

  });

  forwardFrameSliderText.textbox->setFixedSize(Vector2i(60, 25));
  forwardFrameSliderText.textbox->setFontSize(15);
  forwardFrameSliderText.textbox->setAlignment(TextBox::Alignment::Right);

  forwardStartSimulationButton->setCallback([&]() {
      isSimulationOn = true;
      forwardFrameSliderText.slider->setEnabled(false);
  });

  forwardStopSimulationButton->setCallback([&]() {
      isSimulationOn = false;
      forwardFrameSliderText.slider->setEnabled(true);
      playBackId = ((int) simSystems[0]->forwardRecords.size()) - 1;
      forwardFrameSliderText.slider->setValue(1.0);
      forwardFrameSliderText.slider->callback()(1.0);

  });

  fileNameTextBox = new nanogui::TextBox(forwardSimControlWindow);
  fileNameTextBox->setValue("example_output_mesh.txt");
  fileNameTextBox->setFontSize(15);
  fileNameTextBox->setAlignment(nanogui::TextBox::Alignment::Right);
  fileNameTextBox->setEditable(true);
  SettingsControl->addWidget("fileName", fileNameTextBox);

  SettingsControl->addButton("Save Mesh Position", [&]() {
  simSystems[0]->exportCurrentMeshPos(playBackId, fileNameTextBox->value());
  });

  SettingsControl->addButton("Output Animation", [&]() {
      simSystems[0]->exportCurrentSimulation(fileNameTextBox->value());
  });

//  SettingsControl->addButton("Load Animation", [&]() {
//
//      std::string folder = std::string(SOURCE_PATH) + fileNameTextBox->value();
//      frameFileNames = listFiles(folder, ".obj");
//
//      std::sort(frameFileNames.begin(), frameFileNames.end());
//
//      simSystems[0]->loadWindSim2RealAnimationSequence(selectedFolder, frameFileNames,
//                                                       selectedFolder == "flag-ryanwhite");
//      camera.setCamPos(toGlm(simSystems[0]->sceneConfig.camPos));
//      camera.setLookAt(Simulation::getLookAtPos(simSystems[0], simSystems[0]->sceneConfig));
//      isSimulationOn = false;
//      isforward = true;
//      isplaybackSimulationOn = false;
//
//  });


}

void Viewer::addReinitWidgets() {
  reinitWindow = SettingsControl->addWindow(Eigen::Vector2i(30, 500), "Reinit Settings");
  SettingsControl->addGroup("Reinit Settings");

  // load mesh from directory
  nanogui::Widget*  panel = new  nanogui::Widget(reinitWindow);
  panel->setLayout(new nanogui::GridLayout(nanogui::Orientation::Horizontal, 4, nanogui::Alignment::Middle,
                                           0, 0));
  SettingsControl->addWidget("", panel);

  dimControl = SettingsControl->addVariable<int>("dimension", meshDim,   panel);
  dimControl->setFixedWidth(60);
  densityControl = SettingsControl->addVariable<double>("density", density,  panel);
  densityControl->setFixedWidth(60);
  formWidgetSetSpinnableWithIncrement(dimControl, 1);

  // stretching
  strechingStiffnessControl = SettingsControl->addVariable<double>("k_stretch",
                                                                   strechingStiffness, panel);
  strechingStiffnessControl->setFixedWidth(60);
  formWidgetSetSpinnableWithIncrement(strechingStiffnessControl, 5.0);
  // bending
  bendingStiffnessControl = SettingsControl->addVariable<double>("k_bend",
                                                                 bendingStiffness, panel);
  bendingStiffnessControl->setFixedWidth(60);
  formWidgetSetSpinnableWithIncrement(bendingStiffnessControl, 0.1);

  formWidgetSetSpinnableWithIncrement(densityControl, 0.1);

  SettingsControl->addButton("Change Material Config", [&]() {
      std::printf("Change Material Config Button callBack#####");
      Simulation::FabricConfiguration config = simSystems[0]->sceneConfig.fabric;
      config.k_stiff_stretching = strechingStiffness;
      config.k_stiff_bending = bendingStiffness;
      config.density = density;
      reinitFabric(config);
      resetAllSimulation();
  });






  // load animations
  animationOptions = listDirectory(std::string(SOURCE_PATH) + "/src/assets/animation/");

  std::sort(animationOptions.begin(), animationOptions.end());

  boxAnimationFiles = new nanogui::ComboBox(reinitWindow, animationOptions);
  SettingsControl->addWidget("Animations", boxAnimationFiles);

  std::vector<std::string> demoOptions;
  for (Simulation::SceneConfiguration &s : OptimizationTaskConfigurations::sceneConfigArrays) {
    demoOptions.push_back(s.name);
  }
  SettingsControl->addVariable("DemoConfig", currentSceneConfig, true)->setItems(demoOptions);

  SettingsControl->addButton("WindSim2Real", [&]() {
      std::vector<std::string> frameFileNames;
      std::string selectedFolder = animationOptions[boxAnimationFiles->selectedIndex()];
      std::string folder = std::string(SOURCE_PATH) + "/src/assets/animation/" + selectedFolder;
      frameFileNames = listFiles(folder, ".obj");

      std::sort(frameFileNames.begin(), frameFileNames.end());

      simSystems[0]->loadWindSim2RealAnimationSequence(selectedFolder, frameFileNames,
                                                       selectedFolder == "flag-ryanwhite");
      camera.setCamPos(toGlm(simSystems[0]->sceneConfig.camPos));
      camera.setLookAt(Simulation::getLookAtPos(simSystems[0], simSystems[0]->sceneConfig));
      isSimulationOn = false;
      isforward = true;
      isplaybackSimulationOn = false;

  });


  SettingsControl->addButton("Reinit Demo", [&]() {
      std::printf("Reinit Demo\" Button callBack#####");
      reinitWithSceneFabric();
      resetAllSimulation();
      camera.setCamPos(toGlm(simSystems[0]->sceneConfig.camPos));
      camera.setLookAt(Simulation::getLookAtPos(simSystems[0], simSystems[0]->sceneConfig));
  });
}
