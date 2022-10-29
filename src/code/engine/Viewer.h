//
// Created by liyifei@csail.mit.edu on 7/18/18.
//

#ifndef OMEGAENGINE_VIEWER_H
#define OMEGAENGINE_VIEWER_H

#define GL_SILENCE_DEPRECATION
#include "Macros.h"

#include <nanogui/nanogui.h>
#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/button.h>
#include <nanogui/toolbutton.h>
#include <nanogui/popupbutton.h>
#include <nanogui/combobox.h>
#include <nanogui/progressbar.h>
#include <nanogui/entypo.h>
#include <nanogui/messagedialog.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/imagepanel.h>
#include <nanogui/imageview.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/colorwheel.h>
#include <nanogui/colorpicker.h>
#include <nanogui/graph.h>
#include <nanogui/tabwidget.h>

#include "../simulation/Simulation.h"
#include "../optimization/OptimizationTaskConfigurations.h"

#include "Camera.h"
#include "../optimization/BackwardTaskSolver.h"


class Viewer {
public:
    GLFWwindow *glfWwindow;
    Camera camera;

    nanogui::TextBox *fpsTextBox = nullptr, *fileNameTextBox;


    nanogui::Button *forwardStartSimulationButton = nullptr, *playbackSimulationButton = nullptr, *forwardStopSimulationButton = nullptr, *backwardPlayGroundtruthSimulationButton = nullptr, *backwardPlayRecordButton = nullptr;
    nanogui::TextBox *simulationInfoTextBox = nullptr, *infoTextBox = nullptr,  *backwardInfo1 = nullptr, *backwardInfo2 = nullptr;
    std::vector<std::string>   animationOptions;
    nanogui::ComboBox *boxAnimationFiles = nullptr;
    nanogui::ComboBox *boxMeshFiles = nullptr;
    nanogui::FormHelper *SettingsControl = nullptr;
    nanogui::detail::FormWidget<int> *timeStepSetter, *dimControl, *precisionSetter;
    nanogui::detail::FormWidget<double> *strechingStiffnessControl, *bendingStiffnessControl, *densityControl;
    nanogui::ref<nanogui::Window> mainConfigWindow, forwardSimControlWindow, reinitWindow, backwardSimControlWindow, backwardSimInfoWindow, forwardInfoWindow, backwardSimFiniteDiffInfoWindow, particleInfoWindow;
    nanogui::Screen *screen = nullptr;
    std::vector<nanogui::detail::FormWidget<float> *> cameraWidgets;
    nanogui::detail::FormWidget<int> *stepLimit;
    std::vector<Simulation *> simSystems;
    std::vector<nanogui::TextBox *> simSystemNameBoard;
    std::vector<Vec3d> symPosCenters;
    bool useFiniteDiff = false, useLBFGS = false, randomInit = true, perfTest = false, directBackward = false;
    nanogui::detail::FormWidget<int> *perfResolutionWidget;
    struct SliderTextBoxWidgets {
        nanogui::Slider *slider;
        nanogui::TextBox *textbox;
        nanogui::detail::FormWidget<int> *idxWidget;
        int idx;
    };

    SliderTextBoxWidgets backwardRecordSliderText, backwardFrameSliderText, forwardFrameSliderText;

    struct BackwardGradientInfoWidgets {
        nanogui::detail::FormWidget<double> *stiffness[2]; // stiffness and bending
        nanogui::detail::FormWidget<double> *f_ext[3];
        nanogui::detail::FormWidget<double> *f_wind[5];
        std::vector<std::vector<nanogui::detail::FormWidget<double> *>> dL_dspline;
        nanogui::detail::FormWidget<double> *density;
        nanogui::detail::FormWidget<double> *x0;
        nanogui::detail::FormWidget<double> *mu;
        nanogui::detail::FormWidget<double> *loss;
        nanogui::detail::FormWidget<bool> *converged;
        nanogui::detail::FormWidget<double> *rho;
        nanogui::detail::FormWidget<long long> *perfTimer[3];
        nanogui::detail::FormWidget<long long> *accumPerfTimer[3];
        nanogui::detail::FormWidget<int> *convergeIter;
        nanogui::detail::FormWidget<int> *accumConverge;
        nanogui::detail::FormWidget<int> *timerReportIdxControl;
        nanogui::detail::FormWidget<long long> *totalTime;

        int timerId;
        nanogui::TextBox *timerReport;
        nanogui::TextBox *accumTimerReport;


    };


    struct ForwardInfoWidgets {
        nanogui::detail::FormWidget<int> *convergeIter;
        nanogui::detail::FormWidget<int> *accumulateIter;
        nanogui::detail::FormWidget<int> *selfCollisionNumber;
        nanogui::detail::FormWidget<int> *externalCollisionNumber;
        nanogui::detail::FormWidget<double> *avgDeformation;
        nanogui::detail::FormWidget<double> *totalTime;
        nanogui::detail::FormWidget<double> *iterTime;
        nanogui::detail::FormWidget<bool> *converged;
        nanogui::detail::FormWidget<int> *timerReportIdxControl;
        int timerId;
        nanogui::TextBox *timerReport;
        nanogui::TextBox *accumTimerReport;

    };
    struct ParticleInfoWidgets {
        nanogui::detail::FormWidget<double> *mass; // stiffness and bending
        nanogui::detail::FormWidget<double> *area;
        nanogui::detail::FormWidget<int> *particleIdxControl;
        nanogui::detail::FormWidget<int> *neighborIdxToggle;
        int neighborId;
        nanogui::TextBox *triangles;
        nanogui::Button *focus;
        nanogui::Button *triangleNeighborFocus;
    };

    struct TriangleInfoWidgets {
        nanogui::detail::FormWidget<int> *triangleIdxControl;
        nanogui::detail::FormWidget<int> *particleId[3];
        nanogui::Button *focusButtons[3];
        nanogui::detail::FormWidget<double> *area;
        nanogui::Button *focus;
    };
    ParticleInfoWidgets particleInfoWidgets;
    TriangleInfoWidgets triangleInfoWidgets;
    BackwardGradientInfoWidgets gradientInfoWidgets = {}, finiteDiffInfoWidgets = {};
    ForwardInfoWidgets forwardInfoWidgets;
    Simulation::ForwardInformation forwardInfo;
    Simulation::BackwardInformation backwardInfo, finitediffGradientInfo = {};


    bool isSimulationOn = false, isplaybackSimulationOn = false, isforward = true, backwardNeedsInitiation = false, backwardRunning = false, playGroundtruth = false, playLineSearch = false, renderWithGroundTruth = false;
    int selectedDemoIdx = 0;
    int playBackId = 0, backwardSelectedRecordId = -1, particleId = 0, triangleId = 0;
    int meshDim = 7;
    double strechingStiffness = 50.0, bendingStiffness = 0.0, wireframeThickness = 0.07;
    std::string meshFileName = "";
    double density = 0.324;


    nanogui::Color color_custom = nanogui::Color(0.191, 0.547238, 0.738, 1.f);
    nanogui::Color color_doubleside_1 = nanogui::Color(1.0f, 0.45f, 0.0f, 1.f);
    nanogui::Color color_doubleside_2 = nanogui::Color(0.5f, 0.5f, 0.7f, 1.f);
    nanogui::Color color_background = nanogui::Color(0.2022f, 0.2022f, 0.2022f, 1.f);
    nanogui::Color red = nanogui::Color(1.0f, 0.0f, 0.0f, 1.f);

    SceneConfigArray currentSceneConfig;
    nanogui::Widget *panel;
    std::vector<std::string> renderModeStrings = std::vector<std::string>{"FABRIC_COLOR", "SOLID_CUSTOM", "WIREFRAME",
                                                                          "WIREFRAME_COLORED",
                                                                          "SOLID-2SIDES",
                                                                          "DEFORMATION",
                                                                          "NORMAL_MAP", "RANDOM_COLOR"};
    enum RenderMode {
        FABRIC_COLOR, SOLID_CUSTOM, WIREFRAME, WIREFRAME_COLORED, SOLID_TWOSIDES, DEFORMATION, NORMAL_MAP, RANDOM_COLOR
    };

    RenderMode myRendermode = FABRIC_COLOR;
    bool visualizeCollision = false;
    bool visualizeSelfCollisionLayers = false;
    bool collisionShowNormal = false;
    int selfCollisionLayerVisId = 0, perfResolution = 0;
    bool splineVisualization = false;
    bool perStepTrajectoryVisualization = true;
    bool perStepGradientVisualization = true;
    bool renderAxis = true;
    bool renderLights = false;
    bool renderGrid = true;
    bool renderAABB = true;
    bool isSimpleViewer= false;
    bool renderPosPairs = false;

    void updateProjectionMatrix() {
      projectionMat = glm::perspective(glm::radians(dof), width * 1.0f / height, zNear, zFar);

    }

    Viewer(bool isSimpleViewer = false) : width(1600), height(1200), lastX(400), lastY(300), zNear(0.1), zFar(200), firstMouse(true), dof(60.0), isSimpleViewer(isSimpleViewer) {
      camera = Camera();
      updateProjectionMatrix();
    };

    GLFWwindow *init();
    GLFWwindow *newWindow();

    void updateCameraDependentWidgetPoses();

    std::vector<nanogui::Vector2i> modelToScreen(std::vector<Vec3d> &poses);

    void checkPlaybackFinishedCallback();

    void backwardSliderChecker();

    void resetAllSimulation();


    void gradientInfoWindowContentUpdate(BackwardGradientInfoWidgets &gradientWidgets,
                                         Simulation::BackwardInformation &gradient, bool isFiniteDiff);

    void forwardInfoWindowContentUpdate(ForwardInfoWidgets &forwardWidgets,
                                        Simulation::ForwardInformation &forwardInfo);

    void checkBackwardPlaybackFinishedCallback();

    void reinitWithSceneFabric() {
      Simulation::SceneConfiguration config = OptimizationTaskConfigurations::sceneConfigArrays[currentSceneConfig];

      stepLimit->callback()(std::to_string(config.stepNum));

      for (Simulation* system : simSystems) {
        system->sceneConfig = config;

      }

      timeStepSetter->setValue(1.0 / simSystems[0]->sceneConfig.timeStep);
      strechingStiffnessControl->setValue(config.fabric.k_stiff_stretching);
      strechingStiffness = config.fabric.k_stiff_stretching;

      bendingStiffnessControl->setValue(config.fabric.k_stiff_bending);
      bendingStiffness = config.fabric.k_stiff_bending;
      densityControl->setValue(config.fabric.density);
      dimControl->setValue(config.fabric.clothDimX);
      for (Simulation *s : simSystems) {
        s->createClothMesh();
        s->initScene();
      }

      std::printf("finished reinit; reset camera\n");

      camera.setLookAt(Simulation::getLookAtPos(simSystems[0], simSystems[0]->sceneConfig));
      camera.setCamPos(glm::vec3(simSystems[0]->sceneConfig.camPos[0], simSystems[0]->sceneConfig.camPos[1],
                                 simSystems[0]->sceneConfig.camPos[2]));


    }

    void reinitScene() {
      std::printf("Reinit Scene called\n");
      for (Simulation *s : simSystems) {
        s->initScene();
      }
      camera.setLookAt(Simulation::getLookAtPos(simSystems[0], simSystems[0]->sceneConfig));

    }

    void reinitFabric(Simulation::FabricConfiguration config) {
      for (Simulation *s : simSystems) {
        s->sceneConfig.fabric = config;
        s->createClothMesh();
        s->initScene();
      }


      strechingStiffnessControl->setValue(config.k_stiff_stretching);
      strechingStiffness = config.k_stiff_stretching;

      bendingStiffnessControl->setValue(config.k_stiff_bending);
      bendingStiffness = config.k_stiff_bending;

      densityControl->setValue(config.density);
      density = config.density;

      dimControl->setValue(config.clothDimX);
      meshDim = config.clothDimX;
      camera.setLookAt(Simulation::getLookAtPos(simSystems[0], simSystems[0]->sceneConfig));

    }

    void mouse_callback(double xpos, double ypos);

    void startMainLoop();

    void processInput();

    void updateOnScreenInfos();

    nanogui::Vector2i modelToScreen(Vec3d &pose) {
      mat4 Vmax = camera.getViewMat();
      mat4 Pmax = getProjectionMat();
      mat4 PV = Pmax * Vmax;
      nanogui::Vector2i res;
      Vec3d &s = pose;
      glm::vec4 center = glm::vec4(s[0], s[1], s[2], 1);
      glm::vec4 transformed = PV * center;
      double x = (transformed.x / transformed.w);
      double y = (transformed.y / transformed.w);
      double z = (transformed.z / transformed.w);
      res = nanogui::Vector2i((int) ((x * 0.5 + 0.5) * width), (int) (((-y) * 0.5 + 0.5) * height));
      return res;
    }

    void updateInfoTextBox(Vec3d modelPos, std::string value) {
      infoTextBox->setEditable(true);
      infoTextBox->setPosition(modelToScreen(modelPos));
      infoTextBox->setValue(value);

    }

    glm::mat4 getProjectionMat() {
      return projectionMat;
    }

    float zNear;
    float zFar;
    int width;
    int height;
    float dof;

    void addSystems(std::vector<Simulation *> systems);
    void addSystem(Simulation * system);
private:
    volatile float lastTime = 0.0f;
    volatile int frameNum = 0;
    volatile float lastFrame = 0.0f;

    float lastX;
    float lastY;

    glm::mat4 projectionMat;
    bool firstMouse;

    int initalizeGLAD();

    void setViewport();

    void setupNanogui();


    void addForwardSimulationControlWidets();

    void addBackwardSimulationControlWidets();

    void addParticleInfoWidgets();

    void updateParticleInfoContent();

    void addBackwardSimulationInfoWidets(std::string title, BackwardGradientInfoWidgets &infoWidgets,
                                         Simulation::BackwardInformation &gradientInfo,
                                         nanogui::ref<nanogui::Window> window, int widthOffset, bool isFiniteDiff);

    void addForwardSimulationInfoWidets(std::string title, ForwardInfoWidgets &infoWidgets,
                                        Simulation::ForwardInformation &forwardInfo,
                                        nanogui::ref<nanogui::Window> window, int widthOffset);

    void addReinitWidgets();

    void addCameraControlWidgets();

    void addSceneControlWidgets();

    void addTextBoxWidgets();


};


#endif //OMEGAENGINE_VIEWER_H
