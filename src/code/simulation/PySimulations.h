//
// Created by Yifei Li on 4/29/21.
// Email: liyifei@csail.mit.edu
//

#ifndef OMEGAENGINE_PYTHONSIMULATIONS_H
#define OMEGAENGINE_PYTHONSIMULATIONS_H

#include "Simulation.h"
#include "../engine/Macros.h"
#include "../engine/RenderLoop.h"
#include "../optimization/OptimizeHelper.h"
#include "../optimization/OptimizationTaskConfigurations.h"

class PySimulations {
public:




    static void runExample() {


        /** A SceneConfiguration struct stores all configs, this includes
         * @param FabricConfiguration: configs for the fabric, including the mesh model's path, color, dimensions, initial pos files etc
         * @param PrimitiveConfiguration: external objects in the scene
         * @param timeStep: the step size
         * @param stepNum: number of steps the demo should run
         *
         * After creating the system, this config will be stored in Simulation::sceneConfig
         *
         * @param runBackward needs be set to true if you intend to run the backward pass.
         */
        Simulation::SceneConfiguration initSceneProfile = OptimizationTaskConfigurations::hatScene;
        Simulation *clothSystem = Simulation::createSystem(
                                                           initSceneProfile,
                                                           Vec3d(0, 0, 0), true);

        clothSystem->sceneConfig.trajectory = TrajectoryConfigs::PER_STEP_TRAJECTORY;

        int N = 200;// Simulation::sceneConfig.stepNum;

        // This command resets a simulation system to its initial stage
        clothSystem->resetSystem();

        std::cerr << clothSystem->forwardRecords[clothSystem->forwardRecords.size() - 1].x.transpose() << std::endl;
        
        // All ran simulation are stored in Simulation::forwardRecords. Each record stores the complete state detail of the simulation
        std::printf("At intiializaiton, forwardRecords.size() is %zu\n", clothSystem->forwardRecords.size());

        // Running forward simulation by stepping
        std::printf("Forward pass:\n");
        int fixedPointNumber = clothSystem->sysMat[clothSystem->currentSysmatId].fixedPoints.size();
        VecXd dir(fixedPointNumber * 3), ones(fixedPointNumber * 3);
        dir.setOnes();
        ones.setOnes();
        dir = dir / Vec3d::Ones().norm();

        VecXd fixedPointInitPoses = clothSystem->forwardRecords[0].x_fixedpoints;
        Vec3d translation(0, 0, 0);
        VecXd translationAll(fixedPointInitPoses.rows());
        // For the hat task, there are two fixed points.
        for (int i = 0; i < N; i++) {
            std::printf("%d..", i);
            int fixedPointNumber = clothSystem->sysMat[clothSystem->currentSysmatId].fixedPoints.size();
            if (i < N / 2)
                translation[0] += 0.03;
            else
                translation[1] += 0.03;
            for (int fixedPIdx = 0; fixedPIdx < fixedPointNumber; fixedPIdx++) {
                translationAll.segment(fixedPIdx * 3, 3) = translation;
            }
            clothSystem->rlFixedPointPos = fixedPointInitPoses + translationAll;
            dir.setRandom();
            dir = dir + ones;
            dir = dir / 2.0;
            clothSystem->perStepGradient.emplace_back(dir);
            clothSystem->step();
        }
        std::printf("\n");



        RenderLoop::renderRecordsForSystem(clothSystem, clothSystem->forwardRecords, true, "Visualization for pySimulations::runExample");


        // After simulation, each step pushes a record to
        std::printf("After forward simulation with %d steps, forwardRecords.size() is %zu\n", N,
                    clothSystem->forwardRecords.size());

//        for (int i = 0; i < N; i += 15)
//            clothSystem->exportCurrentMeshPos(i, "wearhat-" + std::to_string(i));



        // For the backward pass, first set task information properly
        Simulation::LossInfo lossInfo;
        Simulation::BackwardTaskInformation taskInfo = {.dL_dcontrolPoints = true};
        LossType lossType = LossType::ASSISTED_DRESSING_KEYPOINTS;


        // Run backward pass. The backward pass resets the system, run forward pass with the initialParameters, calculate its loss, and calclate the gradients. Gradients are returned
        // as a vector of BackwardInformation forwardRecords[i] steps the system from [x,v]_{i-1} --> [x,v]_{i}, while backwardRecords[i] backprops the system from [x,v]_{i} --> [x,v]_{i-1}
//      std::vector<Simulation::BackwardInformation> backwardRecords = clothSystem->runBackwardTask(taskInfo,
//                                                                                                  lossType,
//                                                                                                  lossInfo,
//                                                                                                  N,
//                                                                                                  initialParameters,
//                                                                                                  false);
      // Backproped control points gradient is at backwardRecords[0].dL_dsplines

      delete clothSystem;
    }


    static void runExample2() {
//      Simulation::forwardConvergenceThreshold = 1e-5;
      Simulation::SceneConfiguration initSceneProfile = OptimizationTaskConfigurations::hatScene;
      Simulation *clothSystem = Simulation::createSystem(
                                                         initSceneProfile,
                                                         Vec3d(0, 0, 0), false);

      OptimizeHelper helper = BackwardTaskSolver::getOptimizeHelper(clothSystem, Demos::DEMO_WEAR_SOCK);
//      OptimizeHelper helper = BackwardTaskSolver::getOptimizeHelper(clothSystem, Demos::DEMO_WEAR_HAT);

      for (int i = 0; i < 3; i++) {
        VecXd randomInitPrams = helper.getRandomParam(time(NULL));
        std::cout << "random parameters:" << randomInitPrams.transpose() << "\n";
        Simulation::ParamInfo paramInfo = helper.vecXdToParamInfo(randomInitPrams);
        std::printf("Corresponds to info: %s\n", Simulation::parameterToString(helper.taskInfo, paramInfo).c_str());


        std::printf("running simulation...\n");
        helper.taskInfo.forwardAccuracyLevel = 1e-7;
        double loss = helper.runSimulationAndGetLoss(randomInitPrams);
        Logging::logColor("Loss is" + d2str(loss, 3) + "\n", Logging::BLUE);
        std::printf("running rendering...\n");

        RenderLoop::renderRecordsForSystem(clothSystem, clothSystem->forwardRecords, false, false,
                                           "Set text here for whatever you need (only single line is supported): Visualization for pySimulations::runExample");
      }


    }


    static void runSec6_1_SydId_Wind_Tshirt() {
      Simulation::SceneConfiguration initSceneProfile = OptimizationTaskConfigurations::tshirtScene;
      Simulation *clothSystem = Simulation::createSystem(
                                                         initSceneProfile,
                                                         Vec3d(0, 0, 0), false);

      Simulation::forwardConvergenceThreshold = 1e-5;
      OptimizeHelper helper = BackwardTaskSolver::getOptimizeHelper(clothSystem, Demos::DEMO_WIND_TSHIRT);
//      OptimizeHelper helper = BackwardTaskSolver::getOptimizeHelper(clothSystem, Demos::DEMO_WEAR_HAT);

      for (int i = 0; i < 3; i++) {
        VecXd randomInitPrams = helper.getRandomParam(time(NULL));
        std::vector<bool> needsLogScale = helper.paramLogScaleTransformOn;
        std::printf("Log scale transoformation:\n");
        for (bool v : needsLogScale)
          std::printf("%d,", v);
        std::printf("\n");
        std::cout << "random parameters:" << randomInitPrams.transpose() << "\n";
        Simulation::ParamInfo paramInfo = helper.vecXdToParamInfo(randomInitPrams);
        std::printf("Corresponds to info: %s\n", Simulation::parameterToString(helper.taskInfo, paramInfo).c_str());


        std::printf("running simulation...\n");
        double loss = helper.runSimulationAndGetLoss(randomInitPrams);
        std::printf("Loss is %.5f\n", loss);
        std::printf("running rendering...\n");
        RenderLoop::renderRecordsForSystem(clothSystem, clothSystem->forwardRecords, false, false,
                                           "Set text here for whatever you need (only single line is supported): Visualization for pySimulations::runExample");
      }
    }


    static void  runSec6_3_inverseDesign() {
      Simulation::SceneConfiguration initSceneProfile = OptimizationTaskConfigurations::tshirtScene;
      Simulation *clothSystem = Simulation::createSystem(
                                                         initSceneProfile,
                                                         Vec3d(0, 0, 0), false);

      Simulation::forwardConvergenceThreshold = 1e-9;
      OptimizeHelper helper = BackwardTaskSolver::getOptimizeHelper(clothSystem, Demos::DEMO_DRESS_TWIRL);
//      OptimizeHelper helper = BackwardTaskSolver::getOptimizeHelper(clothSystem, Demos::DEMO_WEAR_HAT);

        // std::cerr << "here" << std::endl;
      for (int i = 0; i < 3; i++) {
        VecXd randomInitPrams = helper.getRandomParam(8888);
        // VecXd randomInitPrams = helper.getRandomParam(0);
        std::vector<bool> needsLogScale = helper.paramLogScaleTransformOn;
        std::printf("Log scale transoformation:\n");
        for (bool v : needsLogScale)
          std::printf("%d,", v);
        std::printf("\n");
        std::cout << "random parameters:" << randomInitPrams.transpose() << "\n";
        Simulation::ParamInfo paramInfo = helper.vecXdToParamInfo(randomInitPrams);
        std::printf("Corresponds to info: %s\n", Simulation::parameterToString(helper.taskInfo, paramInfo).c_str());


        std::printf("running simulation...\n");
        double loss = helper.runSimulationAndGetLoss(randomInitPrams);
        std::printf("Loss is %.5f\n", loss);
        std::printf("running rendering...\n");
        RenderLoop::renderRecordsForSystem(clothSystem, clothSystem->forwardRecords, false, false,
                                           "Set text here for whatever you need (only single line is supported): Visualization for pySimulations::runExample");
      }
    }

    static void runSec6_4_sim2real() {
        Simulation::SceneConfiguration initSceneProfile = OptimizationTaskConfigurations::tshirtScene;
        Simulation *clothSystem = Simulation::createSystem(
                                                           initSceneProfile,
                                                           Vec3d(0, 0, 0), false);

        Simulation::forwardConvergenceThreshold = 1e-9;
        OptimizeHelper helper = BackwardTaskSolver::getOptimizeHelper(clothSystem, Demos::DEMO_WIND_SIM2REAL);
//      OptimizeHelper helper = BackwardTaskSolver::getOptimizeHelper(clothSystem, Demos::DEMO_WEAR_HAT);

        for (int i = 0; i < 3; i++) {
          Simulation::forwardConvergenceThreshold = 1e-5;

          VecXd randomInitPrams = helper.getRandomParam(4444);
          std::vector<bool> needsLogScale = helper.paramLogScaleTransformOn;
          std::printf("Log scale transoformation:\n");
          for (bool v : needsLogScale)
            std::printf("%d,", v);
          std::printf("\n");
          std::cout << "random parameters:" << randomInitPrams.transpose() << "\n";
          Simulation::ParamInfo paramInfo = helper.vecXdToParamInfo(randomInitPrams);
          std::printf("Corresponds to info: %s\n", Simulation::parameterToString(helper.taskInfo, paramInfo).c_str());

          Simulation::forwardConvergenceThreshold = 1e-5;

          std::printf("running simulation...\n");
          double loss = helper.runSimulationAndGetLoss(randomInitPrams);
          std::printf("Loss is %.5f\n", loss);
          std::printf("running rendering...\n");
          RenderLoop::renderRecordsForSystem(clothSystem, clothSystem->forwardRecords, false, false,
                                             "Set text here for whatever you need (only single line is supported): Visualization for pySimulations::runExample");
        }
      }


      static void runSec6_1_sphere() {

        Simulation::SceneConfiguration initSceneProfile = OptimizationTaskConfigurations::tshirtScene;
        Simulation *clothSystem = Simulation::createSystem(
                                                           initSceneProfile,
                                                           Vec3d(0, 0, 0), false);

        Simulation::forwardConvergenceThreshold = 1e-9;
        OptimizeHelper helper = BackwardTaskSolver::getOptimizeHelper(clothSystem, Demos::DEMO_SPHERE_ROTATE);
        std::printf("Total parameters: %d\n", helper.totalParamNumber);
        std::printf("Total frames: %d\n", helper.system->sceneConfig.stepNum);
        std::printf("timestep: %.4f / 1/%d\n", helper.system->sceneConfig.timeStep,  (int) (1.0 / helper.system->sceneConfig.timeStep) );

//      OptimizeHelper helper = BackwardTaskSolver::getOptimizeHelper(clothSystem, Demos::DEMO_WEAR_HAT);

       std::printf("mass particle 0: %.5f\n", helper.system->particles[0].mass);
        for (int i = 0; i < 3; i++) {
          std::printf("timestep: %.4f / 1/%d\n", helper.system->sceneConfig.timeStep,  (int) (1.0 / helper.system->sceneConfig.timeStep) );

          VecXd randomInitPrams = helper.getRandomParam(8888);
          std::vector<bool> needsLogScale = helper.paramLogScaleTransformOn;
          std::printf("Log scale transoformation:\n");
          for (bool v : needsLogScale)
            std::printf("%d,", v);
          std::printf("\n");
          std::cout << "random parameters:" << randomInitPrams.transpose() << "\n";
          Simulation::ParamInfo paramInfo = helper.vecXdToParamInfo(randomInitPrams);
          std::printf("Corresponds to info: %s\n", Simulation::parameterToString(helper.taskInfo, paramInfo).c_str());


          std::printf("running simulation...\n");
          std::printf("timestep: %.4f / 1/%d\n", helper.system->sceneConfig.timeStep,  (int) (1.0 / helper.system->sceneConfig.timeStep) );

          double loss = helper.runSimulationAndGetLoss(randomInitPrams);
          std::printf("forward record 5 timestep: %.5f\n", helper.system->forwardRecords[1].t);

          std::printf("Loss is %.5f\n", loss);
          std::printf("running rendering...\n");
          RenderLoop::renderRecordsForSystem(clothSystem, clothSystem->forwardRecords, false, false,
                                             "Set text here for whatever you need (only single line is supported): Visualization for pySimulations::runExample");
        }

    }

};


#endif //OMEGAENGINE_PYTHONSIMULATIONS_H
