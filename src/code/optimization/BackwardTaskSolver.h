//
// Created by Yifei Li on 3/15/21.
// Email: liyifei@csail.mit.edu
//

#ifndef OMEGAENGINE_BACKWARDTASKSOLVER_H
#define OMEGAENGINE_BACKWARDTASKSOLVER_H

#include "../engine/Macros.h"
#include "../engine/UtilityFunctions.h"
#include "../simulation/Simulation.h"
#include "OptimizationTaskConfigurations.h"
#include "OptimizeHelper.h"
#include <time.h>

class BackwardTaskSolver {
public:
	static void
	optimizeLBFGS(Simulation *system, OptimizeHelper &helper, int FORWARD_STEPS,
			int demoNum, bool isRandom, int srandSeed,
			const std::function<void(const std::string &)> &setTextBoxCB);

	static void
	setWindSim2realInitialParams(Simulation::ParamInfo &paramGroundtruth,
			Simulation::BackwardTaskInformation &taskInfo,
			Simulation *system);

	static void setDemoSceneConfigAndConvergence(
			Simulation *system, int demoNum,
			Simulation::BackwardTaskInformation &taskInfo);

	static void
	resetSplineConfigsForControlTasks(int demoNum, Simulation *system,
			Simulation::ParamInfo &paramGroundtruth);

	static void setLossFunctionInformationAndType(LossType &lossType,
			Simulation::LossInfo &lossInfo,
			Simulation *system,
			int demoNum);

	static void
	setInitialConditions(int demoNum, Simulation *system,
			Simulation::ParamInfo &paramGroundtruth,
			Simulation::BackwardTaskInformation &taskInfo);

	static void
	solveDemo(Simulation *system,
			const std::function<void(const std::string &)> &setTextBoxCB,
			int demoNum, bool isRandom, int srandSeed);

	static OptimizeHelper getOptimizeHelper(Simulation *system, int demoNum);

	static OptimizeHelper *getOptimizeHelperPointer(Simulation *system,
			int demoNum);
};

#endif // OMEGAENGINE_BACKWARDTASKSOLVER_H
