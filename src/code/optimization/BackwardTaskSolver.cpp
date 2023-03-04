//
// Created by Yifei Li on 3/15/21.
// Email: liyifei@csail.mit.edu
//
#include "BackwardTaskSolver.h"
#include "OptimizationTaskSetup.h"
#include <LBFGSB.h>
#include <LBFGSpp/Param.h>

void BackwardTaskSolver::solveDemo(
		Simulation *system,
		const std::function<void(const std::string &)> &setTextBoxCB, int demoNum,
		bool isRandom, int srandSeed) {
	OptimizeHelper helper = getOptimizeHelper(system, demoNum);

	helper.taskInfo.optimizer = Optimizer::LBFGS;
	optimizeLBFGS(system, helper, system->sceneConfig.stepNum, demoNum, isRandom,
			srandSeed, setTextBoxCB);
}

void BackwardTaskSolver::optimizeLBFGS(
		Simulation *system, OptimizeHelper &helper, int FORWARD_STEPS, int demoNum,
		bool isRandom, int srandSeed,
		const std::function<void(const std::string &)> &setTextBoxCB) {
	LBFGSpp::LBFGSBParam<double> param;
	param.delta = 0.001; // objective function convergence param
	param.m = 10;
	param.max_linesearch = 20;

	LBFGSpp::LBFGSBSolver<double> solver(param);

	std::printf("Parameter bounds are:\n");
	std::cout << "Lower:" << helper.paramLowerBound.transpose() << "\n";
	std::cout << "Upper:" << helper.paramUpperBound.transpose() << "\n";
	VecXd bestParam = helper.paramInfoToVecXd(helper.param_guess);

	if (isRandom) {
		std::printf("Srand seed is : %d\n", srandSeed);
		bestParam = helper.getRandomParam(srandSeed);
	}

	std::cout << "lbfgs param lowerBound:" << helper.getLowerBound().transpose()
			  << std::endl;
	std::cout << "lbfgs param upperBound:" << helper.getUpperBound().transpose()
			  << std::endl;
	std::cout << "lbfgs starts with param:" << bestParam.transpose() << std::endl;
	double minLoss = 0;
	try {
		int niter = solver.minimize(helper, bestParam, minLoss,
				helper.getLowerBound(), helper.getUpperBound());
		system->exportStatistics((Demos)demoNum, helper.statistics, helper.taskInfo,
				true);
		if (niter == 1)
			helper.saveLastIter();
		std::printf("LBFGS terminated\n");
		std::cout << niter << " iterations" << std::endl;
		std::cout << "x = \n"
				  << bestParam.transpose() << std::endl;
		std::cout << "f(x) = " << minLoss << std::endl;
	} catch (const std::exception &error) {
		std::printf("LBFGS terminated early: %s\n", error.what());
		system->exportStatistics((Demos)demoNum, helper.statistics, helper.taskInfo,
				true);
	}
}

OptimizeHelper *BackwardTaskSolver::getOptimizeHelperPointer(Simulation *system,
		int demoNum) {
	Logging::logOk("getOptimizeHelperPointer for task" + DEMOS_STRINGS[demoNum] +
			"\n");
	OptimizeHelper helper = getOptimizeHelper(system, demoNum);

	return new OptimizeHelper(static_cast<Demos>(demoNum), helper.system,
			helper.lossInfo, helper.taskInfo, helper.lossType,
			helper.system->sceneConfig.stepNum,
			helper.param_actual);
}

OptimizeHelper BackwardTaskSolver::getOptimizeHelper(Simulation *system,
		int demoNum) {
	if (demoNum == Demos::DEMO_WIND_SIM2REAL) {
		std::string folder =
				std::string(SOURCE_PATH) + "/src/assets/animation/flag-ryanwhite";
		std::vector<std::string> frameFileNames = listFiles(folder, ".obj");

		std::sort(frameFileNames.begin(), frameFileNames.end());
		system->loadWindSim2RealAnimationSequence("flag-ryanwhite", frameFileNames,
				true);
	}

	system->runBackward = true;
	system->debugPointTargetPos.clear();
	system->debugShapeTargetPos.clear();
	system->backwardRecordsFiniteDiff.clear();
	system->backwardOptimizationRecords.clear();
	system->backwardOptimizationGuesses.clear();

	Simulation::BackwardTaskInformation taskInfo = Simulation::taskConfigDefault;
	Simulation::ParamInfo paramGroundtruth;
	Simulation::LossInfo lossInfo;
	LossType lossType;

	setDemoSceneConfigAndConvergence(system, demoNum, taskInfo);
	if (demoNum != Demos::DEMO_WIND_SIM2REAL) {
		system->createClothMesh();
		system->initScene();
	}

	setInitialConditions(demoNum, system, paramGroundtruth, taskInfo);
	if (OptimizationTaskConfigurations::demoNumToConfigMap[demoNum]
					.hasGroundtruth) {
		Logging::logColor(
				"Groundtruth param:\n" +
						Simulation::parameterToString(taskInfo, paramGroundtruth) + "\n",
				Logging::LogColor::MAGENTA);
	}

	setLossFunctionInformationAndType(lossType, lossInfo, system, demoNum);

	if (OptimizationTaskConfigurations::demoNumToConfigMap[demoNum]
					.generateGroundtruthSimulation) {
		system->resetSystemWithParams(taskInfo, paramGroundtruth);
		for (int i = 0; i < system->sceneConfig.stepNum; i++)
			system->step();
		system->groundTruthForwardRecords = system->forwardRecords;
	}
	setLossFunctionInformationAndType(lossType, lossInfo, system, demoNum);
	for (int i = 0; i < system->sysMat.size(); i++) {
		taskInfo.splineTypes.emplace_back();
		for (Spline &s : system->sysMat[i].controlPointSplines) {
			taskInfo.splineTypes[i].emplace_back(s.type);
		}
	}

	system->groundtruthParam = paramGroundtruth;
	system->taskInfo = taskInfo;

	int srandSeed = time(NULL);
	std::srand(srandSeed);

	return OptimizeHelper(static_cast<Demos>(demoNum), system, lossInfo, taskInfo,
			lossType, system->sceneConfig.stepNum,
			paramGroundtruth);
}
