//
// Created by Yifei Li on 5/15/21.
// Email: liyifei@csail.mit.edu
//

#include "OptimizeHelper.h"

VecXd OptimizeHelper::getLowerBound() {
	VecXd lb = paramLowerBound;
	return lb;
}

OptimizeHelper::OptimizeHelper(Demos demoNum, Simulation *system,
		Simulation::LossInfo &lossInfo,
		Simulation::BackwardTaskInformation &taskInfo,
		LossType lossType, int FORWARD_STEPS,
		Simulation::ParamInfo paramActual) :
		demoNum(demoNum), system(system), lossInfo(lossInfo), taskInfo(taskInfo), lossType(lossType), FORWARD_STEPS(FORWARD_STEPS), param_actual(paramActual) {
	for (int sysMatId = 0; sysMatId < system->sysMat.size(); sysMatId++) {
		Simulation::SystemMatrix &sysMat = system->sysMat[sysMatId];
		param_guess.controlPointSplines.emplace_back(sysMat.controlPointSplines);
		for (int splineId = 0; splineId < sysMat.controlPointSplines.size();
				splineId++) {
			Spline &s = param_guess.controlPointSplines[sysMatId][splineId];
			for (int segId = 0; segId < s.segments.size(); segId++) {
				{
					if (splineId < 2)
						s.moveEndPoint(segId, s.segments[segId].p0 + Vec3d(-1, 1, 0));
					else
						s.moveEndPoint(segId, s.segments[segId].p0 + Vec3d(1, 1, 0));
				}
			}
		}
	}
	param_guess.mu.resize(taskInfo.mu_primitives.size());
	hasGroundtruthParam = false;
	totalParamNumber = 0;
	totalSplineParamNumber = 0;
	setParameterBounds(taskInfo);
}

void OptimizeHelper::setParameterBounds(
		Simulation::BackwardTaskInformation &taskInfo) {
	std::vector<std::pair<double, double>> bounds;
	if (taskInfo.dL_dfwind) {
		offset.offset_dL_dfwind = totalParamNumber;
		totalParamNumber += 5;
		for (int i = 0; i < 3; i++) {
			bounds.emplace_back(-0.1, 0.1); // windForce
			paramLogScaleTransformOn.emplace_back(false);
			paramName.emplace_back("windForce");
		}
		bounds.emplace_back(0.01, 15); // windFrequency
		paramLogScaleTransformOn.emplace_back(windScale);
		paramName.emplace_back("windFreq");

		bounds.emplace_back(-5, 5); // windPhase
		paramLogScaleTransformOn.emplace_back(false);
		paramName.emplace_back("windPhase");
	}

	if (taskInfo.dL_dfext) {
		offset.offset_dL_dfext = totalParamNumber;
		totalParamNumber += 3;
		for (int i = 0; i < 3; i++) {
			bounds.emplace_back(-3, 3); // windDir
			paramLogScaleTransformOn.emplace_back(false);
			paramName.emplace_back("windDir");
		}
	}

	std::vector<std::pair<double, double>> stiffnessBounds = {
		{ 0, 200 }, { 63, 10000 }, { 80, 1500 }, { 1e-7, 5 }
	};

	if (taskInfo.dL_dx0) {
		offset.offset_dL_dx0 = totalParamNumber;
		totalParamNumber += system->particles.size() * 3;
		for (int i = 0; i < system->particles.size(); i++) {
			for (int dim = 0; dim < 3; dim++) {
				bounds.emplace_back(
						system->sceneConfig.sceneBbox.min[dim],
						system->sceneConfig.sceneBbox.max[dim]); // windForce
			}
		}
	}

	if (taskInfo.dL_dconstantForceField) {
		offset.offset_dL_dconstantForceField = totalParamNumber;
		totalParamNumber += system->particles.size() * 3;
		for (int i = 0; i < system->particles.size(); i++) {
			for (int dim = 0; dim < 3; dim++) {
				bounds.emplace_back(-10, 10); // constantForceField
			}
		}
	}

	for (int i = 0; i < Constraint::CONSTRAINT_NUM; i++) {
		if (taskInfo.dL_dk_pertype[i]) {
			offset.offset_dL_k[i] = totalParamNumber;
			totalParamNumber += 1;
			bounds.emplace_back(stiffnessBounds[i]);
			paramLogScaleTransformOn.emplace_back(stiffnessLogScale);
			paramName.emplace_back(CONSTRAINT_TYPE_STRINGS[i]);
		}
	}

	if (taskInfo.dL_density) {
		offset.offset_dL_density = totalParamNumber;
		totalParamNumber += 1;
		bounds.emplace_back(0.01, 1.0);
		paramLogScaleTransformOn.emplace_back(densityLogScale);
		paramName.emplace_back("density");
	}

	if (taskInfo.dL_dcontrolPoints) {
		offset.offset_dL_dspline = totalParamNumber;
		for (Simulation::SystemMatrix &sysmat : system->sysMat) {
			int paramNum = Spline::getParameterNumber(sysmat.controlPointSplines);
			totalParamNumber += paramNum;
			totalSplineParamNumber += paramNum;
			for (Spline &s : sysmat.controlPointSplines) {
				std::pair<VecXd, VecXd> splineBounds =
						s.getParameterBounds(system->sceneConfig.sceneBbox);
				for (int i = 0; i < splineBounds.first.rows(); i++) {
					bounds.emplace_back(splineBounds.first[i], splineBounds.second[i]);
					paramLogScaleTransformOn.emplace_back(false);
					paramName.emplace_back("spline");
				}
			}
		}
	}

	if (taskInfo.dL_dmu) {
		for (int primId : taskInfo.mu_primitives) {
			offset.offset_dL_dmu.emplace_back(totalParamNumber);
			bounds.emplace_back(0.01, 0.95);
			paramLogScaleTransformOn.emplace_back(false);
			totalParamNumber++;
			paramName.emplace_back("mu");
		}
	}

	paramLowerBound = VecXd(totalParamNumber);
	paramLowerBound.setZero();
	paramUpperBound = VecXd(totalParamNumber);
	paramUpperBound.setZero();
	for (int i = 0; i < totalParamNumber; i++) {
		paramLowerBound[i] = bounds[i].first;
		paramUpperBound[i] = bounds[i].second;
	}

	param_mean = (paramLowerBound + paramUpperBound) * 0.5;
	param_scale = paramUpperBound - paramLowerBound;
	experimentName = currentTimestampToStr() + "-" + system->sceneConfig.name;
}

VecXd OptimizeHelper::getUpperBound() {
	VecXd ub = paramUpperBound;
	return ub;
}

VecXd OptimizeHelper::paramInfoToVecXd(Simulation::ParamInfo &param) {
	VecXd x(totalParamNumber);
	x.setZero();
	if (taskInfo.dL_dfwind) {
		x.segment(offset.offset_dL_dfwind, 5) = param.f_extwind;
	}

	if (taskInfo.dL_dfext)
		x.segment(offset.offset_dL_dfext, 3) = param.f_ext;

	if (taskInfo.dL_dwindFactor) {
		x.segment(offset.offset_dL_dfwind_timestep,
				param_guess.f_ext_timestep.rows()) = param.f_ext_timestep;
	}

	if (taskInfo.dL_dx0) {
		x.segment(offset.offset_dL_dx0, 3 * system->particles.size()) = param.x0;
	}

	if (taskInfo.dL_dconstantForceField) {
		x.segment(offset.offset_dL_dconstantForceField,
				3 * system->particles.size()) = param.f_constantForceField;
	}

	for (int i = 0; i < 4; i++) {
		if (taskInfo.dL_dk_pertype[i]) {
			x[offset.offset_dL_k[i]] = param.k_pertype[i];
		}
	}

	if (taskInfo.dL_density) {
		x[offset.offset_dL_density] = param.density;
	}

	if (taskInfo.dL_dmu) {
		for (int i = 0; i < taskInfo.mu_primitives.size(); i++) {
			x[offset.offset_dL_dmu[i]] = param.mu[i].second;
		}
	}

	if (taskInfo.dL_dcontrolPoints) {
		VecXd splineParams(totalSplineParamNumber);
		splineParams.setZero();
		int curSplineoffset = 0;
		for (int sysMatId = 0; sysMatId < system->sysMat.size(); sysMatId++) {
			for (int splineId = 0;
					splineId < system->sysMat[sysMatId].controlPointSplines.size();
					splineId++) {
				int splineDOF = param_guess.controlPointSplines[sysMatId][splineId]
										.getParameterNumber();
				int splineDOF2 =
						param.controlPointSplines[sysMatId][splineId].getParameterNumber();
				VecXd splineParam = splineParams.segment(curSplineoffset, splineDOF);
				if (splineDOF != splineDOF2)
					std::printf("WARNING: spline param number mismatch %d %d\n",
							splineDOF, splineDOF2);
				splineParams.segment(curSplineoffset, splineDOF) =
						param.controlPointSplines[sysMatId][splineId].paramToVector();
				curSplineoffset += splineDOF;
			}
		}

		x.segment(offset.offset_dL_dspline, totalSplineParamNumber) = splineParams;
	}
	// x = 2 * (x_raw - mean) / scale
	// x_raw = 0.5 * x * scale + mean

	return x;
}

VecXd OptimizeHelper::gradientInfoToVecXd(
		Simulation::BackwardInformation &backwardInfo) {
	VecXd grad(totalParamNumber);
	grad.setZero();
	if (taskInfo.dL_dfwind) {
		grad.segment(offset.offset_dL_dfwind, 5) = backwardInfo.dL_dwind;
		if (windScale) {
			// TODO: removed scaling
		}
	}

	if (taskInfo.dL_dwindFactor) {
		grad.segment(offset.offset_dL_dfwind_timestep,
				backwardInfo.dL_dwindtimestep.rows()) =
				backwardInfo.dL_dwindtimestep;
	}

	if (taskInfo.dL_dx0) {
		grad.segment(offset.offset_dL_dx0, 3 * system->particles.size()) =
				backwardInfo.dL_dx;
	}

	if (taskInfo.dL_dconstantForceField) {
		grad.segment(offset.offset_dL_dconstantForceField,
				3 * system->particles.size()) =
				backwardInfo.dL_dconstantForceField;
	}

	if (taskInfo.dL_dfext)
		grad.segment(offset.offset_dL_dfext, 3) = backwardInfo.dL_dfext;

	for (int i = 0; i < 4; i++) {
		if (taskInfo.dL_dk_pertype[i]) {
			grad[offset.offset_dL_k[i]] =
					backwardInfo.dL_dk_pertype[i]; // Note: Removed scaling
		}
	}

	if (taskInfo.dL_density) {
		grad[offset.offset_dL_density] =
				backwardInfo.dL_ddensity; // TODO: removed scaling
	}

	if (taskInfo.dL_dmu) {
		for (int i = 0; i < taskInfo.mu_primitives.size(); i++) {
			grad[offset.offset_dL_dmu[i]] = backwardInfo.dL_dmu[i].second;
		}
	}

	if (taskInfo.dL_dxfixed) {
		// grad.segment(offset.offset_dL_dxfixed, backwardInfo.dL_dxfixed.rows()) =
		// backwardInfo.dL_dxfixed;
	}
	if (taskInfo.dL_dcontrolPoints) {
		std::printf("totalSplineParams: %d\n", totalSplineParamNumber);
		VecXd splineParams(totalSplineParamNumber);
		splineParams.setZero();
		int curSplineOffset = 0;
		for (int sysMatId = 0; sysMatId < system->sysMat.size(); sysMatId++) {
			for (int splineId = 0;
					splineId < system->sysMat[sysMatId].controlPointSplines.size();
					splineId++) {
				int splineDOF = param_guess.controlPointSplines[sysMatId][splineId]
										.getParameterNumber();
				if (splineDOF != backwardInfo.dL_dsplines[sysMatId][splineId].rows())
					std::printf("WARNING: spline param number mismatch %d %zu\n",
							splineDOF,
							backwardInfo.dL_dsplines[sysMatId][splineId].rows());
				splineParams.segment(curSplineOffset, splineDOF) =
						backwardInfo.dL_dsplines[sysMatId][splineId];
				curSplineOffset += splineDOF;
			}
		}
		grad.segment(offset.offset_dL_dspline, totalSplineParamNumber) =
				splineParams;
	}

	return grad;
}

std::pair<bool, VecXd> OptimizeHelper::parameterFromRandSeed(int randSeed) {
	VecXd x(totalParamNumber);
	VecXd ones(totalParamNumber);
	ones.setOnes();
	x.setRandom();
	x = x * 0.5 + ones * 0.5;
	x = paramLowerBound + x.cwiseProduct(paramUpperBound - paramLowerBound);
	Simulation::ParamInfo param = vecXdToParamInfo(x);
	if (taskInfo.dL_dcontrolPoints) {
		std::vector<std::vector<Spline>> initControlSplines =
				param_guess.controlPointSplines;

		// Force spline shapes to reduce init param space
		VecXd splineEndPointTranslation =
				param.controlPointSplines[0][0].segments[0].p1 -
				param.controlPointSplines[0][0].segments[0].p0;

		param.controlPointSplines = initControlSplines;
		for (std::vector<Spline> &splines : param.controlPointSplines) {
			for (int i = 0; i < splines.size(); i++) {
				Spline &s = splines[i];
				s.moveEndPoint(0, s.segments[0].p0 + splineEndPointTranslation);
			}
		}
	}

	if (taskInfo.dL_dfext) {
		param.f_ext = std::min(1.0, param.f_ext.norm()) * param.f_ext.normalized();
	}

	if (taskInfo.dL_dfwind) {
		Vec3d windDir = param.f_extwind.segment(0, 3);
		param.f_extwind.segment(0, 3) =
				std::min(2.0, windDir.norm()) * windDir.normalized();
		std::printf("f_wind is normalized to norm %.3f\n",
				param.f_extwind.segment(0, 3).norm());
	}

	if (taskInfo.dL_dwindFactor) {
		// TODO
		Logging::logFatal("dL_dwindFactor not handled!!\n");
	}

	x = paramInfoToVecXd(param);
	bool isInBound = paramIsWithinBound(x);
	return std::make_pair(isInBound, x);
}

Simulation::ParamInfo OptimizeHelper::vecXdToParamInfo(const VecXd &x) {
	Simulation::ParamInfo param = {};
	if (taskInfo.dL_dfwind) {
		param.f_extwind = x.segment(offset.offset_dL_dfwind, 5);
	}

	if (taskInfo.dL_dwindFactor) {
		param.f_ext_timestep = x.segment(offset.offset_dL_dfwind_timestep,
				param_guess.f_ext_timestep.rows());
	}

	if (taskInfo.dL_dx0) {
		param.x0 = x.segment(offset.offset_dL_dx0, 3 * system->particles.size());
	}

	if (taskInfo.dL_dconstantForceField) {
		param.f_constantForceField = x.segment(offset.offset_dL_dconstantForceField,
				3 * system->particles.size());
	}

	if (taskInfo.dL_dfext)
		param.f_ext = x.segment(offset.offset_dL_dfext, 3);

	for (int i = 0; i < 4; i++) {
		if (taskInfo.dL_dk_pertype[i]) {
			param.k_pertype[i] = x[offset.offset_dL_k[i]];
		}
	}

	if (taskInfo.dL_density) {
		param.density = x[offset.offset_dL_density];
	}

	if (taskInfo.dL_dmu) {
		for (int i = 0; i < taskInfo.mu_primitives.size(); i++) {
			param.mu.emplace_back(taskInfo.mu_primitives[i],
					x[offset.offset_dL_dmu[i]]);
		}
	}

	if (taskInfo.dL_dcontrolPoints) {
		VecXd splineParams =
				x.segment(offset.offset_dL_dspline, totalSplineParamNumber);
		int curSplinePos = 0;
		for (int sysMatId = 0; sysMatId < system->sysMat.size(); sysMatId++) {
			param.controlPointSplines.emplace_back();
			for (int splineId = 0;
					splineId < system->sysMat[sysMatId].controlPointSplines.size();
					splineId++) {
				int splineDOF = param_guess.controlPointSplines[sysMatId][splineId]
										.getParameterNumber();
				VecXd splineParam = splineParams.segment(curSplinePos, splineDOF);
				Spline splineFromParam = Spline::splineFromParam(
						param_guess.controlPointSplines[sysMatId][splineId], splineParam);
				param.controlPointSplines[sysMatId].emplace_back(splineFromParam);
				curSplinePos += splineDOF;
			}
		}
	}

	return param;
}

std::pair<VecXd, int> OptimizeHelper::clampParameter(VecXd &in) {
	VecXd out = in;
	int activeParam = 0;
	if (in.rows() != paramLowerBound.rows()) {
		Logging::logWarning(
				"OptimizeHelper::clampParameter:  parameter dimension invalid");
		return std::make_pair(in, 0);
	}

	for (int i = 0; i < in.rows(); i++) {
		if (out[i] < paramLowerBound[i]) {
			out[i] = paramLowerBound[i];
		} else if (out[i] > paramUpperBound[i]) {
			out[i] = paramUpperBound[i];
		} else {
			activeParam++;
		}
	}

	return std::make_pair(out, activeParam);
}

VecXd OptimizeHelper::getRandomParam(int randSeed) {
	std::srand(randSeed);
	VecXd x;
	while (true) {
		int seed = std::rand();
		taskInfo.randSeed = seed;
		taskInfo.srandSeed = randSeed;
		std::cerr << "seed = " << seed << std::endl;
		std::pair<bool, VecXd> paramCandidate = parameterFromRandSeed(seed);
		x = paramCandidate.second;
		if (!paramCandidate.first)
			continue;
		Simulation::ParamInfo param = vecXdToParamInfo(x);

		bool isValid = true;
		if (taskInfo.dL_dcontrolPoints) {
			int splineNum = param.controlPointSplines[0].size();
			// make sure splines are close, trajectory points are all in bound
			for (int s1 = 0; s1 < splineNum; s1++) {
				isValid = isValid && param.controlPointSplines[0][s1].isWithinBound(system->sceneConfig.sceneBbox);
				if (!isValid)
					break;
				for (int s2 = s1 + 1; s2 < splineNum; s2++) {
					double dist = (param.controlPointSplines[0][s1].segments[0].p0 -
							param.controlPointSplines[0][s2].segments[0].p0)
										  .norm();
					if (!Spline::isClose(param.controlPointSplines[0][s1],
								param.controlPointSplines[0][s2], dist + 0.5l)) {
						isValid = false;
						break;
					}
				}
			}
		}

		if (isValid) {
			std::printf("Parameter from randseed %d is valid, proceed...\n", seed);
			std::cout << "  param lowerBound:" << getLowerBound().transpose()
					  << std::endl;
			std::cout << "  param upperBound:" << getUpperBound().transpose()
					  << std::endl;
			std::cout << " starts with param:" << x.transpose() << std::endl;
			break;
		}
	}
	return x;
}

bool OptimizeHelper::paramIsWithinBound(VecXd x) {
	for (int i = 0; i < totalParamNumber; i++) {
		if (x[i] > paramUpperBound[i])
			return false;
		if (x[i] < paramLowerBound[i])
			return false;
	}
	return true;
}

double OptimizeHelper::runSimulationAndGetLoss(const VecXd &x) {
	Simulation::ParamInfo param = vecXdToParamInfo(x);

	double loss = system
						  ->runBackwardTask(
								  taskInfo, lossType, lossInfo, statistics, FORWARD_STEPS,
								  param, true, [&](const std::string &v) {}, false)[0]
						  .loss;
	return loss;
}

std::vector<Simulation::BackwardInformation>
OptimizeHelper::runSimulationAndGetLossAndGradients(const VecXd &x) {
	Simulation::ParamInfo param = vecXdToParamInfo(x);

	return system->runBackwardTask(
			taskInfo, lossType, lossInfo, statistics, FORWARD_STEPS, param, false,
			[&](const std::string &v) {}, false);
}

Simulation::BackwardInformation
OptimizeHelper::runSimulationAndGetLossAndGradient(const VecXd &x) {
	return runSimulationAndGetLossAndGradients(x)[0];
}

VecXd OptimizeHelper::getActualParam() {
	Logging::logColor("******************Actual Wind\n", Logging::GREEN);
	std::cout << param_actual.f_extwind << "\n";
	return paramInfoToVecXd(param_actual);
}

void OptimizeHelper::saveLastIter() {
	system->backwardOptimizationRecords.emplace_back(lastBacakwardOptRecord);
	system->backwardOptimizationGuesses.push_back(lastGuess);
	system->exportStatistics((Demos)demoNum, statistics, taskInfo);
}

double OptimizeHelper::operator()(const VecXd &x, VecXd &grad) {
	iter++;
	Logging::logOk("=========Evaluation Iter " + int2str(iter) + " ========\n");
	Logging::logOk("Total Optimization Iter:" +
			int2str(system->backwardOptimizationRecords.size()) + "\n");

	std::printf("Total parameters: %d\n", totalParamNumber);
	std::printf("Total frames: %d\n", system->sceneConfig.stepNum);
	std::printf("timestep: %.4f / 1/%d\n", system->sceneConfig.timeStep,
			(int)(1.0 / system->sceneConfig.timeStep));

	Simulation::ParamInfo param = vecXdToParamInfo(x);
	Logging::logColor("LBFGS param:\n" +
					Simulation::parameterToString(taskInfo, param) + "\n",
			Logging::LogColor::MAGENTA);

	std::vector<Simulation::BackwardInformation> backwardRecords =
			system->runBackwardTask(taskInfo, lossType, lossInfo, statistics,
					FORWARD_STEPS, param, false);

	lastBacakwardOptRecord =
			std::make_pair(system->forwardRecords, backwardRecords);
	lastGuess = std::make_pair(param, backwardRecords[0].loss);
	double loss = backwardRecords[0].loss;
	saveLastIter();
	system->exportStatistics((Demos)demoNum, statistics, taskInfo);

	Simulation::BackwardInformation gradient = backwardRecords[0];
	grad = gradientInfoToVecXd(gradient);
	Simulation::backwrdInfoAndGradToString(taskInfo, backwardRecords[0]);
	system->linesearchRecords = system->forwardRecords;
	Logging::logOk("Loss is: " + d2str(loss, 8) + "\n");
	return loss;
}