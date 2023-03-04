//
// Created by Yifei Li (liyifei@mit.edu) on 10/7/22.
//

#include "OptimizationTaskSetup.h"
#include "BackwardTaskSolver.h"
#include <LBFGSB.h>
#include <LBFGSpp/Param.h>

void BackwardTaskSolver::setDemoSceneConfigAndConvergence(
		Simulation *system, int demoNum,
		Simulation::BackwardTaskInformation &taskInfo) {
	system->sceneConfig =
			OptimizationTaskConfigurations::demoNumToConfigMap[demoNum].scene;
	taskInfo.forwardAccuracyLevel = system->sceneConfig.forwardConvergenceThresh;
	taskInfo.backwardAccuracyLevel =
			system->sceneConfig.backwardConvergenceThresh;
}

void BackwardTaskSolver::setWindSim2realInitialParams(
		Simulation::ParamInfo &paramActual,
		Simulation::BackwardTaskInformation &taskInfo, Simulation *system) {
	system->sceneConfig.windConfig = WIND_SIN_AND_FALLOFF;
	taskInfo.dL_dfwind = true;
	paramActual.f_extwind.segment(0, 3) = Vec3d(1, 0.1, 1.0).normalized() * 1;
	paramActual.f_extwind[3] = 14.0;
	paramActual.f_extwind[4] = 0;
	taskInfo.dL_dk_pertype[2] = true;
	paramActual.k_pertype[2] = 3000.0;
	taskInfo.dL_dk_pertype[3] = true;
	paramActual.k_pertype[3] = 0.01;
	taskInfo.dL_density = true;
	paramActual.density = 0.15;
}

void BackwardTaskSolver::resetSplineConfigsForControlTasks(
		int demoNum, Simulation *system, Simulation::ParamInfo &paramActual) {
	for (Spline &s : system->sysMat[0].controlPointSplines) {
		s.type = Spline::ENDPOINT_AND_TANGENTS;
	}
	system->updateBackwardDefaultInfo();
	paramActual.controlPointSplines = {};
}

void BackwardTaskSolver::setLossFunctionInformationAndType(
		LossType &lossType, Simulation::LossInfo &lossInfo, Simulation *system,
		int demoNum) {
	if (OptimizationTaskConfigurations::demoNumToConfigMap[demoNum]
					.generateGroundtruthSimulation)
		lossInfo.targetSimulation = system->groundTruthForwardRecords;
	lossType =
			OptimizationTaskConfigurations::demoNumToConfigMap[demoNum].lossType;
	switch (demoNum) {
		case DEMO_WEAR_HAT: {
			Vec3d bustCenter = system->sphere_head.center +
					Vec3d(0, system->sphere_head.radius * 0.6, 0);
			Vec3d hatCenter = (system->restShapeMinDim + system->restShapeMaxDim) * 0.5;
			Vec3d translation = bustCenter - hatCenter;
			lossInfo.targetFrameShape.emplace_back(system->sceneConfig.stepNum,
					VecXd(system->particles.size() * 3));
			VecXd hatTargetPos(system->particles.size() * 3);
			std::vector<Vec3d> hatOnBust =
					MeshFileHandler::loadPosFile_txt("remeshed/Hat/hat_target.txt");
			for (int i = 0; i < system->particles.size(); i++) {
				hatTargetPos.segment(i * 3, 3) = hatOnBust[i];
			}
			lossInfo.targetFrameShape[0].second = hatTargetPos;
			system->debugShapeTargetPos = lossInfo.targetFrameShape;
			lossInfo.targetTranslation = translation;
			break;
		}

		case DEMO_WEAR_SOCK: {
			Capsule &foot = *(system->sockLeg.foot);
			Capsule &leg = *(system->sockLeg.leg);
			Rotation &legRotation = leg.rotationFromParent;
			Rotation &footRotation = foot.rotationFromParent;
			Vec3d legBaseCenter = system->sockLeg.center + leg.center;
			Vec3d footBaseCenter = system->sockLeg.center + foot.center;

			Vec3d centerTop =
					leg.getTransformedPosFromJointBindPos(Vec3d(0, leg.length, 0));
			Vec3d centerTopLeft = leg.getTransformedPosFromJointBindPos(
					Vec3d(-leg.radius, leg.length, 0));
			Vec3d centerTopRight =
					leg.getTransformedPosFromJointBindPos(Vec3d(leg.radius, leg.length, 0));
			Vec3d centerTopFront =
					leg.getTransformedPosFromJointBindPos(Vec3d(0, leg.length, leg.radius));
			Vec3d centerTopBack = leg.getTransformedPosFromJointBindPos(
					Vec3d(0, leg.length, -leg.radius));

			Vec3d calfPoint = leg.getTransformedPosFromJointBindPos(
					Vec3d(0, system->sockLeg.leg->length * 0.4, -leg.radius));

			Vec3d heelPoint = foot.getTransformedPosFromJointBindPos(
					Vec3d(0.0, foot.length, -foot.radius));
			Vec3d archPoint = foot.getTransformedPosFromJointBindPos(
					Vec3d(0.0, foot.length * 0.5, foot.radius));
			Vec3d toePoint =
					foot.getTransformedPosFromJointBindPos(Vec3d(0, -foot.radius, 0));
			Vec3d footTipBackPoint =
					foot.getTransformedPosFromJointBindPos(Vec3d(0.0, 0, -foot.radius));
			Vec3d footTipLeftPoint =
					foot.getTransformedPosFromJointBindPos(Vec3d(-foot.radius, 0, 0));
			Vec3d footTipRightPoint =
					foot.getTransformedPosFromJointBindPos(Vec3d(foot.radius, 0, 0));
			std::vector<int> topFrontPoints = { 104, 27, 43, 475, 392,
				903, 416, 413, 895 },
							 topLeftPoints = { 11, 30, 164, 755, 30 },
							 topRightPoints = { 563, 43, 474, 14 },
							 toePoints = { 865, 420, 946, 250, 80 },
							 openingBackPoints = { 102, 81, 842, 318, 12 };

			std::vector<Simulation::CorresPondenceTargetInfo> mappingPairs;
			int lastFrameIdx = OptimizationTaskConfigurations::sockScene.stepNum;
			mappingPairs.emplace_back(lastFrameIdx, heelPoint,
					std::vector<int>{ 2, 20, 336, 792, 995 });
			mappingPairs.emplace_back(lastFrameIdx, toePoint, toePoints);
			mappingPairs.emplace_back(lastFrameIdx, archPoint,
					std::vector<int>{ 282, 343, 249 });
			mappingPairs.emplace_back(lastFrameIdx, centerTopFront, topFrontPoints);
			mappingPairs.emplace_back(lastFrameIdx, centerTopLeft,
					topLeftPoints); // sock right
			mappingPairs.emplace_back(lastFrameIdx, centerTopRight,
					topRightPoints); // sock left
			mappingPairs.emplace_back(lastFrameIdx, centerTopBack, openingBackPoints);

			mappingPairs.emplace_back(lastFrameIdx, calfPoint,
					std::vector<int>{ 37, 241, 349 }); // sock left

			for (int i = 0; i < 3; i++) { // increase weight for opening
				mappingPairs.emplace_back(lastFrameIdx * 0.62 + i, toePoint,
						topFrontPoints); // sock left
				mappingPairs.emplace_back(lastFrameIdx * 0.62 + i, footTipBackPoint,
						openingBackPoints);
				mappingPairs.emplace_back(lastFrameIdx * 0.62 + i, footTipLeftPoint,
						topLeftPoints);
				mappingPairs.emplace_back(lastFrameIdx * 0.62 + i, footTipRightPoint,
						topRightPoints);
			}

			lossInfo.targetPosPairs = mappingPairs;
			system->debugPointTargetPos = mappingPairs;
			break;
		}

		case DEMO_SLOPE_PERF: {
			Vec3d slopeEnd = system->slope.lowerLeft + system->slope.lowerRight;
			Vec3d hatCenter = (system->restShapeMinDim + system->restShapeMaxDim) * 0.5;
			Vec3d translation = slopeEnd - hatCenter;
			lossInfo.targetTranslation = translation;
			break;
		}

		case DEMO_DRESS_TWIRL: {
			lossInfo.targetTwirlHeight = 0.3;
			system->sceneConfig.name =
					"dressTwirl-" + d2str(lossInfo.targetTwirlHeight, 5);
			lossInfo.loopPoints = {};
			printf("Dress loop points:\n");
			for (Particle &p : system->particles) {
				if (std::abs(p.pos_rest[1] - system->restShapeMinDim[1]) < 1.2) {
					lossInfo.loopPoints.insert(p.idx);
					printf("%d,", p.idx);
				}
			}
			system->debugLoopPoints = lossInfo.loopPoints;
			printf("\n");
			break;
		}
	}
}

void BackwardTaskSolver::setInitialConditions(
		int demoNum, Simulation *system, Simulation::ParamInfo &paramGroundtruth,
		Simulation::BackwardTaskInformation &taskInfo) {
	switch (demoNum) {
		case DEMO_WIND: {
			system->setWindAncCollision(true, true, true);
			system->wind = Vec3d(0, 1, 1) * 0.02;
			taskInfo.dL_dfext = true;
			paramGroundtruth.f_ext = system->wind * 0.2;
			break;
		}
		case DEMO_WIND_TSHIRT: {
			system->setWindAncCollision(true, true, true);
			taskInfo.dL_dk_pertype[2] = true;
			paramGroundtruth.k_pertype[2] =
					OptimizationTaskConfigurations::tshirtScene.fabric.k_stiff_stretching;
			taskInfo.dL_dfwind = true;
			paramGroundtruth.f_extwind.segment(0, 3) =
					Vec3d(1, 0.1, 1.0).normalized() * 0.1 * 0.15;
			paramGroundtruth.f_extwind[3] = 10;
			paramGroundtruth.f_extwind[4] = 0.5;

			break;
		}

		case DEMO_SPHERE_ROTATE: {
			system->setWindAncCollision(false, true, true);
			taskInfo.dL_dmu = true;
			taskInfo.mu_primitives.emplace_back(0);
			paramGroundtruth.mu.emplace_back(0, 0.3);
			break;
		}

		case DEMO_WIND_SIM2REAL: {
			system->setWindAncCollision(true, true, true);
			system->sceneConfig.stepNum = 100;
			system->groundTruthForwardRecords.resize(system->sceneConfig.stepNum + 1);
			setWindSim2realInitialParams(paramGroundtruth, taskInfo, system);

			break;
		}

		case DEMO_WEAR_HAT:
		case DEMO_WEAR_SOCK: {
			system->setWindAncCollision(false, true, true);
			taskInfo.dL_dcontrolPoints = true;
			resetSplineConfigsForControlTasks(demoNum, system, paramGroundtruth);
			break;
		}

		case DEMO_SLOPE_PERF: {
			system->setWindAncCollision(false, true, true);
			taskInfo.dL_dmu = true;
			taskInfo.mu_primitives.emplace_back(0);
			paramGroundtruth.mu.emplace_back(0, 0.5);
			break;
		}

		case DEMO_DRESS_TWIRL: {
			system->setWindAncCollision(false, true, true);
			taskInfo.dL_density = true;
			taskInfo.dL_dk_pertype[Constraint::CONSTRAINT_TRIANGLE_BENDING] = true;
			paramGroundtruth.density = 0.01;
			paramGroundtruth.k_pertype[Constraint::CONSTRAINT_TRIANGLE] = 2.0;
			break;
		}
	}
	taskInfo.paramActual = paramGroundtruth;
}