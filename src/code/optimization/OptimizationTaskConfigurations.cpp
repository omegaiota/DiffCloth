//
// Created by Yifei Li on 3/3/21.
// Email: liyifei@csail.mit.edu
//

#include "OptimizationTaskConfigurations.h"

// For structs that contain other static initialized structs as members, make
// sure you initize the structs in order
Simulation::FabricConfiguration
		OptimizationTaskConfigurations::normalFabric6lowres = {
			.clothDimX = 6,
			.clothDimY = 6,
			.k_stiff_stretching = 250,
			.k_stiff_bending = 0.05,
			.gridNumX = 5,
			.gridNumY = 5,
			.density = 0.324,
			.keepOriginalScalePoint = false,
			.isModel = false,
			.custominitPos = false,
			.fabricIdx = FabricEnumArray::NORMAL_6_LOWRES,
			.color = COLOR_EGGPLANT,
			.name = "dim6x6-grid25x25-dens0.32-k50",
		};

Simulation::FabricConfiguration OptimizationTaskConfigurations::normalFabric6 = {
	.clothDimX = 6,
	.clothDimY = 6,
	.k_stiff_stretching = 100,
	.k_stiff_bending = 0.00,
	.gridNumX = 25,
	.gridNumY = 25,
	.density = 0.054,
	.keepOriginalScalePoint = false,
	.isModel = false,
	.custominitPos = false,
	.fabricIdx = FabricEnumArray::NORMAL,
	.color = COLOR_EGGPLANT,
	.name = "dim6x6-grid25x25-dens0.32-k50",
};

Simulation::FabricConfiguration
		OptimizationTaskConfigurations::conitnuousNormalTestFabric = {
			.clothDimX = 4,
			.clothDimY = 4,
			.k_stiff_stretching = 100,
			.k_stiff_bending = 0.0,
			.gridNumX = 5,
			.gridNumY = 5,
			.density = 0.524,
			.keepOriginalScalePoint = true,
			.isModel = false,
			.custominitPos = true,
			.initPosFile = "continuousNormal_354.txt",
			.fabricIdx = FabricEnumArray::CONTINUOUS_NORMAL_FABRIC,
			.color = COLOR_EGGPLANT,
			.name = "continuousNormal",
		};

Simulation::FabricConfiguration OptimizationTaskConfigurations::tshirt1000 = {
	.clothDimX = 6,
	.clothDimY = 6,
	.k_stiff_stretching = 550,
	.k_stiff_bending = 0.01, // TODO: change back
	.gridNumX = 40, // 25,
	.gridNumY = 80, // 50,
	.density = 0.124,
	.keepOriginalScalePoint = false,
	.isModel = true,
	.custominitPos = false,
	.fabricIdx = FabricEnumArray::TSHIRT1000,
	.color = COLOR_EGGPLANT,
	.name = "remeshed/T-shirt/tshirt1000-tri.obj",
};

Simulation::FabricConfiguration OptimizationTaskConfigurations::sphereFabric = {
	.clothDimX = 4.5,
	.clothDimY = 4.5,
	.k_stiff_stretching = 150,
	.k_stiff_bending = 0.00001, // 0.0005,
	.gridNumX = 25,
	.gridNumY = 25,
	.density = 0.3,
	.keepOriginalScalePoint = false,
	.isModel = false,
	.custominitPos = false,
	.initPosFile = "remeshed/Slope/rest_on_plane.txt",
	.fabricIdx = FabricEnumArray::SLOPE_FAB,
	.color = COLOR_IBM_ORANGE40,
	.name = "sphereFabric"
};

Simulation::FabricConfiguration
		OptimizationTaskConfigurations::slopeFabricRestOnPlane = {
			.clothDimX = 4.5,
			.clothDimY = 4.5,
			.k_stiff_stretching = 50,
			.k_stiff_bending = 0.0000, // 0.0005,
			.gridNumX = 25,
			.gridNumY = 25,
			.density = 0.2,
			.keepOriginalScalePoint = false,
			.isModel = false,
			.custominitPos = true,
			.initPosFile = "remeshed/Slope/slopeFabric_onPlane.txt",
			.fabricIdx = FabricEnumArray::SLOPE_FAB_ONPLINE,
			.color = COLOR_IBM_ORANGE40,
			.name = "dim3x3-grid5x5-dens0.2-k50"
		};

Simulation::FabricConfiguration
		OptimizationTaskConfigurations::dressv7khandsUpDrape = {
			.clothDimX = 13,
			.clothDimY = 13,
			.k_stiff_stretching = 3000, // 2000,
			.k_stiff_bending = 0.3, // 0.2,
			.gridNumX = 40, // 25,
			.gridNumY = 80, // 50,
			.density = 0.3,
			.keepOriginalScalePoint = false,
			.isModel = true,
			.custominitPos = false,
			.fabricIdx = FabricEnumArray::DRESS_v7k_DRAPE,
			.color = Vec3d(0.9, 0.9, 0.9),
			.name = "remeshed/dress-handsup-drape.obj",
		};

Simulation::FabricConfiguration OptimizationTaskConfigurations::agenthat579 = {
	.clothDimX = 6,
	.clothDimY = 6,
	.k_stiff_stretching =
			1200, // old param: 300 // TODO: WARNING: change back to 1200
	.k_stiff_bending =
			120, // old param: 50 // TODO: WARNING: change back to 120
	.gridNumX = 40, //
	.gridNumY = 80, //
	.density = 0.224, // old param: 0.324
	.keepOriginalScalePoint = false,
	.isModel = true,
	.custominitPos = false,
	.fabricIdx = FabricEnumArray::AGENT_HAT579,
	.color = Vec3d(1, 0, 0),
	.name = "remeshed/agenthat2-579-rotated.obj",
};

Simulation::FabricConfiguration OptimizationTaskConfigurations::sock482 = {
	.clothDimX = 5,
	.clothDimY = 5,
	.k_stiff_stretching = 600,
	.k_stiff_bending = 1,
	.gridNumX = 40, // 25,
	.gridNumY = 80, // 50,
	.density = 0.224,
	.keepOriginalScalePoint = false,
	.isModel = true,
	.custominitPos = false,
	.fabricIdx = FabricEnumArray::SOCK482,
	.color = COLOR_IBM_GOLD20,
	.name = "remeshed/sock1055-2081.obj",
	//        .name =  "remeshed/sock482.obj",
};

Simulation::SceneConfiguration OptimizationTaskConfigurations::hatScene = {
	.fabric = agenthat579,
	.orientation = Orientation::FRONT,
	.attachmentPoints = AttachmentConfigs::CUSTOM_ARRAY,
	.customAttachmentVertexIdx = { { 0.0, { 394, 32 } } }, //{{0.0, {501}}},
	.trajectory = TrajectoryConfigs::CORNERS_2_WEARHAT,
	.primitiveConfig = PrimitiveConfiguration::PLANE_BUST_WEARHAT,
	.windConfig = WindConfig::NO_WIND,
	.camPos = Vec3d(-22.14, 9.24, 7.59),
	.camFocusPointType = CameraFocusPointType::CLOTH_CENTER,
	.sceneBbox = AABB(Vec3d(-5, -1.5, -14), Vec3d(7, 10, 5)),
	.timeStep = 1.0 / 100.0,
	.stepNum = 400,
	.forwardConvergenceThresh = 1e-8,
	.backwardConvergenceThresh = 5e-4,
	.name = "demo_wearhat"

};

Simulation::SceneConfiguration
		OptimizationTaskConfigurations::continousNormalScene = {
			.fabric = conitnuousNormalTestFabric,
			.orientation = Orientation::FRONT,
			.upVector = Vec3d(1, 0, 1),
			.attachmentPoints = AttachmentConfigs::NO_ATTACHMENTS,
			.trajectory = TrajectoryConfigs::NO_TRAJECTORY,
			.primitiveConfig = PrimitiveConfiguration::BIG_SPHERE,
			.windConfig = WindConfig::NO_WIND,
			.camPos = Vec3d(-22.14, 9.24, 7.59),
			.camFocusPointType = CameraFocusPointType::CLOTH_CENTER,
			.sceneBbox = AABB(Vec3d(-5, -1.5, -14), Vec3d(7, 10, 5)),
			.timeStep = 1.0 / 100.0,
			.stepNum = 50,
			.forwardConvergenceThresh = 1e-9,
			.backwardConvergenceThresh = 5e-4,
			.name = "bigsphere_continuousNormal"

		};

Simulation::SceneConfiguration OptimizationTaskConfigurations::simpleScene = {
	.fabric = normalFabric6lowres, // normalFabric6,
	.orientation = Orientation::FRONT,
	.attachmentPoints = AttachmentConfigs::NO_ATTACHMENTS,
	.trajectory = TrajectoryConfigs::NO_TRAJECTORY,
	.primitiveConfig = PrimitiveConfiguration::NONE,
	.windConfig = WindConfig::NO_WIND,
	.camPos = Vec3d(0, 5, 20),
	.camFocusPointType = CameraFocusPointType::ORIGIN,
	.sceneBbox = AABB(Vec3d(-7, -7, -7), Vec3d(7, 7, 7)),
	.timeStep = 1.0 / 30.0,
	.stepNum = 100,
	.forwardConvergenceThresh = 1e-9,
	.backwardConvergenceThresh = 5e-4,
	.name = "none"

};

Simulation::SceneConfiguration
		OptimizationTaskConfigurations::rotatingSphereScene = {
			.fabric = sphereFabric,
			.orientation = Orientation::DOWN,
			.attachmentPoints = AttachmentConfigs::NO_ATTACHMENTS,
			.trajectory = TrajectoryConfigs::NO_TRAJECTORY,
			.primitiveConfig = PrimitiveConfiguration::PLANE_AND_SPHERE,
			.windConfig = WindConfig::NO_WIND,
			.camPos = Vec3d(-11.67, 20.40, -11.67),
			.camFocusPointType = CameraFocusPointType::PRIM0CENTER,
			.sceneBbox = AABB(Vec3d(-7, -7, -7), Vec3d(7, 7, 7)),
			.timeStep = 1.0 / 180.0,
			.stepNum = 350,
			.forwardConvergenceThresh = 1e-9,
			.backwardConvergenceThresh = 5e-4,
			.name = "rotating_sphere"

		};

Simulation::SceneConfiguration OptimizationTaskConfigurations::windScene = {
	.fabric = normalFabric6,
	.orientation = Orientation::FRONT,
	.attachmentPoints = AttachmentConfigs::LEFT_RIGHT_CORNERS_2,
	.trajectory = TrajectoryConfigs::NO_TRAJECTORY,
	.primitiveConfig = PrimitiveConfiguration::NONE,
	.windConfig = WindConfig::WIND_CONSTANT,
	.camPos = Vec3d(-10.38, 4.243, 12.72),
	.camFocusPos = Vec3d(0, 0, 0),
	.camFocusPointType = CameraFocusPointType::CAMERA_POINT,
	.sceneBbox = AABB(Vec3d(-7, -7, -7), Vec3d(7, 7, 7)),
	.timeStep = 1.0 / 90.0,
	.stepNum = 200,
	.forwardConvergenceThresh = 1e-9,
	.backwardConvergenceThresh = 5e-4,
	.name = "wind"

};

Simulation::SceneConfiguration OptimizationTaskConfigurations::tshirtScene = {
	.fabric = tshirt1000,
	.orientation = Orientation::BACK,
	.attachmentPoints = AttachmentConfigs::LEFT_RIGHT_CORNERS_2,
	.trajectory = TrajectoryConfigs::NO_TRAJECTORY,
	.primitiveConfig = PrimitiveConfiguration::NONE,
	.windConfig = WindConfig::WIND_SIN,
	.camPos = Vec3d(-10.38, 4.243, 12.72),
	.camFocusPos = Vec3d(0, 0, 0),
	.camFocusPointType = CameraFocusPointType::CAMERA_POINT,
	.sceneBbox = AABB(Vec3d(-7, -7, -7), Vec3d(7, 7, 7)),
	.timeStep = 1.0 / 90.0,
	.stepNum = 250,
	.forwardConvergenceThresh = 1e-8,
	.backwardConvergenceThresh = 5e-4,
	.name = "wind_tshirt"

};

Simulation::SceneConfiguration OptimizationTaskConfigurations::dressScene = {
	.fabric = dressv7khandsUpDrape, // dressv7k,
	.orientation = Orientation::FRONT,
	.attachmentPoints = AttachmentConfigs::CUSTOM_ARRAY,
	.customAttachmentVertexIdx = { { 0.0,
			{
					1335,
					1336,
					1334,
					1360,
					1339,
					1347,
					1345,
					1342,
					1349,
					1351,
					1352,
					3604,
					1145,
					1150,
					1137,
					1142,
					1143,
					1285,
					3496,
					3497,
					3501,
					1152,
					1153,
					3499,
					3498,
					3500,
					3559,
					1146,
					1333,
					1355,
					1350,
			} }

	}, // v2 points {{0.0, {2657, 24}}},
	.trajectory = TrajectoryConfigs::TRAJECTORY_DRESS_TWIRL,
	.primitiveConfig = PrimitiveConfiguration::NONE,
	.windConfig = WindConfig::NO_WIND,
	.camPos = Vec3d(-10.38, 4.243, 12.72),
	.camFocusPos = Vec3d(0, 0, 0),
	.camFocusPointType = CameraFocusPointType::CAMERA_POINT,
	.sceneBbox = AABB(Vec3d(-7, -7, -7), Vec3d(7, 7, 7)),
	.timeStep = 1.0 / 120.0,
	.stepNum = 125,
	.forwardConvergenceThresh = 1e-10,
	.backwardConvergenceThresh = 5e-4,
	.name = "dress_twirl"

};

Simulation::SceneConfiguration
		OptimizationTaskConfigurations::slopeSimplifiedScene{
			// solution mu: 0-0.8
			.fabric = slopeFabricRestOnPlane,
			.orientation = Orientation::FRONT,
			.upVector = Vec3d(0, 1, 0.0).normalized(),
			.attachmentPoints = AttachmentConfigs::NO_ATTACHMENTS,
			.trajectory = TrajectoryConfigs::NO_TRAJECTORY,
			.primitiveConfig = PrimitiveConfiguration::SLOPE_SIMPLIEFIED,
			.windConfig = WindConfig::NO_WIND,
			.camPos = Vec3d(-10.7, 2.6, 43.8),
			.camFocusPos = Vec3d(-0.9, -33.1, 25.6),
			.camFocusPointType = CameraFocusPointType::CAMERA_POINT,
			.sceneBbox = AABB(Vec3d(-7, -7, -7), Vec3d(7, 7, 7)),
			.timeStep = 1.0 / 100.0,
			.stepNum = 300,
			.forwardConvergenceThresh = 1e-8,
			.backwardConvergenceThresh = 5e-4,
			.name = "slope_simplified"
		};

Simulation::SceneConfiguration OptimizationTaskConfigurations::sockScene{
	// sockLegUp is Vec3d(0, 1, 0)
	.fabric = sock482,
	.orientation = Orientation::CUSTOM_ORIENTATION,
	.upVector = Vec3d(0.0, 1.0, 0.0).normalized(),
	.attachmentPoints = AttachmentConfigs::CUSTOM_ARRAY,
	.customAttachmentVertexIdx = { { 0.0,
			{ 14, 30, 3, 81 } } }, // sock opening back is 81
	.trajectory = TrajectoryConfigs::CORNERS_2_WEARSOCK,
	.primitiveConfig = PrimitiveConfiguration::FOOT,
	.windConfig = WindConfig::NO_WIND,
	.camPos = Vec3d(-32.19, 28.88, 11.32),
	.sockLegOrientation = Vec3d(0, 1, 0),
	.sceneBbox = AABB(Vec3d(-7, -5, -7), Vec3d(7, 15, 6)),
	.timeStep = 1.0 / 320.0,
	.stepNum = 400,
	.forwardConvergenceThresh = 1e-9,
	.backwardConvergenceThresh = 5e-4,
	.name = "wear_sock1"
};

Simulation::TaskConfiguration OptimizationTaskConfigurations::demoWindSim2Real = { .scene = windScene,
	.hasGroundtruth = true,
	.generateGroundtruthSimulation = false,
	.lossType = LossType::MATCH_TRAJECTORY };

Simulation::TaskConfiguration OptimizationTaskConfigurations::demoWInd = {
	.scene = windScene,
	.hasGroundtruth = true,
	.generateGroundtruthSimulation = true,
	.lossType = LossType::MATCH_TRAJECTORY
};

Simulation::TaskConfiguration OptimizationTaskConfigurations::demoTshirt = {
	.scene = tshirtScene,
	.hasGroundtruth = true,
	.generateGroundtruthSimulation = true,
	.lossType = LossType::MATCH_TRAJECTORY
};

Simulation::TaskConfiguration OptimizationTaskConfigurations::demoSphere = {
	.scene = rotatingSphereScene,
	.hasGroundtruth = true,
	.generateGroundtruthSimulation = true,
	.lossType = LossType::MATCH_TRAJECTORY
};

Simulation::TaskConfiguration OptimizationTaskConfigurations::demoHat = {
	.scene = hatScene,
	.hasGroundtruth = false,
	.generateGroundtruthSimulation = false,
	.lossType = LossType::MATCHSHAPE_WITH_TRANSLATION
};

Simulation::TaskConfiguration OptimizationTaskConfigurations::demoSock = {
	.scene = sockScene,
	.hasGroundtruth = false,
	.generateGroundtruthSimulation = false,
	.lossType = LossType::ASSISTED_DRESSING_KEYPOINTS
};

Simulation::TaskConfiguration OptimizationTaskConfigurations::demoSlope = {
	.scene = slopeSimplifiedScene,
	.hasGroundtruth = true,
	.generateGroundtruthSimulation = true,
	.lossType = LossType::MATCHSHAPE_WITH_TRANSLATION
};

Simulation::TaskConfiguration OptimizationTaskConfigurations::demoDress = {
	.scene = dressScene,
	.hasGroundtruth = false,
	.generateGroundtruthSimulation = false,
	.lossType = LossType::DRESS_ANGLE
};

std::map<int, Simulation::TaskConfiguration>
		OptimizationTaskConfigurations::demoNumToConfigMap = {
			{ DEMO_WIND, OptimizationTaskConfigurations::demoWInd },
			{ DEMO_WIND_SIM2REAL, OptimizationTaskConfigurations::demoWindSim2Real },
			{ DEMO_SPHERE_ROTATE, OptimizationTaskConfigurations::demoSphere },
			{ DEMO_WIND_TSHIRT, OptimizationTaskConfigurations::demoTshirt },
			{ DEMO_WEAR_HAT, OptimizationTaskConfigurations::demoHat },
			{ DEMO_WEAR_SOCK, OptimizationTaskConfigurations::demoSock },
			{ DEMO_SLOPE_PERF, OptimizationTaskConfigurations::demoSlope },
			{ DEMO_DRESS_TWIRL, OptimizationTaskConfigurations::demoDress },
		};

std::vector<Simulation::SceneConfiguration>
		OptimizationTaskConfigurations::sceneConfigArrays = {
			windScene, continousNormalScene, rotatingSphereScene, hatScene,
			sockScene, tshirtScene, dressScene
		};
