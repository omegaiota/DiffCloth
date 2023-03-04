//
// Created by Yifei Li on 4/6/21.
// Email: liyifei@csail.mit.edu
//

#ifndef OMEGAENGINE_CONSTANTS_H
#define OMEGAENGINE_CONSTANTS_H

#include "Macros.h"

enum LossType {
	MATCHSHAPE_WITH_TRANSLATION,
	MULTISTEP_MATCHSHAPE,
	MATCHSHAPE_TRANSLATION_INVARINT,
	ASSISTED_DRESSING_KEYPOINTS,
	MATCH_TRAJECTORY,
	MATCH_TRAJECTORY_MAX,
	MATCH_VELOCITY,
	DRESS_ANGLE,

};

static std::vector<std::string> CONSTRAINT_TYPE_STRINGS =
		std::vector<std::string>{ "CONSTRAINT_SPRING", "CONSTRAINT_ATTACHMENT",
			"CONSTRAINT_TRIANGLE",
			"CONSTRAINT_TRIANGLE_BENDING" };
static std::vector<std::string> SPLINE_TYPE_STRINGS = std::vector<std::string>{
	"ENDPOINT", "ENDPOINT_AND_UP", "ENDPOINT_AND_TANGENTS"
};

enum Orientation {
	FRONT,
	DOWN,
	BACK,
	CUSTOM_ORIENTATION,
};
enum AttachmentConfigs { NO_ATTACHMENTS,
	LEFT_RIGHT_CORNERS_2,
	CUSTOM_ARRAY };

enum TrajectoryConfigs {
	NO_TRAJECTORY,
	CORNERS_2_UP,
	CORNERS_2_WEARHAT,
	CORNERS_1_WEARHAT,
	CORNERS_2_WEARSOCK,
	FIXED_POINT_TRAJECTORY,
	TRAJECTORY_DRESS_TWIRL,
	PER_STEP_TRAJECTORY
};

enum WindConfig {
	NO_WIND,
	WIND_CONSTANT,
	WIND_SIN,
	WIND_SIN_AND_FALLOFF,
	WIND_FACTOR_PER_STEP
};

static std::vector<std::string> windConfigStrings =
		std::vector<std::string>{ "NO_WIND", "WIND_CONSTANT", "WIND_SIN",
			"WIND_SIN_AND_FALLOFF", "WIND_FACTOR_PER_STEP" };
enum Demos {
	DEMO_WIND, // 0
	DEMO_WIND_TSHIRT,
	DEMO_WIND_SIM2REAL, // 2
	DEMO_WEAR_HAT, // 3
	DEMO_WEAR_SOCK, // 4
	DEMO_SLOPE_PERF, // 5
	DEMO_DRESS_TWIRL, // 6
	DEMO_SPHERE_ROTATE, // 7
};

static std::vector<std::string> DEMOS_STRINGS = std::vector<std::string>{
	"WIND", "WIND_TSHIRT", "WIND_SIM2REAL", "HAT",
	"SOCK", "DEMO_SLOPE_PERF", "DEMO_DRESS_TWIRL0.3",
	"SPHERE", // 19
};

enum PrimitiveConfiguration {
	PLANE_BUST_WEARHAT,
	SLOPE,
	SLOPE_SIMPLIEFIED,
	PLANE_AND_SPHERE,
	FOOT,
	NONE,
	Y0PLANE,
	BIG_SPHERE
};

enum CameraFocusPointType { ORIGIN,
	CLOTH_CENTER,
	PRIM0CENTER,
	CAMERA_POINT };

enum Optimizer { LBFGS };

enum FabricEnumArray {
	NORMAL,
	NORMAL_6_LOWRES,
	TSHIRT1000,
	SLOPE_FAB,
	SLOPE_FAB_ONPLINE,
	DRESS_v7k_DRAPE,
	AGENT_HAT579,
	SOCK482,
	CONTINUOUS_NORMAL_FABRIC
};

enum SceneConfigArray {
	SCENE_WIND,
	SCENE_CONTINUOUS_NORMAL,
	SCENE_NAPKIN,
	SCENE_ROTATING_SPHERE,
	SCENE_DEMO_HAT,
	SCENE_DEMO_SOCK,
	SCENE_DEMO_SLOE,
	SCENE_DEMO_TSHIRT,
	SCENE_DEMO_DRESS,
	SCENE_NONE
};

static std::vector<std::string> SCENE_CONFIG_STRINGS = std::vector<std::string>{
	"WIND", "CONTINUOUS_NORMAL", "ROTATING_SPHERE",
	"DEMO_HAT", "DEMO_SOCK", "DEMO_SLOPE",
	"DEMO_TSHIRT", "DEMO_DRESS", "NONE"
};

static Vec3d COLOR_IBM_ULTRAMARINE40 =
		Vec3d(100 / 255.0, 143 / 255.0, 255 / 255.0);
static Vec3d COLOR_IBM_INDIGO50 = Vec3d(120 / 255.0, 94 / 255.0, 240 / 255.0);
static Vec3d COLOR_IBM_MAGENTA50 = Vec3d(220 / 255.0, 38 / 255.0, 127 / 255.0);
static Vec3d COLOR_IBM_ORANGE40 = Vec3d(254 / 255.0, 97 / 255.0, 0);
static Vec3d COLOR_IBM_GOLD20 = Vec3d(1.0, 176 / 255.0, 0);

static Vec3d COLOR_RED = Vec3d(0.28, 0, 0);
static Vec3d COLOR_BURGUNDY = Vec3d(0.235, 0.085, 0.107);
static Vec3d COLOR_LAVENDER = Vec3d(0.351, 0.191, 0.738);
static Vec3d COLOR_EGGPLANT = Vec3d(0.203, 0.0998, 0.748);
// static Vec3d COLOR_LAVENDER(0.350, 0.0936, 0.561);
static Vec3d COLOR_PEACH = Vec3d(0.738, 0.191, 0.324);
static Vec3d COLOR_GRAY57 = Vec3d(0.57, 0.57, 0.57);
static Vec3d COLOR_GRAY70 = Vec3d(0.7, 0.7, 0.7);
static Vec3d COLOR_GRAY50 = Vec3d(0.5, 0.5, 0.5);
static Vec3d COLOR_NAVY = Vec3d(0.011, 0.12, 0.65);
static Vec3d COLOR_SKY = Vec3d(0.191, 0.535917, 0.738);
static Vec3d COLOR_ICE = Vec3d(0.352, 0.554, 0.663);
static Vec3d COLOR_YELLOW = Vec3d(0.738, 0.68032, 0.191);
static Vec3d COLOR_SKIN = Vec3d(0.626, 0.307, 0.290);
static Vec3d COLOR_WOOD = Vec3d(0.671993, 0.395403, 0.0829298);
static std::string OUTPUT_PARENT_FOLDER = std::string(SOURCE_PATH) + "/output/";
#endif // OMEGAENGINE_CONSTANTS_H