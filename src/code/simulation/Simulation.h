//
// Created by Yifei Li on 9/24/20.
//

#ifndef OMEGAENGINE_SIMULATION_H
#define OMEGAENGINE_SIMULATION_H

#include "../engine/Constants.h"
#include "../engine/Macros.h"
#include "../engine/MeshFileHandler.h"
#include "../engine/Timer.h"
#include "../engine/UtilityFunctions.h"
#include "AttachmentSpring.h"
#include "Constraint.h"
#include "FixedPoint.h"
#include "Particle.h"
#include "Primitive.h"
#include "Spline.h"
#include "Spring.h"
#include "Triangle.h"
#include "TriangleBending.h"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <iomanip>
#include <memory>
#include <set>
#include <vector>

class Simulation {
public:
	enum CollisionType { TAKE_OFF,
		STICK,
		SLIDE,
		NUM_CASES };

	struct PrimitiveCollisionInformation {
		int primitiveId;
		int particleId;
		Vec3d normal;
		Vec3d v_out;
		bool collides;
		double dist;
		int primTotalCollision;
		Vec3d r;
		Vec3d d;
		CollisionType type;
	};

	struct SelfCollisionInformation {
		int particleId1;
		int particleId2;
		Vec3d normal;
		Vec3d d;
		Vec3d r;
		bool collides;
		double dist;
		int layerId;
		CollisionType type;
	};

	typedef std::pair<std::vector<PrimitiveCollisionInformation>,
			std::vector<SelfCollisionInformation>>
			collisionInfoPair;
	typedef std::pair<collisionInfoPair,
			std::vector<std::vector<SelfCollisionInformation>>>
			completeCollisionInfo;
	struct ForwardInformation {
		VecXd x;
		VecXd v;
		VecXd x_prev;
		VecXd v_prev;
		VecXd f;
		VecXd r;
		VecXd At_p_weightless_pertype[Constraint::CONSTRAINT_NUM];
		VecXd s_n;
		VecXd x_prim;
		VecXd v_prim;
		VecXd vpre_prim;
		VecXd f_prim;
		VecXd x_fixedpoints;
		Vec5d windParams;
		completeCollisionInfo collisionInfos;
		TimerContent timer;
		std::vector<TimerEntry> accumTimer;
		std::vector<Spline> splines;
		int sysMatId = 0;
		long long totalRuntime;
		double simDurartionFraction;
		double avgDeformation;
		double maxDeformation;
		double windFactor;
		double t;
		bool converged;
		int convergeIter;
		int totalConverged;
		int cumulateIter;
		int stepIdx;
		double loss; // last frame, if applicable
	};

	struct FabricConfiguration {
		double clothDimX;
		double clothDimY;
		double k_stiff_stretching;
		double k_stiff_bending;
		int gridNumX;
		int gridNumY;
		double density;
		bool keepOriginalScalePoint;
		bool isModel;
		bool custominitPos;
		std::string initPosFile;
		int fabricIdx; // in SimulationConstant::fabricArrays
		Vec3d color;
		std::string name; // if ismodel, this is the path to the mesh model;
						  // otherwise describes a fabric
	};

	struct ParamInfo {
		VecXd x0;
		VecXd v0;
		VecXd f_ext;
		VecXd f_ext_timestep;
		Vec5d f_extwind;
		VecXd f_constantForceField;
		std::vector<VecXd> x_fixed;

		double density;
		double k_pertype[Constraint::CONSTRAINT_NUM];
		std::vector<std::vector<Spline>> controlPointSplines;
		std::vector<std::pair<int, double>> mu;
	};

	struct BackwardInformation {
		VecXd dL_dx;
		VecXd dL_dv;
		VecXd dL_dconstantForceField;
		Vec3d dL_dfext;
		Vec5d dL_dwind; // vec3(force),period,phase
		VecXd dL_dwindtimestep;
		double dL_ddensity;
		double dL_dk_pertype[Constraint::CONSTRAINT_NUM];
		std::vector<std::vector<VecXd>> dL_dsplines;
		VecXd dL_dxfixed;
		VecXd dL_dxfixed_accum;
		std::vector<std::pair<int, double>> dL_dmu;
		int badMatrixCounter;
		int goodMatrixCounter;
		double loss;
		long long totalRuntime;
		TimerContent timer;
		std::vector<TimerEntry> accumTimer;
		PerformanceTiming accumSolvePerformanceReport;
		bool converged;
		int convergedAccum;
		int backwardIters;
		int backwardTotalIters;
		int correspondingForwardIdxInStats;
		double rho;
	};

	BackwardInformation backwardInfoDefault = {
		.dL_dx = VecXd(0),
		.dL_dv = VecXd(0),
		.dL_dconstantForceField = VecXd(0),
		.dL_dfext = Vec3d(0, 0, 0),
		.dL_dwind = MatXd::Zero(5, 1),
		.dL_ddensity = 0.0,
		.dL_dk_pertype = { 0.0, 0.0, 0.0, 0.0 },
		.dL_dsplines = {},
		.dL_dmu = { { 0, 0 } },
		.badMatrixCounter = 0,
		.goodMatrixCounter = 0,
		.loss = 0,
		.totalRuntime = 0,
		.timer = {},
		.accumTimer = {},
		.accumSolvePerformanceReport = {},
		.converged = false,
		.convergedAccum = 0,
		.backwardIters = 0,
		.backwardTotalIters = 0,
		.rho = 0
	};

	struct BackwardTaskInformation {
		bool dL_dk_pertype[Constraint::CONSTRAINT_NUM];
		bool dL_density;
		bool dL_dfext;
		bool dL_dconstantForceField;
		bool dL_dfwind;
		bool adddr_dd;
		bool dL_dcontrolPoints;
		bool dL_dxfixed;
		bool dL_dmu;
		bool dL_dx0;
		bool dL_dwindFactor;
		double forwardAccuracyLevel;
		double backwardAccuracyLevel;
		std::vector<std::vector<Spline::SplineType>> splineTypes;
		std::vector<int> mu_primitives;
		Optimizer optimizer;
		ParamInfo paramActual;
		int randSeed;
		int srandSeed;
	};

	struct TaskSolveStatistics {
		int totalForwardSim;
		int totalBackprop;
		int forwardWritten;
		int backwardWritten;
		int optimizationRecordsSaved;
		bool configWritten = false;
		std::string experimentName;

		std::vector<std::pair<ParamInfo, ForwardInformation>>
				completeForwardLog; // For each simulation, only log the last frame of
									// forward sim
		std::vector<std::pair<ParamInfo, BackwardInformation>>
				completeBackwardLog; // For each backprop, only log the first frame of
									 // backprop

		TaskSolveStatistics() {
			optimizationRecordsSaved = 0;
			totalForwardSim = 0;
			totalBackprop = 0;
			forwardWritten = 0;
			backwardWritten = 0;
			configWritten = false;
			experimentName = "";
		}
	};

	static BackwardTaskInformation taskConfigDefault; // all disabled

	struct CorresPondenceTargetInfo {
		int frameIdx;
		Vec3d targetPos;
		std::vector<int> particleIndices;

		CorresPondenceTargetInfo(int idx, Vec3d pos, std::vector<int> particleIdx) :
				frameIdx(idx), targetPos(pos), particleIndices(particleIdx) {}
	};

	struct LossInfo {
		Vec3d targetLoc;
		Vec3d targetTranslation;
		VecXd targetShape;
		double targetTwirlHeight; // 1.0 maxTwirl, 0.0 no twirl
		std::vector<Simulation::ForwardInformation> targetSimulation;
		std::vector<CorresPondenceTargetInfo> targetPosPairs;
		std::vector<std::pair<int, VecXd>> targetFrameShape;
		std::set<int> loopPoints;
	};

	struct SystemConfiguration {
		VecXd x0;
		VecXd v0;
		double k_stiff;
	};

	struct SceneConfiguration {
		FabricConfiguration fabric; // in SimulationConstant::fabricArrs
		Orientation orientation;
		Vec3d upVector;
		AttachmentConfigs attachmentPoints;
		std::vector<std::pair<double, std::vector<int>>>
				customAttachmentVertexIdx; // outside loop: different sets; each
										   // element: startFrame x vIdx
		TrajectoryConfigs trajectory;
		PrimitiveConfiguration primitiveConfig;
		WindConfig windConfig;
		Vec3d camPos;
		Vec3d camFocusPos;
		Vec3d sockLegOrientation;
		CameraFocusPointType camFocusPointType;
		AABB sceneBbox;
		double timeStep;
		int stepNum;
		double forwardConvergenceThresh;
		double backwardConvergenceThresh;
		std::string name;
	};

	struct TaskConfiguration {
		SceneConfiguration scene;
		bool hasGroundtruth;
		bool generateGroundtruthSimulation;
		LossType lossType;
	};

	struct FileMesh {
		std::string path;
		double dim;
		bool flipWinding;
		std::vector<Vec3d> points;
		std::vector<Vec3d> normals;
		std::vector<Vec3i> triangles;

		void getMesh(std::vector<Vec3i> &mesh, VecXd &primPos, Vec3d center) {
			primPos = VecXd(points.size() * 3);
			primPos.setZero();

			for (Vec3i &t : triangles)
				mesh.emplace_back(t);

			for (int i = 0; i < points.size(); i++) {
				primPos.segment(i * 3, 3) = points[i] + center;
			}
		}
	};

	int totalIter = 0, currentSysmatId = 0;
	SystemConfiguration myConfig;
	//    static AttachmentConfigs restPosConfiguration;
	SceneConfiguration sceneConfig;
	bool backwardGradientForceDirectSolver = false;
	static volatile bool gravityEnabled, windEnabled, staticEnabled,
			contactEnabled, selfcollisionEnabled, enableConstantForcefield;
	static volatile bool bendingEnabled;

	bool debugSkipfixedpointstep = false;
	bool useCustomRLFixedPoint = false;
	bool gradientClipping = true;
	double gradientClippingThreshold = 16.0;

	VecXd rlFixedPointPos;
	std::vector<VecXd> fixedPointTrajectory;

	static double windNorm, windFrequency, windPhase;
	static double forwardConvergenceThreshold, backwardConvergenceThreshold;
	static int PD_TOTAL_ITER;
	bool runBackward, calcualteSeperateAt_p = false;
	int uniqueID = 0;

	std::vector<CorresPondenceTargetInfo> debugPointTargetPos;
	std::vector<std::pair<int, VecXd>> debugShapeTargetPos;
	std::set<int> debugLoopPoints;

	Vec3d restShapeMinDim, restShapeMaxDim, restShapeMidPoint,
			meshInitTransofrmMinDim;
	double meshInitTransofrmScale;
	VecXd windFallOff;
	VecXd perstepWindFactor;
	std::vector<Vec3d> modelPoints;
	std::vector<Vec3i> modelTris;
	FileMesh arrow1, arrow2, arrow3, clip;
	Vec3d lightPos = Vec3d(0, 10.0f, 0);
	Vec3d lightDir = Vec3d(0, -1.0, -0.7);
	Vec3d gravity{ 0, -9.8, 0 };
	Vec3d wind{ 0.01, 0, 1 };
	Vec3d systemCenter;
	SpMat dproj_dxnew, dproj_dxnew_t;

	struct SystemMatrix { // All these matrices are constraint weight dependent,
						  // so if constraint weight change, these matrices change
		// Additionally, in our simulation sometimes the constraints need to be
		// changed because of change of attachment constraints
		SpMat P;
		SpMat A;
		SpMat A_t;
		SpMat C;
		SpMat C_t;
		SpMat A_t_times_A_pertype[Constraint::CONSTRAINT_NUM];
		SpMat A_pertype[Constraint::CONSTRAINT_NUM]; // without weight
		SpMat A_t_pertype[Constraint::CONSTRAINT_NUM];
		SpMat A_t_dp_dxfixed;
		int constraintNum;
		int constraintNum_pertype[Constraint::CONSTRAINT_NUM]; // without weight
		int startFrameNum;
		std::vector<Constraint *> constraints;
		std::vector<AttachmentSpring> attachments;
		std::vector<FixedPoint> fixedPoints;
		std::vector<Spline> controlPointSplines;
		Eigen::SimplicialLLT<SpMat> solver;

		SystemMatrix() {}

		SystemMatrix(const SystemMatrix &other) {
			P = other.P;
			A = other.A;
			C = other.C;
			C_t = other.C_t;
			A_t = other.A_t;
			attachments = other.attachments;
			constraints = other.constraints;
			A_t_dp_dxfixed = other.A_t_dp_dxfixed;
			controlPointSplines = other.controlPointSplines;
			fixedPoints = other.fixedPoints;
			constraintNum = other.constraintNum;

			for (int i = 0; i < Constraint::CONSTRAINT_NUM; i++) {
				A_t_times_A_pertype[i] = other.A_t_times_A_pertype[i];
				A_pertype[i] = other.A_pertype[i];
				A_t_pertype[i] = other.A_t_pertype[i];
				constraintNum_pertype[i] = other.constraintNum_pertype[i];
			}
			P = factorizeDirectSolverLLT(P, solver, "sysmat");
		}
	};

	std::vector<SystemMatrix> sysMat;

	static double *k_stiff_arr[Constraint::CONSTRAINT_NUM];

	VecXd projections_pertype[Constraint::CONSTRAINT_NUM];
	Eigen::SparseMatrix<double> M, M_inv, Area, Area_inv;
	VecXd projections;

	VecXd s_n; // intertia term defined in PD (Bouazziz 2014)
	VecXd x_n, v_n, gravity_n,
			external_force_field; // velocity and position from previous solved state
								  // / beginning of current timestep
	VecXd m_primitives, m_primitivesinv; // primitive mass vectors
	SpMat dxnew_dxfixed_rhs, P_inv, I;
	Sphere sphere5, sphere2, sphere_head, sphereForFixedPointRender,
			veryBigSphere;
	Bowl bowl;
	LowerLeg sockLeg, leftLeg, rightLeg;
	Capsule body, neck, head, leftUpperArm, rightUpperArm, leftLowerArm,
			rightLowerArm;
	Plane plane1, slope;
	Sphere slopeGoal;
	bool explosionEncountered;
	Timer timeSteptimer;

	std::vector<Particle> particles;
	std::vector<Spring> springs;
	std::vector<Triangle> mesh;
	std::vector<TriangleBending> bendingConstraints;
	std::vector<std::vector<int>> particleTriangleMap;
	std::vector<std::vector<bool>> pointpointConnectionTable;
	std::vector<VecXd> perstepTrajectory, perStepGradient;

	std::vector<Primitive *> primitives, allPrimitivesToRender;
	std::vector<std::string> log;
	std::vector<ForwardInformation> forwardRecords, groundTruthForwardRecords,
			linesearchRecords;
	std::vector<std::pair<std::vector<ForwardInformation>,
			std::vector<BackwardInformation>>>
			backwardOptimizationRecords;
	Simulation::BackwardTaskInformation taskInfo;
	std::vector<std::pair<Simulation::ParamInfo, double>>
			backwardOptimizationGuesses;
	Simulation::ParamInfo groundtruthParam;
	std::vector<std::vector<BackwardInformation>> backwardRecordsFiniteDiff;
	Timer timer;

	Eigen::SimplicialLLT<SpMat> Msolver;
	Eigen::SparseLU<SpMat> solverSparseLU;

	Simulation(Vec3d center) :
			systemCenter(center),
			sphere5(Vec3d(0, 0, 0), 5, Vec3d(0.352, 0.554, 0.663)),
			veryBigSphere(Vec3d(0, 0, 0), 15, Vec3d(0.352, 0.554, 0.663)),
			bowl(Vec3d(0, 0.5, 0), 0.5, COLOR_GRAY50),
			slopeGoal(Vec3d(0, 0, 0), 2, COLOR_IBM_MAGENTA50),
			sockLeg(Vec3d(0, 0, 0), Vec3d(0.4, 1, -0.2).normalized(), 5, 4),
			rightLeg(Vec3d(0, 0, 0), Vec3d(0, 1, -0.3), 5, 4),
			leftLeg(Vec3d(0, 0, 0), Vec3d(0, 1, -0.3), 5, 4),
			leftUpperArm(Vec3d(0, 0, 0), 0.6, 2.2, Vec3d(0, 1, 0),
					Vec3d(-1, -0.2, 0)),
			rightUpperArm(Vec3d(0, 0, 0), 0.6, 2.2, Vec3d(0, 1, 0),
					Vec3d(1, -0.2, 0)),
			leftLowerArm(Vec3d(0, 0, 0), 0.6, 2, Vec3d(0, 1, 0),
					Vec3d(-1, -0.4, 0)),
			rightLowerArm(Vec3d(0, 0, 0), 0.6, 2, Vec3d(0, 1, 0),
					Vec3d(1, -0.4, 0)),
			body(Vec3d(0, 0, 0), 2, 3.3, Vec3d(0, 1, 0), Vec3d(0, 1, 0)),
			neck(Vec3d(0, 0, 0), 0.6, 0.5, Vec3d(0, 1, 0), Vec3d(0, 1, 0)),
			head(Vec3d(0, 0, 0), 1, 1.5, Vec3d(0, 1, 0), Vec3d(0, 1, 0)),
			sphere2(Vec3d(0, 0, 0), 2, Vec3d(0.352, 0.554, 0.663)),
			sphere_head(Vec3d(0, 0, 0), 2.1, COLOR_GRAY57),
			sphereForFixedPointRender(Vec3d(0, 0, 0), 0.07, Vec3d(0.5, 0.5, 0), 6),
			plane1(Vec3d(0, 0, 5), Vec3d(-10, 0, -4), Vec3d(10, 0, -4),
					COLOR_GRAY50),
			slope(Vec3d(0, -11, 10), Vec3d(-8, -1, -1), Vec3d(8, -1, -1),
					COLOR_GRAY50),
			printVerbose(true) {
		loadSceneMeshes();
		SystemMatrix mat;
		sysMat.emplace_back(mat);
		sysMat.reserve(5);
		backwardOptimizationRecords.reserve(10);

		sceneConfig.fabric.clothDimX = 6;
		sceneConfig.fabric.clothDimY = 6;
		sceneConfig.fabric.k_stiff_stretching = 1.2;
		sceneConfig.fabric.density = 0.000324;
		sceneConfig.fabric.k_stiff_bending = 0.01;
		sceneConfig.orientation = Orientation::FRONT;
		sceneConfig.attachmentPoints = AttachmentConfigs::LEFT_RIGHT_CORNERS_2;
		sceneConfig.trajectory = NO_TRAJECTORY;
		sceneConfig.primitiveConfig = PrimitiveConfiguration::PLANE_AND_SPHERE;
		sceneConfig.timeStep = 1.0 / 60;
		sceneConfig.stepNum = 100;
	}

	static Simulation *createSystem(Simulation::SceneConfiguration sceneConfig,
			Vec3d center, bool runBackward = true);

	~Simulation() {}

	void enableStaticPoint(bool v) {
		staticEnabled = v;
		updateMassMatrix();
	}

	void updateBackwardDefaultInfo() {
		backwardInfoDefault.dL_dsplines.resize(sysMat.size());
		backwardInfoDefault.dL_dxfixed.resize(sysMat[0].fixedPoints.size() * 3);
		backwardInfoDefault.dL_dxfixed_accum.resize(sysMat[0].fixedPoints.size() *
				3);
		backwardInfoDefault.dL_dxfixed.setZero();
		backwardInfoDefault.dL_dxfixed_accum.setZero();
		backwardInfoDefault.dL_dconstantForceField = VecXd(particles.size() * 3);
		backwardInfoDefault.dL_dconstantForceField.setZero();

		for (int sysMatId = 0; sysMatId < sysMat.size(); sysMatId++) {
			backwardInfoDefault.dL_dsplines[sysMatId].resize(
					sysMat[sysMatId].controlPointSplines.size());
			for (int i = 0; i < sysMat[sysMatId].controlPointSplines.size(); i++) {
				backwardInfoDefault.dL_dsplines[sysMatId][i] =
						VecXd(sysMat[sysMatId].controlPointSplines[i].getParameterNumber());
				backwardInfoDefault.dL_dsplines[sysMatId][i].setZero();
			}
		}
	}

	void setWindAncCollision(bool windEnable, bool collisionEnable,
			bool selfCollisionEnable,
			bool enableConstantForcefield = false) {
		Simulation::windEnabled = windEnable;
		Simulation::contactEnabled = collisionEnable;
		Simulation::selfcollisionEnabled = selfCollisionEnable;
		Simulation::enableConstantForcefield = enableConstantForcefield;
	}

	static glm::vec3 getLookAtPos(Simulation *system,
			Simulation::SceneConfiguration &profile) {
		Vec3d p(0, 0, 0);
		switch (profile.camFocusPointType) {
			case CameraFocusPointType::ORIGIN: {
				p = Vec3d(0, 0, 0);
				break;
			}
			case CameraFocusPointType::CLOTH_CENTER: {
				p = (system->restShapeMinDim + system->restShapeMaxDim) * 0.5;
				break;
			}
			case CameraFocusPointType::PRIM0CENTER: {
				p = system->primitives[0]->center;
				break;
			}

			case CameraFocusPointType::CAMERA_POINT: {
				p = system->sceneConfig.camFocusPos;
			}
		}

		return glm::vec3(p[0], p[1], p[2]);
	}

	BackwardInformation
	stepBackwardNN(Simulation::BackwardTaskInformation &taskInfo, VecXd &dL_dxnew,
			VecXd &dL_dvnew, const ForwardInformation &forwardInfo_new,
			bool isStart, const VecXd &dL_dxinit, const VecXd &dL_dvinit);

	BackwardInformation
	stepBackward(Simulation::BackwardTaskInformation &taskInfo,
			BackwardInformation &gradient_new,
			const ForwardInformation &forwardInfo_new, bool isStart,
			const VecXd &dL_dxinit, const VecXd &dL_dvinit);

	void resetForwardRecordsFromFolder(std::string subFolder) {
		std::vector<Vec3d> modelPoints;
		std::vector<Vec3i> modelTris;
		std::string fullFolderPath =
				std::string(SOURCE_PATH) + "/output/" + subFolder + "/";
		std::vector<std::string> frameFileNames = listFiles(fullFolderPath, ".obj");
		std::sort(frameFileNames.begin(), frameFileNames.end(),
				[](const std::string a, const std::string b) -> bool {
					int idxA = std::stoi(a.substr(0, a.length() - 4));
					int idxB = std::stoi(b.substr(0, b.length() - 4));
					return idxA < idxB;
				});
		Logging::logNormal("Loaded " + std::to_string(frameFileNames.size()) +
				" cloth frame files from folder " + fullFolderPath +
				"\n");
		VecXd x_i(3 * particles.size()), empty(3 * particles.size());
		empty.setZero();
		ForwardInformation recordInit = forwardRecords[0];

		for (int i = 0; i < frameFileNames.size(); i++) {
			std::string fName = fullFolderPath + frameFileNames[i];
			modelPoints.clear();
			modelTris.clear();
			MeshFileHandler::loadOBJFile(fName.c_str(), modelPoints, modelTris);
			for (int pIdx = 0; pIdx < particles.size(); pIdx++) {
				x_i.segment(pIdx * 3, 3) = modelPoints[pIdx];
			}
			ForwardInformation record = recordInit;
			record.t = sceneConfig.timeStep * i;
			record.x = x_i;
			record.stepIdx = i;
			record.x_fixedpoints = VecXd(3 * sysMat[0].fixedPoints.size());
			VecXd clipPosition(3 * sysMat[0].fixedPoints.size());
			clipPosition.setZero();
			for (AttachmentSpring &s : sysMat[0].attachments) {
				record.x_fixedpoints.segment(3 * s.pfixed_idx, 3) =
						record.x.segment(3 * s.p1_idx, 3);
			}

			forwardRecords.emplace_back(record);
			for (Primitive *p : primitives) {
				p->forwardRecords.emplace_back(p->forwardRecords[0]);
				if (p->isPrimitiveCollection) { // TODO: this is a hacky way to reset
												// primitive trees with <= 2 levels;
												// should be changed to recursion
					for (Primitive *pChild : p->primitives) {
						pChild->forwardRecords.emplace_back(pChild->forwardRecords[0]);
					}
				}
			}
		}
	}
	void clearConstraintsElementsAndRecords();

	void loadSceneMeshes();

	void createClothMesh();

	void createClothMeshFromConfig();

	void createClothMeshFromModel(std::vector<Vec3d> &particleIn,
			std::vector<Vec3i> &meshIn);

	void createBendingConstraints();

	void createAttachments(bool isModel);

	void createConstraints();

	void updateCollisionRadii();

	void restoreToSingleRecordFromCurrentState();

	void rotatePointsAccordingToConfig(std::vector<Vec3d> &particleIn) {
		switch (sceneConfig.orientation) {
			case Orientation::CUSTOM_ORIENTATION: {
				std::printf("orientation custom: rotate\n");
				particleIn = rotatePointsAroundCenter(
						particleIn, axisToRotation(sceneConfig.upVector, Vec3d(0, 1, 0)));
				break;
			}

			case Orientation::FRONT: {
				std::printf("orientation front: nothing\n");

				break;
			}
			case Orientation::DOWN: {
				std::printf("orientation down: rotate\n");
				particleIn = rotatePointsAroundCenter(
						particleIn, axisToRotation(Vec3d(0, 1, 0), Vec3d(0, 0, 1)));
				break;
			}

			case Orientation::BACK: {
				std::printf("orientation down: rotate\n");
				particleIn = rotatePointsAroundCenter(
						particleIn, axisToRotation(Vec3d(0, 0, 1), Vec3d(1, 0, 0)) * axisToRotation(Vec3d(1, 0, 0), Vec3d(0, 0, -1)));
				break;
			}

			default: {
				break;
			}
		}
	}

	std::vector<Vec3d> rotatePointsAroundCenter(std::vector<Vec3d> &particleIn,
			Rotation rotation);

	void loadWindSim2RealAnimationSequence(std::string folderName,
			std::vector<std::string> files,
			bool isRyanWhite = true);

	void initScene();

	SelfCollisionInformation isSelfCollision(const Particle &a, const Particle &b,
			const Vec3d &x_a, const Vec3d &x_b,
			const Vec3d &v_a,
			const Vec3d &v_b) const;

	Simulation::SelfCollisionInformation isSelfCollision(const Triangle &a,
			const Particle &p3,
			const Vec3d v_a,
			const Vec3d v_b) const;

	PrimitiveCollisionInformation
	isInContactWithObstacle(const Vec3d &pos, const Vec3d &v_in,
			const VecXd &x_prim_new,
			const VecXd &v_prim_new) const;

	void stepPrimitives(VecXd &x_n, VecXd &v_n, VecXd &delta_v);

	void updateMassMatrix();

	void updateAreaMatrix();

	void appendPerStepGradient(VecXd &x);

	int gridIndicesToParticle(int a, int b) const;

	void step();

	void stepNN(int idx, const VecXd &x, const VecXd &v,
			const VecXd &fixedPointPos);

	void stepPrimitives();

	void unstepPrimitives();

	double stepFixPoints(double t);

	void writeMatrix(std::ofstream &myfile, MatXd &A, std::string name,
			int precision) {
		myfile << name.c_str() << "\n";
		myfile << A.rows() << " " << A.cols() << "\n";
		for (int i = 0; i < A.rows(); i++) {
			for (int j = 0; j < A.cols(); j++) {
				myfile << d2str(A(i, j), precision) << " ";
			}
			myfile << "\n";
		}
	}

	void writeMatrix(std::ofstream &myfile, SpMat &A, std::string name,
			int precision) {
		MatXd Adense = A.toDense();
		myfile << name.c_str() << "\n";
		myfile << A.rows() << " " << A.cols() << "\n";

		for (int i = 0; i < A.rows(); i++) {
			for (int j = 0; j < A.cols(); j++) {
				myfile << d2str(Adense(i, j), precision) << " ";
			}
			myfile << "\n";
		}
	}

	void writeMatrixSparse(std::ofstream &myfile, SpMat &A, std::string name,
			int precision) {
		MatXd Adense = A.toDense();
		myfile << name.c_str() << "\n";
		myfile << A.nonZeros() << "\n";
		myfile << A.rows() << " " << A.cols() << "\n";

		for (int k = 0; k < A.outerSize(); ++k) {
			for (SpMat::InnerIterator it(A, k); it; ++it) {
				int r = it.row();
				int c = it.col();
				double elem = Adense(it.row(), it.col());
				myfile << r << " " << c << " " << d2str(elem, precision) << "\n";
			}
		}
	}

	void writeMatrix(std::ofstream &myfile, VecXd &delta_P, std::string name,
			int precision) {
		myfile << name.c_str() << "\n";
		myfile << delta_P.rows() << " " << delta_P.cols() << "\n";
		for (int i = 0; i < delta_P.rows(); i++) {
			for (int j = 0; j < delta_P.cols(); j++) {
				myfile << std::fixed << std::setprecision(precision) << delta_P(i, j)
					   << " ";
			}
			myfile << "\n";
		}
	}

	void updateParticleNormals(const VecXd &x_now);

	double calculateTriangleDeformation(VecXd &x_new);

	double calculateMaxTriangleDeformation(VecXd &x_new);

	static VecXd getParticleNormals(std::vector<Triangle> mesh,
			const VecXd &x_now);

	double calculateLossAndGradient(LossType &lossType, LossInfo &lossInfo,
			VecXd &dL_dx, VecXd &dL_dv, int idx,
			bool calculateLoss);

	void resetSystemWithParams(BackwardTaskInformation &taskConfiguration,
			ParamInfo &param);

	void calculateFiniteDiffLossArr(
			ParamInfo paramPlus, ParamInfo paramMinus, double delta,
			std::vector<ForwardInformation> &centerForward,
			std::vector<double> &L_arr, BackwardTaskInformation taskConfiguration,
			LossType lossType, LossInfo &lossInfo, int FORWARD_STEPS,
			const std::function<void(const std::string &value)> &setTextBoxCB =
					[](const std::string &v) {});
	;

	std::vector<BackwardInformation> finiteDifferenceBackward(
			BackwardTaskInformation taskConfiguration, LossType lossType,
			LossInfo &lossInfo, int FORWARD_STEPS, ParamInfo guess,
			const std::function<void(const std::string &value)> &setTextBoxCB =
					[](const std::string &v) {});

	static std::string
	forwardInfoToString(BackwardTaskInformation taskInfo,
			const Simulation::ForwardInformation &forwardInfo) {
		std::string out;
		out += "============Forward Info:======================\n";
		out += "Total PD Iters:" + std::to_string(forwardInfo.cumulateIter) + "\n";
		out +=
				"Total Frames Converged:" + std::to_string(forwardInfo.totalConverged) +
				"\n";
		out += "Forward Total Runtime[ms]:" +
				d2str((double)(forwardInfo.totalRuntime * 1.0 / 1000.0), 5) + "\n";
		out += "Loss:" + d2str(forwardInfo.loss, 5) + "\n";

		out += "============Timer Breakdown:======================\n";
		for (const TimerEntry &breakdown : forwardInfo.accumTimer) {
			std::string label = breakdown.first;
			long long duration = breakdown.second;
			double percentage = ((double)duration * 1.0) / forwardInfo.totalRuntime;
			out += label + "[ms]:" + d2str(duration / 1000.0, 5) + "\t\t|" +
					d2str(double(percentage * 100), 3) + "%\n";
		}

		return out;
	}

	static std::string backwrdInfoAndGradToString(
			const Simulation::BackwardTaskInformation &taskInfo,
			BackwardInformation grad);

	static std::string
	parameterToString(const Simulation::BackwardTaskInformation &taskInfo,
			ParamInfo paramGuess);

	static void appendStringToFile(std::string fileName, std::string content) {
		std::ofstream myfile;
		myfile.open(fileName.c_str(), std::ofstream::app);
		myfile << content;
		myfile.close();
	}

	static std::string
	taskInfoToString(const Simulation::BackwardTaskInformation &taskInfo);

	void exportStatistics(Demos demoIdx, TaskSolveStatistics &statistics,
			BackwardTaskInformation taskInfo,
			bool writePerf = false);

	void exportOptimizationRecords(Demos demoIdx,
			std::string experimentNameGiven = "",
			bool useGiven = false);

	void exportConfig(int demoIdx, SceneConfiguration &config,
			std::string fileName);

	void exportFrameInfo(Simulation *system,
			Simulation::ForwardInformation &record,
			std::string fileName) {
		std::ofstream myfile;
		myfile.open(fileName);

		for (int fixedPointId = 0; fixedPointId < record.x_fixedpoints.rows() / 3;
				fixedPointId++) {
			Vec3d clipPos = record.x_fixedpoints.segment(fixedPointId * 3, 3);
			myfile << "CLIP_" << fixedPointId << ":" << d2str(clipPos[0], 5) << ","
				   << d2str(clipPos[1], 5) << "," << d2str(clipPos[2], 5) << "\n";
		}

		myfile.close();
	}

	void exportStatsSimulations(
			std::vector<std::pair<ParamInfo, ForwardInformation>> &records,
			std::string subFolder, int prevWrriten, int newWritten);

	void exportSimulation(std::string fileName,
			std::vector<ForwardInformation> &records,
			bool staticPrimitivesFirstFrameOnly = true);

	void exportCurrentSimulation(std::string fileName);

	void exportCurrentMeshPos(int frameIdx, std::string fileName);

	std::vector<BackwardInformation> runBackwardTask(
			BackwardTaskInformation taskConfiguration, LossType lossType,
			LossInfo &lossInfo, TaskSolveStatistics &taskStatistics,
			int FORWARD_STEPS, ParamInfo guess, bool lossOnly,
			const std::function<void(const std::string &value)> &setTextBoxCB =
					[](const std::string &v) {},
			bool skipForward = false);

	Vec3d calculatedri_dmu(const Vec3d &n, const Vec3d &f_i, double mu) const;

	Vec3d getInitParticlePos(int i, int j) const;

	double evaluateSystemEnergy(const VecXd &v, const VecXd &x) const;

	std::pair<Eigen::VectorXd, Eigen::VectorXd> getCurrentPosVelocityVec() const;

	void updateCurrentPosVelocityVec();

	std::pair<collisionInfoPair,
			std::vector<std::vector<SelfCollisionInformation>>>
	collisionDetection(const VecXd &x_n, const VecXd &v, const VecXd &x_prim,
			const VecXd &v_prim);

	std::vector<std::vector<Simulation::SelfCollisionInformation>>
	contactSorting(Simulation::collisionInfoPair &detections,
			std::map<int, std::set<int>> &selfCollisionMap,
			MatXi &selfCollisionTable);

	std::pair<VecXd, VecXd> calculateDryFrictionVector(
			const VecXd &f,
			std::pair<collisionInfoPair,
					std::vector<std::vector<SelfCollisionInformation>>>
					&detectionInfos);

	std::pair<std::pair<SpMat, SpMat>, VecXd>
	calculatedr_df(const completeCollisionInfo &infos,
			bool calculatePrimitiveGradient,
			bool calculateDensityGradient) const;

	VecXd calculatedr_ddensity(const completeCollisionInfo &detectionInfos,
			bool calculatePrimitiveGradient) const;

	std::vector<VecXd>
	calculatedr_dmu(const std::vector<PrimitiveCollisionInformation> &infos,
			const std::vector<int> &primIds) const;

	//    VecXd calculateDryFrictionAndVelocityVector(const VecXd &f, VecXd &v)
	//    const;
	Vec3d calcualteDryFrictionForce(const Vec3d &n, const Vec3d &f_i, double mu,
			CollisionType &type) const;

	Mat3x3d calculatedri_dfi(const Vec3d &n, const Vec3d &f_i, double mu) const;

	double evaluateEnergy(const VecXd &x_new) const;

	void resetSystem();

	void resetSystem(Vec3d windForce); // externalforce

	void resetParticlesAndPrimitivesToRestPose();

	void setParticlePosVelToVec(const std::pair<VecXd, VecXd> &x0v0);

	void resetSystem(const std::pair<VecXd, VecXd> &x0v0);

	void resetSystem(double k_stiff);

	void resetSystem(std::vector<Spline> &controlPoints);

	void resetSystem(std::vector<std::pair<int, double>> &primitiveMus) {
		resetSystem();
	}

	void initializePrefactoredMatrices();

private:
	const double fillForces(Eigen::VectorXd &f_int, Eigen::VectorXd &f_ext,
			const Eigen::VectorXd &velocityVec,
			const Eigen::VectorXd &posVec, double t_now);

	static SpMat factorizeDirectSolverLLT(const SpMat &A,
			Eigen::SimplicialLLT<SpMat> &lltSolver,
			const std::string &warning_msg);

	SpMat factorizeDirectSolverSparseLU(const SpMat &A,
			Eigen::SparseLU<SpMat> &lltSolver,
			const std::string &warning_msg);

	std::chrono::steady_clock::time_point getTimeNow() {
		return std::chrono::steady_clock::now();
	}

	// extra functions for python binding APIs
public:
	// get number of action DoFs
	// TODO: need to fix when there are multiple systems
	int getActionDim() { return sysMat[currentSysmatId].fixedPoints.size() * 3; }

	// get the forward info of the current step
	ForwardInformation getStateInfo() {
		return forwardRecords[forwardRecords.size() - 1];
	}

	// get the forward info of the current step
	ForwardInformation getPastStateInfo(int stepIdx) {
		return forwardRecords[stepIdx];
	}

	// set target position for the clips
	void setAction(VecXd action) {
		assert(action.size() == sysMat[currentSysmatId].fixedPoints.size() * 3);
		rlFixedPointPos = action;
	}

	// get number of particles
	int getNumParticles() { return particles.size(); }

	static std::string boolToStringTrueOnly(std::string labelTrue, bool v) {
		if (v)
			return labelTrue + "\n";
		return "";
	}

	static std::string boolToString(std::string labelTrue, std::string labelFalse,
			bool v) {
		if (v)
			return labelTrue;
		return labelFalse;
	}

	// set printVerbose
	void setPrintVerbose(bool flag) { printVerbose = flag; }

	LossInfo taskLossInfo;

	bool printVerbose;

	VecXd solveDirect(VecXd &dL_dxnew, double t_2, SpMat &dproj_dxnew_t,
			SystemMatrix &currentSysMat, SpMat &dr_df_plusI_t,
			SpMat &dr_df_t);
};

#endif // OMEGAENGINE_SIMULATION_H
