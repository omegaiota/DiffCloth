//
// Created by liyifei@csail.mit.edu on 7/19/18.
//

#ifndef OMEGAENGINE_MACROS_H
#define OMEGAENGINE_MACROS_H

#define GLM_ENABLE_EXPERIMENTAL
#define GLFW_INCLUDE_NONE
#include <chrono>
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <sstream>
#include <thread>

#include "../supports/Logging.h"
#include <iomanip>

// #include <GLFW/glfw3.h>
#include "Eigen/Core"
#include "stb_image.h"
#include <Eigen/Eigenvalues>

#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <algorithm>
#define ARRAY_COUNT(x) (sizeof(x) / sizeof((x)[0]))

#ifdef USE_DEBUG
#define DEBUG_PRINTF(x) printf x
#define AssertDebug(x) assert(x)
#else
#define DEBUG_PRINTF(x)
#define AssertDebug(x)
#endif

typedef unsigned int Index;
typedef Eigen::VectorXd VecXd;
typedef Eigen::VectorXi VecXi;
typedef Eigen::Vector2i Vec2i;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Vector2d Vec2d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector4d Vec4d;

typedef Eigen::Matrix<double, 5, 1> Vec5d;
typedef Eigen::Matrix<double, 6, 1> Vec6d;
typedef Eigen::Matrix<double, 9, 1> Vec9d;
typedef Eigen::Matrix<double, 12, 1> Vec12d;
typedef Eigen::Matrix<double, 12, 12> Mat12x12d;

typedef Eigen::Matrix<int, 2, 2> Mat2x2i;
typedef Eigen::Matrix<double, 1, 1> Mat1x1d;
typedef Eigen::Matrix<double, 2, 2> Mat2x2d;
typedef Eigen::Matrix<double, 2, 3> Mat2x3d;
typedef Eigen::Matrix<double, 3, 1> Mat3x1d;
typedef Eigen::Matrix<double, 3, 2> Mat3x2d;
typedef Eigen::Matrix<double, 3, 3> Mat3x3d;
typedef Eigen::Matrix<double, 3, 4> Mat3x4d;
typedef Eigen::Matrix<double, 3, 5> Mat3x5d;
typedef Eigen::Matrix<double, 3, 6> Mat3x6d;
typedef Eigen::Matrix<double, 3, 9> Mat3x9d;
typedef Eigen::Matrix<double, 3, 12> Mat3x12d;
typedef Eigen::Matrix<double, 4, 4> Mat4x4d;
typedef Eigen::Matrix<double, 4, 6> Mat4x6d;
typedef Eigen::Matrix<double, 4, 9> Mat4x9d;
typedef Eigen::Matrix<double, 5, 3> Mat5x3d;
typedef Eigen::Matrix<double, 6, 3> Mat6x3d;
typedef Eigen::Matrix<double, 6, 4> Mat6x4d;
typedef Eigen::Matrix<double, 6, 6> Mat6x6d;
typedef Eigen::Matrix<double, 6, 9> Mat6x9d;
typedef Eigen::Matrix<double, 9, 9> Mat9x9d;
typedef std::pair<std::string, long long> TimerEntry;
typedef std::vector<Eigen::Triplet<double>> TripleVector;
typedef Eigen::Transform<double, 3, Eigen::Affine> Rotation;

struct PerformanceTiming {
	long long nonSolveTimeMicroseconds;
	long long solveDirectMicroseconds;
	long long solveIterativeMicroseconds;
};

struct TimerContent {
	std::vector<TimerEntry> timeMicroseconds;
	long long totalMicroseconds;
	PerformanceTiming solvePerfReport;
};

struct AABB { // axis-aligned bounding box
	Vec3d min;
	Vec3d max;

	AABB() {
		min.setZero();
		max.setZero();
	}

	AABB(Vec3d min, Vec3d max) :
			min(min), max(max){};
};

template <int n>
struct Eig {
	Eigen::Matrix<double, n, n> Q;
	Eigen::Matrix<double, n, 1> l;
};
typedef Eigen::MatrixXd MatXd;
typedef Eigen::MatrixXi MatXi;
typedef Eigen::Triplet<double> Triplet;
typedef Eigen::SparseMatrix<double> SpMat;

struct DrawSurfObject {
	float *vertices;
	float *indices;
	int vbSize;
	int idSize;
};

const int OPENMP_ENABLED = true;
#include <omp.h>

#endif // OMEGAENGINE_MACROS_H
