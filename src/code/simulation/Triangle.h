//
// Created by Yifei Li on 9/28/20.
// Email: liyifei@csail.mit.edu
//

#ifndef OMEGAENGINE_TRIANGLE_H
#define OMEGAENGINE_TRIANGLE_H

#include "../engine/UtilityFunctions.h"
#include "Constraint.h"
#include "Particle.h"

class Triangle : public Constraint {
public:
	enum EnergyType { QUADRATIC,
		NON_QUADRATIC };

	Mat6x9d newF;
	int p0_idx, p1_idx, p2_idx;
	std::vector<int> idxArr;
	std::vector<Particle> &pArr;
	Vec3d color, normal;
	bool overrideColor;
	Vec9d stretchingForceBuffer, dfi_dk_buffer;
	static Eigen::Vector4d k;
	static double k_stiff;
	EnergyType energyType;
	/* reused param for J and H*/
	//    Mat3x2d F;
	Mat3x2d P; // material coordinates
	Mat2x2d I_two, rest;
	Mat3x3d I_three;
	Mat2x2d deltaUV, inv_deltaUV; // material space
	Mat9x9d hessian_buffer;
	double area_rest; // area, material space
	double constrainWeightSqrt; // material space
	Eigen::Matrix<double, 6, 9>
			dF_dx; // gradient of deformation gradient wrt position

	Triangle(int p0_idx, int p1_idx, int p2_idx, std::vector<Particle> &pArr,
			bool checkArea = false);

	Triangle &operator=(const Triangle &other);

	double evaluateEnergy(const VecXd &x_new) override;

	double evaluateEnergy(const Mat3x2d &F, const VecXd &x_new);

	void hessianPrecalc(const VecXd &x_new) {
		hessian_buffer = stretchingHessian(x_new);
	}

	Mat3x2d projectToManifold(const VecXd &x_vec) const;

	Vec3i getIdxVector() { return Vec3i(p0_idx, p1_idx, p2_idx); }

	Particle *p0() const { return &(pArr[p0_idx]); }

	Particle *p1() const { return &(pArr[p1_idx]); }

	Particle *p2() const { return &(pArr[p2_idx]); }

	Vec3d p0_vec3(const VecXd &x) const { return x.segment(p0()->idx * 3, 3); }

	Vec3d p1_vec3(const VecXd &x) const { return x.segment(p1()->idx * 3, 3); }

	Vec3d p2_vec3(const VecXd &x) const { return x.segment(p2()->idx * 3, 3); }

	double getDeformation(const VecXd &x) const {
		double areaNow = getArea(x);
		return areaNow / area_rest;
	}

	Vec9d dDeformation_dxnew(const VecXd &x) const {
		return dArea_dxnew(x) / area_rest;
	}

	Mat3x3d skew(const Vec3d &a) const { // a x b = skew(a) x b
		std::vector<double> skewElements = { 0, -a[2], a[1], a[2], 0,
			-a[0], -a[1], a[0], 0 };
		Mat3x3d out;
		out.setZero();
		for (int i = 0; i < 9; i++) {
			out(i / 3, i % 3) = skewElements[i];
		}

		return out;
	}

	Vec9d dArea_dxnew(const VecXd &x) const {
		int x0_col = 0, x1_col = 3, x2_col = 6;
		Vec3d x0 = p0_vec3(x);
		Vec3d x1 = p1_vec3(x);
		Vec3d x2 = p2_vec3(x);

		Vec3d v1 = x1 - x0;
		Vec3d v2 = x2 - x0;
		Vec3d crossProd = v1.cross(v2);

		Mat3x3d dv1_dx1 = Mat3x3d::Identity();
		Mat3x3d dv1_dx2 = Mat3x3d::Zero();

		Mat3x3d dv2_dx1 = Mat3x3d::Identity();
		Mat3x3d dv2_dx2 = Mat3x3d::Zero();

		Mat6x9d dv1v2_dx0x1x2;
		int v1_row = 0, v2_row = 3;
		dv1v2_dx0x1x2.block<3, 3>(v1_row, x0_col) = -Mat3x3d::Identity(); // dv1_dx0
		dv1v2_dx0x1x2.block<3, 3>(v2_row, x0_col) = -Mat3x3d::Identity(); // dv2_dx0

		dv1v2_dx0x1x2.block<3, 3>(v1_row, x1_col) = Mat3x3d::Identity(); // dv1_dx1
		dv1v2_dx0x1x2.block<3, 3>(v2_row, x1_col).setZero(); // dv2_dx1

		dv1v2_dx0x1x2.block<3, 3>(v1_row, x2_col).setZero(); // dv1_dx1
		dv1v2_dx0x1x2.block<3, 3>(v2_row, x2_col) = Mat3x3d::Identity(); // dv2_dx1

		Mat3x6d dv1xv2_v1v2;
		dv1xv2_v1v2.block<3, 3>(0, 0) = -skew(v2); // dv1xv2_dv1
		dv1xv2_v1v2.block<3, 3>(0, 3) = skew(v1); // dv1xv2_dv2

		Mat3x1d dout_dv1xv2 = 0.5 * crossProd.normalized();

		Vec9d dout_dx0x1x2 =
				(dout_dv1xv2.transpose() * dv1xv2_v1v2 * dv1v2_dx0x1x2).transpose();

		return dout_dx0x1x2;
	}

	double getArea(const VecXd &x) const {
		Vec3d pos0 = p0_vec3(x);
		Vec3d pos1 = p1_vec3(x);
		Vec3d pos2 = p2_vec3(x);

		Vec3d v1 = pos1 - pos0;
		Vec3d v2 = pos2 - pos0;

		return v1.cross(v2).norm() / 2.0;
	}

	static double getAngleABCInDegreeBetween(Vec3d a, Vec3d b, Vec3d c) {
		Vec3d AB = b - a;
		Vec3d BC = c - b;
		double theta = std::acos(AB.dot(BC) / (AB.norm() * BC.norm()));
		return theta;
	}

	void setConstraintWeight() override {
		constrainWeightSqrt = std::sqrt(area_rest * Triangle::k_stiff);
	};

	static Vec3d getNormal(const Vec3d &p0, const Vec3d &p1, const Vec3d &p2) {
		Vec3d v0 = (p1 - p0);
		Vec3d v1 = (p2 - p0);

		return v0.cross(v1).normalized();
	}

	Vec3d getNormal() const {
		Vec3d v0 = (p1()->pos - p0()->pos);
		Vec3d v1 = (p2()->pos - p0()->pos);

		return v0.cross(v1).normalized();
	}

	void addConstraint(std::vector<Triplet> &tri, int &c_idx,
			bool withWeight) override;

	VecXd project(const VecXd &x_vec) const override;

	void projectBackward(const VecXd &x_vec, TripleVector &triplets) override;

	void projectBackwardPrecompute(const VecXd &x_vec) override;

	VecXd dp_dk(const VecXd &x_vec) const override;

	Mat6x9d projectToManifoldBackward(const VecXd &x_vec) const;

	Mat3x2d getDeformationGradient(const VecXd &x_vec) const;

	Vec9d stretchingForce(const VecXd &x_new);

	Mat9x9d stretchingHessian(const VecXd &x_new) const;

	Vec9d dfi_dk(const VecXd &x_new);
	//    std::pair<Eigen::Matrix<double, 9, 9>, Vec9d>
	//    stretchingForceAndJacobian() const;

	std::pair<Mat6x9d, Mat3x2d> forwardBackwardCheck(const VecXd &x_vec) const;

#ifdef USE_DEBUG
	std::pair<std::vector<Vec9d>, Mat9x9d>
	stretchingForceAndJacobianCheck() const;
#endif

	Mat2x3d getDerivative() const;
};

#endif // OMEGAENGINE_TRIANGLE_H
