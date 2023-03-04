//
// Created by Yifei Li on 9/24/20.
//

#include "Spring.h"
#include "../engine/UtilityFunctions.h"
#include <algorithm>

double Spring::k_stiff = 1.0;

double Spring::evaluateEnergy(const VecXd &x_new) {
	Vec3d p1_n = x_new.segment(p1()->idx * 3, 3);
	Vec3d p2_n = x_new.segment(p2()->idx * 3, 3);
	Vec3d dX = (p1_n - p2_n);
	double dL = dX.norm() - l0;

	// std::printf("spring with %d %d deformation energy: %.4f dl: %.4f restl:
	// %.4f\n", p1()->idx, p2()->idx,0.5 * k_s * dL * dL, dL, l0);
	energyBuffer = (0.5 * k_s * dL * dL);
	return energyBuffer;
}

Vec6d Spring::dforce_dk(const VecXd &x_vec) const {
	Vec3d dX = (p2_vec3(x_vec) - p1_vec3(x_vec));

	double dL = dX.norm() - l0;
	Vec3d dir = dX.normalized();
	Vec3d dfs_dk = dir * dL;

	Vec6d ret;
	ret.segment(0, 3) = dfs_dk;
	ret.segment(3, 3) = -dfs_dk;
	return ret;
}

Vec6d Spring::getForce(const VecXd &x_vec) const {
	Vec3d dX = (p2_vec3(x_vec) - p1_vec3(x_vec));

	double dL = dX.norm() - l0;

	Vec3d dir = dX.normalized();
	Vec3d f_s = k_s * dir * dL;
	//  Vec3d f_d = k_d * dX.dot(p2v - p1v) * dir;
	//  Vec3d f = f_s + f_d;

	Vec3d f = f_s;
	Vec6d ret;
	ret.segment(0, 3) = f;
	ret.segment(3, 3) = -f;
	return ret;
}

// https://blog.mmacklin.com/2012/05/04/implicitsprings/
Mat3x3d Spring::getStretchingHessian(const VecXd &x_vec) const {
	Vec3d pos_diff = (p2_vec3(x_vec) - p1_vec3(x_vec));
	double l = pos_diff.norm();
	Vec3d dir = pos_diff.normalized();

	Mat3x3d I = Mat3x3d::Identity();
	Mat3x3d A = pos_diff * pos_diff.transpose() / (l * l);

	//  m_H = ks * (I - l0 / l_ij* (I - (x_ij*x_ij.transpose()) / (l*l)));

	Mat3x3d H = k_s * (-I + (l0 / l) * (I - A));

	Eigen::EigenSolver<Mat3x3d> evd;
	evd.compute(H);
	Mat3x3d Q = evd.eigenvectors().real();
	Vec3d LAMBDA = evd.eigenvalues().real();
	// assert(LAMBDA(0) > 0);
	// ScalarType smallest_lambda = LAMBDA(0) * 1e-10;
	double smallest_lambda = 1e-6;
	for (unsigned int i = 0; i != LAMBDA.size(); i++) {
		// assert(LAMBDA(0) > LAMBDA(i));
		if (LAMBDA(i) < smallest_lambda) {
			LAMBDA(i) = smallest_lambda;
		}
	}
	H = Q * LAMBDA.asDiagonal() * Q.transpose();
	return H;
}

Eigen::VectorXd Spring::project(const VecXd &x_vec) const {
	Vec3d pos_diff = (p1_vec3(x_vec) - p2_vec3(x_vec));
	Vec3d dir = pos_diff.normalized();
	Vec3d p = dir * l0;
	return sqrtConstraintWeight * p;
}

void Spring::projectBackwardPrecompute(const VecXd &x_vec) {
	Mat3x3d I_three = Mat3x3d::Identity();
	Vec3d pos_diff = (p1_vec3(x_vec) - p2_vec3(x_vec));
	Mat3x3d d_posdiff_dx1 = I_three;
	Mat3x3d d_posdiff_dx2 = -I_three;
	Vec3d dir = pos_diff.normalized();
	Vec3d newPos1, newPos2;
	double l = pos_diff.norm();

	// TODO: no stiffness, is this right?
	Mat3x3d ddir_dposdiff = (I_three - dir * dir.transpose()) / l;
	dp_dx1 = l0 * ddir_dposdiff * d_posdiff_dx1;
	dp_dx2 = l0 * ddir_dposdiff * d_posdiff_dx2;
}

void Spring::projectBackward(const VecXd &x_vec, TripleVector &triplets) {
	insertIntoTriplets(triplets, dp_dx1, 3, 3, 0, 0, c_idx, p1()->idx * 3);
	insertIntoTriplets(triplets, dp_dx2, 3, 3, 0, 0, c_idx, p2()->idx * 3);

	//  dproj_dxnew.block<3,3>(c_idx,p1()->idx * 3) += sqrtConstraintWeight *
	//  dp_dx1; dproj_dxnew.block<3,3>(c_idx,p2()->idx * 3) +=
	//  sqrtConstraintWeight *  dp_dx2;
}

void Spring::addConstraint(std::vector<Triplet> &tri, int &c_idx,
		bool withWeight) {
	if (!withWeight) {
		std::printf("This is a spring constraint, which is deprecated in favor to "
					"triangle constraint. Should not be included in the backward "
					"gradient calculation regarding to stiffness parameter\n");
		exit(0);
	}
	this->c_idx = c_idx;
	double weightUsed = withWeight ? sqrtConstraintWeight : 1;

	for (int dim = 0; dim < 3; ++dim) {
		tri.emplace_back(c_idx + dim, p1()->idx * 3 + dim, weightUsed);
		tri.emplace_back(c_idx + dim, p2()->idx * 3 + dim, -weightUsed);
	}
	c_idx += constraintNum;
}
