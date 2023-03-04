//
// Created by Yifei Li on 11/23/20.
// Email: liyifei@csail.mit.edu
//

#include "AttachmentSpring.h"

#include "../engine/Debug.h"

double AttachmentSpring::k_stiff =
		10000.0; // TODO: WARNING: change back to 10000

double AttachmentSpring::evaluateEnergy(const VecXd &x_new) {
	energyBuffer =
			(0.5 * k_stiff * (p1_vec3(x_new) - fixedPointPos()).squaredNorm());
	return energyBuffer;
}

Vec3d AttachmentSpring::stretchingForce(const VecXd &x_vec) const {
	return -k_stiff * (p1_vec3(x_vec) - fixedPointPos());
}

Vec3d AttachmentSpring::dforce_dk(const VecXd &x_vec) const {
	return -(p1_vec3(x_vec) - fixedPointPos());
}

Eigen::VectorXd AttachmentSpring::project(const VecXd &x_vec) const {
	Vec3d ret = fixedPointPos();
	return sqrtConstraintWeight * ret;
}

void AttachmentSpring::projectBackwardPrecompute(const VecXd &x_vec) {}

void AttachmentSpring::projectBackward(const VecXd &x_vec,
		TripleVector &triplets) {
	// nothing to do because it's zero
}

Mat3x3d AttachmentSpring::dp_dfixedPose() const {
	Mat3x3d I_three = Mat3x3d::Identity();
	return sqrtConstraintWeight * I_three;
}

// https://blog.mmacklin.com/2012/05/04/implicitsprings/
Mat3x3d AttachmentSpring::getStretchingHessian(const VecXd &x_vec) const {
	return Mat3x3d::Identity() * k_stiff;
}

void AttachmentSpring::addConstraint(std::vector<Triplet> &tri, int &c_idx,
		bool withWeight) {
	if (withWeight) {
		this->c_idx = c_idx;
	} else {
		this->c_weightless_idx = c_idx;
	}

	double weightUsed = withWeight ? sqrtConstraintWeight : 1;

	for (int dim = 0; dim < 3; ++dim) {
		tri.emplace_back(c_idx + dim, p1_idx * 3 + dim, weightUsed);
	}
	c_idx += constraintNum;
}
