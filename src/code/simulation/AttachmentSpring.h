//
// Created by Yifei Li on 11/23/20.
// Email: liyifei@csail.mit.edu
//

#ifndef OMEGAENGINE_ATTACHMENTSPRING_H
#define OMEGAENGINE_ATTACHMENTSPRING_H

#include "../engine/Macros.h"
#include "Constraint.h"
#include "Eigen/Sparse"
#include "FixedPoint.h"
#include "Particle.h"

class AttachmentSpring : public Constraint {
public:
	double sqrtConstraintWeight;
	int p1_idx;
	int pfixed_idx; // fixed point
	std::vector<Particle> *pArr;
	std::vector<FixedPoint> *pFixedArr; // fixed point arr
	static double k_stiff; // spring stiffness

	AttachmentSpring(){};

	AttachmentSpring(int p1_idx, std::vector<Particle> *pArr, int pfixed_idx,
			std::vector<FixedPoint> *pFixedArr) :
			Constraint(Constraint::ConstraintType::CONSTRAINT_ATTACHMENT, 3),
			p1_idx(p1_idx),
			pfixed_idx(pfixed_idx),
			pArr(pArr),
			pFixedArr(pFixedArr),
			sqrtConstraintWeight(std::sqrt(AttachmentSpring::k_stiff)) {}

	AttachmentSpring(const AttachmentSpring &other) :
			p1_idx(other.p1_idx), pfixed_idx(other.pfixed_idx), sqrtConstraintWeight(other.sqrtConstraintWeight), pArr(other.pArr), pFixedArr(other.pFixedArr) {}

	Particle *p1() const {
		assert(((*pArr)[p1_idx]).idx == p1_idx);
		return &((*pArr)[p1_idx]);
	}

	FixedPoint *pFixed() const {
		assert(((*pFixedArr)[pfixed_idx]).idx == pfixed_idx);
		return &((*pFixedArr)[pfixed_idx]);
	}

	Vec3d fixedPointPos() const {
		//      std::printf("fixedPos: %.3f %.3f %.3f\n", pFixed()->pos[0],
		//      pFixed()->pos[1], pFixed()->pos[2] ); std::printf("p1: %.3f %.3f
		//      %.3f\n", p1()->pos[0], p1()->pos[1], p1()->pos[2] );
		return pFixed()->pos;
	}

	Vec3d p1_vec3(const VecXd &x) const { return x.segment(p1()->idx * 3, 3); }

	Vec3d stretchingForce(const VecXd &x_vec) const;

	Mat3x3d getStretchingHessian(const VecXd &x_vec) const;

	Vec3d dforce_dk(const VecXd &x_vec) const;

	Eigen::VectorXd project(const VecXd &x_vec) const override;

	void projectBackward(const VecXd &x_vec, TripleVector &triplets) override;
	void projectBackwardPrecompute(const VecXd &x_vec) override;
	Mat3x3d dp_dfixedPose() const;

	double evaluateEnergy(const VecXd &x_new) override;

	void setConstraintWeight() override {
		sqrtConstraintWeight = std::sqrt(AttachmentSpring::k_stiff);
	}

	void addConstraint(std::vector<Triplet> &tri, int &c_idx,
			bool withWeight) override;
};

#endif // OMEGAENGINE_ATTACHMENTSPRING_H
