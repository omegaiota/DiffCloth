//
// Created by Yifei Li on 3/17/21.
// Email: liyifei@csail.mit.edu
//

#include "TriangleBending.h"

double TriangleBending::k_stiff = 0.0;
bool useEnergy1 = false;
void TriangleBending::addConstraint(std::vector<Triplet> &tri, int &c_idx,
		bool withWeight) {
	if (withWeight) {
		this->c_idx = c_idx;
	} else {
		this->c_weightless_idx = c_idx;
	}

	double weightUsed =
			withWeight ? constrainWeightSqrt : std::sqrt(3.0 / (A0 + A1));

	for (int dim = 0; dim < 3; dim++) {
		for (int i = 0; i < 4; ++i) {
			tri.emplace_back(c_idx + dim, idx_arr[i] * 3 + dim,
					weightUsed * weightVert[i]);
		}
	}

	c_idx += constraintNum;
}

double TriangleBending::evaluateEnergy(const VecXd &x_new) {
	Vec3d proj = project(x_new);
	double E = 0;

	Vec3d w_times_x;
	w_times_x.setZero();
	for (int i = 0; i < 4; i++) {
		Vec3d x_i = x_new.segment(idx_arr[i] * 3, 3);
		w_times_x += weightVert[i] * x_i;
	}

	E = 0.5 * (constrainWeightSqrt * constrainWeightSqrt) *
			(w_times_x - proj).squaredNorm();

	energyBuffer = E;
	return energyBuffer;
}

Vec12d TriangleBending::bendingForce(const VecXd &x_new) {
	Vec12d gradE;
	gradE.setZero();

	Vec3d proj = project(x_new);

	double w_constraint = std::pow(constrainWeightSqrt, 2);
	Mat3x12d grad_proj = backwardGradient(x_new);

	/* gradEv1 <--- this is the same as gradEv2, but has redundant calculation
	 * because of the Envelope Theorem */
	/*
	 Vec3d sumwx;
	 sumwx.setZero();
	 for (int i = 0; i < 4; i++) {
	   Vec3d x_i = x_new.segment(idx_arr[i] * 3, 3);
	   double w_i = weightVert[i];
	   sumwx += w_i * x_i;
	 }


   for (int i = 0; i < 4; i++) {
	 Vec3d x_i = x_new.segment(idx_arr[i] * 3, 3);
	 double w_i = weightVert[i];
	 gradE.segment(3 * i, 3) += w_constraint * (sumwx - proj).transpose() * (w_i *
   Mat3x3d::Identity() - grad_proj.block<3, 3>(0, 3 * i) );
   }
   */

	Vec12d x;
	x.setZero();
	for (int i = 0; i < 4; i++) {
		Vec3d x_i = x_new.segment(idx_arr[i] * 3, 3);
		x.segment(i * 3, 3) = x_i;
	}

	Vec12d gradEv2;
	gradEv2.setZero();
	gradEv2 = w_constraint * A_w.transpose() * (A_w * x - proj);

	return -gradEv2;
}

Vec12d TriangleBending::dfi_dk(const VecXd &x_new) {
	dfi_dk_buffer =
			bendingForce(x_new) / (constrainWeightSqrt * constrainWeightSqrt);
	return dfi_dk_buffer;
}

Mat12x12d TriangleBending::projectSecondOrderJacobian(const VecXd &x_vec,
		const Vec3d &v) {
	Mat12x12d J;
	J.setZero();

	if (n <= 1e-6)
		return J;

	Vec3d e;
	e.setZero();
	const Mat3x3d I = Mat3x3d::Identity();
	Mat3x12d de;
	de.setZero();
	for (int i = 0; i < 4; i++) {
		e += weightVert[i] * x_vec.segment(3 * idx_arr[i], 3);
		de.middleCols(3 * i, 3) += weightVert[i] * I;
	}
	// de = [w0 * I, w1 * I, w2 * I, w3 * I] and is irrelevant to x_vec.
	const double e_norm = e.norm();
	const Vec3d e_normalized = e / e_norm;
	const Vec12d de_norm(e_normalized.transpose() * de); // d(e_norm)/dx.
	const Mat3x12d de_normalized =
			(I - e_normalized * e_normalized.transpose()) / e_norm * de;
	// This is = de / e_norm - e_normalized * e_normalized.T * de / e_norm.
	for (int i = 0; i < 12; ++i) {
		Mat3x12d dde_normalized_dxi;
		dde_normalized_dxi.setZero();
		// First, the gradient of de / e_norm.
		dde_normalized_dxi = -de * de_norm(i) / (e_norm * e_norm);
		// Second, the gradient of e_normalizd * e_normalizd.T * de / e_norm.
		dde_normalized_dxi -=
				(de_normalized.col(i) * e_normalized.transpose() / e_norm +
						e_normalized * de_normalized.col(i).transpose() / e_norm -
						e_normalized * e_normalized.transpose() * de_norm(i) /
								(e_norm * e_norm)) *
				de;
		J.col(i) = Vec12d(v.transpose() * dde_normalized_dxi);
	}
	return constrainWeightSqrt * J * n;
}

VecXd TriangleBending::project(const VecXd &x_vec) const {
	Vec3d e;
	e.setZero();
	double clampLow = 1.0;
	double clampHigh = 1.0;

	if (n > 1e-6) {
		for (int i = 0; i < 4; i++)
			e += weightVert[i] * x_vec.segment(3 * idx_arr[i], 3);

		e = e.normalized() * n;
	}
	return constrainWeightSqrt * e;
}

Mat3x12d TriangleBending::backwardGradient(const VecXd &x_vec) {
	// Tao: I am simplifying this function because Yifei Li confirms k == 1 in
	// project(x_vec).
	if (n <= 1e-6)
		return Mat3x12d::Zero();

	// Now the forward pass. Copied for our reference.
	Vec3d e;
	e.setZero();
	const Mat3x3d I = Mat3x3d::Identity();
	Mat3x12d de;
	de.setZero();
	for (int i = 0; i < 4; i++) {
		e += weightVert[i] * x_vec.segment(3 * idx_arr[i], 3);
		de.middleCols(3 * i, 3) += weightVert[i] * I;
	}
	const double e_norm = e.norm();
	const Vec3d e_normalized = e / e_norm;
	const Mat3x12d de_normalized =
			(I - e_normalized * e_normalized.transpose()) / e_norm * de;
	// ret = constrainWeightSqrt * e_normalized * n.
	return constrainWeightSqrt * de_normalized * n;
}

void TriangleBending::projectBackwardPrecompute(const VecXd &x_vec) {
	grad = backwardGradient(x_vec);
}

void TriangleBending::projectBackward(const VecXd &x_vec,
		TripleVector &triplets) {
	insertIntoTriplets(triplets, grad, 3, 3, 0, 0, c_idx, p0_idx * 3);
	insertIntoTriplets(triplets, grad, 3, 3, 0, 3, c_idx, p1_idx * 3);
	insertIntoTriplets(triplets, grad, 3, 3, 0, 6, c_idx, p2_idx * 3);
	insertIntoTriplets(triplets, grad, 3, 3, 0, 9, c_idx, p3_idx * 3);
}

TriangleBending::TriangleBending(int p0_idx, int p1_idx, int p2_idx, int p3_idx,
		std::vector<Particle> &pArr) :
		Constraint(Constraint::ConstraintType::CONSTRAINT_TRIANGLE_BENDING, 3),
		p0_idx(p0_idx),
		p1_idx(p1_idx),
		p2_idx(p2_idx),
		p3_idx(p3_idx),
		pArr(pArr) {
	idx_arr.clear();
	idx_arr.emplace_back(p0_idx);
	idx_arr.emplace_back(p1_idx);
	idx_arr.emplace_back(p2_idx);
	idx_arr.emplace_back(p3_idx);
	Mat3x4d pos(3, 4);
	pos.setZero();
	for (int i = 0; i < 4; i++)
		pos.col(i) = pArr[idx_arr[i]].pos_rest;

	double l01 = (pos.col(1) - pos.col(0)).norm();
	double l02 = (pos.col(2) - pos.col(0)).norm();
	double l03 = (pos.col(3) - pos.col(0)).norm();
	double l12 = (pos.col(1) - pos.col(2)).norm();
	double l13 = (pos.col(1) - pos.col(3)).norm();

	double r0 = 0.5 * (l01 + l02 + l12); // triangle 1
	A0 = std::sqrt(r0 * (r0 - l01) * (r0 - l02) * (r0 - l12));
	double r1 = 0.5 * (l01 + l13 + l03); // triangle 2
	A1 = std::sqrt(r1 * (r1 - l01) * (r1 - l03) * (r1 - l13));

	double cot02 = ((l01 * l01) - (l02 * l02) + (l12 * l12)) / (4.0 * A0);
	double cot12 = ((l01 * l01) + (l02 * l02) - (l12 * l12)) / (4.0 * A0);
	double cot03 = ((l01 * l01) - (l03 * l03) + (l13 * l13)) / (4.0 * A1);
	double cot13 = ((l01 * l01) + (l03 * l03) - (l13 * l13)) / (4.0 * A1);

	weightVert.setZero();
	weightVert(0) = cot02 + cot03;
	weightVert(1) = cot12 + cot13;
	weightVert(2) = -(cot02 + cot12);
	weightVert(3) = -(cot03 + cot13);
	n = (pos * weightVert).norm();

	A_w.setZero();
	for (int i = 0; i < 4; i++) {
		for (int dim = 0; dim < 3; dim++) {
			A_w(dim, i * 3 + dim) = weightVert[i];
		}
	}

	de_dxnew.setZero();
	for (int i = 0; i < 4; i++) {
		de_dxnew.block<3, 3>(0, i * 3) = Mat3x3d::Identity() * weightVert[i];
	}

	setConstraintWeight();
}
