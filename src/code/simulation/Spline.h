//
// Created by Yifei Li on 12/18/20.
// Email: liyifei@csail.mit.edu
//

#ifndef OMEGAENGINE_SPLINE_H
#define OMEGAENGINE_SPLINE_H

#include "../engine/Macros.h"

class Spline {
public:
	struct Segment {
		Vec3d p0;
		Vec3d m0;
		Vec3d p1;
		Vec3d m1;
		double yUp = 8;
		double startFraction; // fraction in terms of total simulation
		double endFraction;
		int segId;
	};
	std::vector<Segment> segments;
	int pFixed;

	enum SplineType { ENDPOINT,
		ENDPOINT_AND_UP,
		ENDPOINT_AND_TANGENTS };

	SplineType type = ENDPOINT;

	Spline(Vec3d p0, Vec3d p1, double yUp, int pFixed, double startFraction = 0.0,
			double endFraction = 1.0) :
			pFixed(pFixed) {
		Segment seg;
		seg.p0 = p0;
		seg.p1 = p1;
		seg.yUp = yUp;

		type = ENDPOINT;
		Vec3d dir = p1 - p0;
		seg.segId = 0;
		seg.m0 = p1 - p0;
		seg.m0[1] += yUp;
		seg.m1 = p1 - p0;
		seg.m1[1] -= yUp;
		seg.startFraction = startFraction;
		seg.endFraction = endFraction;

		segments.emplace_back(seg);
	}

	bool isWithinBound(AABB sceneBbox) {
		std::pair<VecXd, VecXd> bounds = getParameterBounds(sceneBbox);
		for (int i = 0; i < 8; i++) {
			Vec3d p = evalute(i * 1.0 / 8.0);
			for (int dim = 0; dim < 3; dim++) {
				if (p[dim] > sceneBbox.max[dim])
					return false;
				if (p[dim] < sceneBbox.min[dim])
					return false;
			}
		}
		return true;
	}

	static bool isClose(Spline &a, Spline &b, double threshold) {
		for (int i = 0; i < 5; i++) {
			Vec3d pA = a.evalute(i * 0.2);
			Vec3d pB = b.evalute(i * 0.2);
			if ((pA - pB).norm() > threshold)
				return false;
		}

		return true;
	}

	std::pair<VecXd, VecXd> getParameterBounds(AABB sceneBbox) {
		int paramNum = getParameterNumber();
		int singleSplineParamNum = getParameterNumber(type);
		VecXd boundsMin(paramNum), boundsMax(paramNum);
		boundsMin.setZero();
		boundsMax.setZero();

		for (int i = 0; i < segments.size(); i++) {
			int offset = singleSplineParamNum * i;
			boundsMin.segment(0 + offset, 3) = sceneBbox.min;
			boundsMax.segment(0 + offset, 3) = sceneBbox.max;
			switch (type) {
				case Spline::SplineType::ENDPOINT: {
					break;
				}

				case Spline::SplineType::ENDPOINT_AND_UP: {
					boundsMin[3 + offset] = sceneBbox.min[1];
					boundsMax[3 + offset] = sceneBbox.max[1];
					break;
				}

				case Spline::SplineType::ENDPOINT_AND_TANGENTS: {
					boundsMin.segment(3 + offset, 3) = Vec3d::Ones() * -50;
					boundsMax.segment(3 + offset, 3) = Vec3d::Ones() * 50;

					boundsMin.segment(6 + offset, 3) = Vec3d::Ones() * -50;
					boundsMax.segment(6 + offset, 3) = Vec3d::Ones() * 50;
					break;
				}
			}
		}
		return std::make_pair(boundsMin, boundsMax);
	}

	void addSegment(Vec3d p1, int yUp, double startFraction, double endFraction) {
		if (segments.empty()) {
			std::printf("WARNING: calling addSegment but spline is empty at the "
						"moment, please init spline with a segment\n");
			return;
		}
		Segment seg;
		seg.p0 = segments[segments.size() - 1].p1;
		seg.p1 = p1;
		seg.yUp = yUp;
		seg.segId = segments.size();
		seg.m0 = seg.m1 = p1 - seg.p0;
		seg.m0[1] += seg.yUp;
		seg.m1[1] -= seg.yUp;
		seg.startFraction = startFraction;
		seg.endFraction = endFraction;

		segments.emplace_back(seg);
	}

	static int getParameterNumberBeforeIdx(const std::vector<Spline> &splines,
			int idx) {
		int total = 0;
		for (int i = 0; i < idx; i++) {
			const Spline &s = splines[i];
			total += s.getParameterNumber();
		}
		return total;
	}

	static int getParameterNumber(const std::vector<Spline> &splines) {
		int total = 0;
		for (const Spline &s : splines) {
			total += s.getParameterNumber();
		}
		return total;
	}

	Segment getSegment(double t_simulationFraction) {
		for (Segment &s : segments) {
			if (s.endFraction >= t_simulationFraction)
				return s;
		}

		if (segments.empty())
			return Segment();
		return segments[segments.size() - 1];
	}

	double t_simPercentTot_SplinePercent(const Segment &seg, double t_sim) {
		// t_sim is the percentage of the simulation. Say there are 400*1/60 time
		// steps in total, then timestep at 200*1/60 is t_sim = 0.5

		// t_spline percentage is the linear interp between startFraction and
		// endFraction
		if (t_sim > seg.endFraction)
			return 1;
		if (t_sim < seg.startFraction)
			return 0;

		return (t_sim - seg.startFraction) / (seg.endFraction - seg.startFraction);
	}

	int getParameterNumber() const {
		return getParameterNumber(type) * segments.size();
	};

	static int getParameterNumber(SplineType type) {
		switch (type) {
			case ENDPOINT:
				return 3;

			case ENDPOINT_AND_UP:
				return 4;

			case ENDPOINT_AND_TANGENTS:
				return 9;
		}

		return 0;
	};

	static double p0Constant(
			double t,
			int order = 0) { // t should be the actual spline percentage, so t=0 will
							 // map to splineStart and t=1 will map to splineEnd
		double t_3 = t * t * t;
		double t_2 = t * t;

		if (order == 1)
			return (6 * t_2 - 6 * t);

		//    if (order == 0)
		return (2 * t_3 - 3 * t_2 + 1);
	}

	static double p1Constant(double t, int order = 0) {
		double t_3 = t * t * t;
		double t_2 = t * t;

		if (order == 1)
			return (-6 * t_2 + 6 * t);
		//      if (order == 0)
		return (-2 * t_3 + 3 * t_2);
	}

	static double m0Constant(double t, int order = 0) {
		double t_3 = t * t * t;
		double t_2 = t * t;

		if (order == 1)
			return (3 * t_2 - 4 * t + 1);

		//      if (order == 0)
		return (t_3 - 2 * t_2 + t);
	}

	static double m1Constant(double t, int order = 0) {
		double t_3 = t * t * t;
		double t_2 = t * t;

		if (order == 1)
			return (3 * t_2 - 2 * t);

		//      if (order == 0)
		return (t_3 - t_2);
	}

	VecXd paramToVector() {
		int paramNum = getParameterNumber();
		int singleSplineParamNum = getParameterNumber(type);
		VecXd retAll(paramNum);
		VecXd retSingle(singleSplineParamNum);

		for (int i = 0; i < segments.size(); i++) {
			retSingle.setZero();
			Segment &seg = segments[i];
			int offset = i * singleSplineParamNum;
			switch (type) {
				case ENDPOINT: {
					retSingle.segment(0, 3) = seg.p1; // p1
					break;
				}

				case ENDPOINT_AND_UP: {
					retSingle.segment(0, 3) = seg.p1;
					retSingle[3] = seg.yUp;
					break;
				}

				case ENDPOINT_AND_TANGENTS: {
					retSingle.segment(0, 3) = seg.p1;
					retSingle.segment(3, 3) = seg.m0;
					retSingle.segment(6, 3) = seg.m1;
					break;
				}
			}

			retAll.segment(offset, singleSplineParamNum) = retSingle;
		}

		return retAll;
	}

	MatXd dxfixed_dcontrolPoints(double t) { // deval(x)_dcontrolPoints
		// m0,p1,m1
		int paramNum = getParameterNumber();
		int singleSplineParamNum = getParameterNumber(type);
		MatXd retAll(3, paramNum);
		MatXd retSingle(3, singleSplineParamNum);
		retAll.setZero();
		retSingle.setZero();

		Segment seg = getSegment(t);
		t = t_simPercentTot_SplinePercent(seg, t);

		switch (type) {
			case ENDPOINT: {
				retSingle = (p1Constant(t) + m0Constant(t) + m1Constant(t)) *
						Mat3x3d::Identity(); // p1
				break;
			}

			case ENDPOINT_AND_UP: {
				retSingle.block<3, 3>(0, 0) =
						(p1Constant(t) + m0Constant(t) + m1Constant(t)) *
						Mat3x3d::Identity(); // p1
				// (idx for axis y is 1 , idx for offset for yUp is 3)
				retSingle(1, 3) = m0Constant(t) - m1Constant(t);
				break;
			}

			case ENDPOINT_AND_TANGENTS: {
				retSingle.block<3, 3>(0, 0) = p1Constant(t) * Mat3x3d::Identity();
				retSingle.block<3, 3>(0, 3) = m0Constant(t) * Mat3x3d::Identity();
				retSingle.block<3, 3>(0, 6) = m1Constant(t) * Mat3x3d::Identity();
				break;
			}
		}

		retAll.block(0, singleSplineParamNum * seg.segId, 3, singleSplineParamNum) =
				retSingle;
		return retAll;
	}

	Vec3d evalute(double t, int order = 0) {
		t = std::clamp(t, 0.0, 1.0);
		Segment seg = getSegment(t);
		t = t_simPercentTot_SplinePercent(seg, t);

		return (p0Constant(t, order) * seg.p0 + m0Constant(t, order) * seg.m0 +
				p1Constant(t, order) * seg.p1 + m1Constant(t, order) * seg.m1);
	}

	static Spline splineFromParam(Spline original, VecXd param) {
		Spline ret = original;
		int singleParam = getParameterNumber(original.type);
		if (param.rows() != original.getParameterNumber()) {
			std::printf("WARNING: Updating control points for spline type %d but "
						"dimension is %zu instead of %d\n",
					original.type, param.rows(), original.getParameterNumber());
		}
		for (int i = 0; i < ret.segments.size(); i++) {
			int paramOffset = singleParam * i;
			Segment &seg = ret.segments[i];
			switch (ret.type) {
				case ENDPOINT: {
					seg.p1 = param.segment(paramOffset, 3);
					seg.m0 = seg.p1 - seg.p0;
					seg.m0[1] += seg.yUp;
					seg.m1 = seg.p1 - seg.p0;
					seg.m1[1] -= seg.yUp;
					break;
				}

				case ENDPOINT_AND_UP: {
					seg.p1 = param.segment(paramOffset, 3);
					seg.yUp = param[3];

					seg.m0 = seg.p1 - seg.p0;
					seg.m0[1] += seg.yUp;
					seg.m1 = seg.p1 - seg.p0;
					seg.m1[1] -= seg.yUp;
					break;
				}

				case ENDPOINT_AND_TANGENTS: {
					seg.p1 = param.segment(0 + paramOffset, 3);
					seg.m0 = param.segment(3 + paramOffset, 3);
					seg.m1 = param.segment(6 + paramOffset, 3);
					break;
				}

				default: {
					std::printf("not implemented!\n");
				}
			}
		}

		return ret;
	}

	void updateControlPoints(const VecXd &step) { // (p1, m0, m1)
		int singleParam = getParameterNumber(type);
		if (step.rows() != getParameterNumber()) {
			std::printf("WARNING: Updating control points for spline type %d but "
						"dimension is %zu instead of %d\n",
					type, step.rows(), getParameterNumber());
		}
		for (int i = 0; i < segments.size(); i++) {
			int paramOffset = singleParam * i;
			Segment &seg = segments[i];
			switch (type) {
				case ENDPOINT: {
					seg.p1 += step.segment(paramOffset, 3);
					seg.m0 = seg.p1 - seg.p0;
					seg.m0[1] += seg.yUp;
					seg.m1 = seg.p1 - seg.p0;
					seg.m1[1] -= seg.yUp;
					break;
				}

				case ENDPOINT_AND_UP: {
					seg.p1 += step.segment(paramOffset, 3);
					seg.yUp += step[3];
					seg.m0 = seg.p1 - seg.p0;
					seg.m0[1] += seg.yUp;
					seg.m1 = seg.p1 - seg.p0;
					seg.m1[1] -= seg.yUp;
					break;
				}

				case ENDPOINT_AND_TANGENTS: {
					seg.p1 += step.segment(0 + paramOffset, 3);
					seg.m0 += step.segment(3 + paramOffset, 3);
					seg.m1 += step.segment(6 + paramOffset, 3);
					break;
				}

				default: {
					std::printf("not implemented!\n");
				}
			}
		}
	}

	void moveEndPoint(int segId, Vec3d newp1) {
		Segment &seg = segments[segId];
		seg.p1 = newp1;
		seg.m0 = seg.p1 - seg.p0;
		seg.m0[1] += seg.yUp;
		seg.m1 = seg.p1 - seg.p0;
		seg.m1[1] -= seg.yUp;

		if (segId + 1 < segments.size()) {
			seg = segments[segId + 1];
			seg.p0 = newp1;
			seg.m0 = seg.p1 - seg.p0;
			seg.m0[1] += seg.yUp;
			seg.m1 = seg.p1 - seg.p0;
			seg.m1[1] -= seg.yUp;
		}
	}
};

#endif // OMEGAENGINE_SPLINE_H