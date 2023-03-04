//
// Created by Yifei Li on 5/1/21.
// Email: liyifei@csail.mit.edu
//

#ifndef OMEGAENGINE_FIXEDPOINT_H
#define OMEGAENGINE_FIXEDPOINT_H

#include "../engine/Macros.h"

class FixedPoint {
public:
	Vec3d pos;
	Vec3d pos_rest;
	int idx;
	FixedPoint() :
			pos(0, 0, 0), pos_rest(0, 0, 0), idx(0){};
	FixedPoint(Vec3d pos_rest, int idx) :
			pos(pos_rest), pos_rest(pos_rest), idx(idx){};
};

#endif // OMEGAENGINE_FIXEDPOINT_H
