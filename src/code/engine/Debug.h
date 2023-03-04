//
// Created by Yifei Li on 11/23/20.
// Email: liyifei@csail.mit.edu
//

#ifndef OMEGAENGINE_DEBUG_H
#define OMEGAENGINE_DEBUG_H

#include "Macros.h"

inline void printVec(const Vec4d &v, std::string s) {
	std::printf("%s: %.6f %.6f %.6f\n", s.c_str(), v[0], v[1], v[2]);
}

inline void throwError(std::string message) {
	std::cerr << "[Error] " << message << std::endl;
	throw message;
}

#endif // OMEGAENGINE_DEBUG_H
