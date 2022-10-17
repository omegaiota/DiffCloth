//
// Created by liyifei@csail.mit.edu on 8/6/2018.
//

#include "Camera.h"

void Camera::updateDir()
{
	vec3 direction;
	direction.x = cos(radians(pitch)) * cos(radians(yaw));
	direction.y = sin(radians(pitch));
	direction.z = cos(radians(pitch)) * sin(radians(yaw));
	camFront = normalize(direction);
//#ifdef DEBUG_PRINTF
  DEBUG_PRINTF( ("pitch:%.2f yaw:%.2f\n", pitch, yaw));
//#endif
	updateViewMat();
}

void Camera::printState() {

}
