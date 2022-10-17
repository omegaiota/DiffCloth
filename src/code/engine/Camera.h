//
// Created by liyifei@csail.mit.edu on 8/6/2018.
//

#ifndef OMEGAENGINE_CAMERA_H
#define OMEGAENGINE_CAMERA_H
#include "Macros.h"

using namespace glm;
class Camera {
public:
    glm::vec3 lookAtPos;

    Camera() : lookAtPos(0, 0, 0), camPos(glm::vec3(-22.14, 9.24, 7.59)), speed(0.05f), camFront(glm::vec3(0, 0, -1)),
               camUp(glm::vec3(0, 1, 0)) {
      setPitchAndYaw(-33, 5.5);
//glm::vec3(-5, 45, 60)
//glm::vec3(-59, 25, -45)
//      setPitchAndYaw(-13, -315.5);
      updateViewMat();
    }

    Camera(glm::vec3 pos) {
      Camera();
      setCamPos(pos);
    }

    Camera(glm::vec3 pos, glm::vec3 dir) {
      Camera();
      setCamPosAndTarget(pos, dir);
    }

    void translateCamPos(glm::vec3 pos) {
      setCamPos(camPos + pos);

    }

    void setSpeed(float newSpeed) {
      speed = newSpeed;
    }

    void setCamPosAndTarget(glm::vec3 pos, glm::vec3 dir) {
      camPos = pos;
      camFront = dir;
      updateViewMat();
    }

    void setCamPos(glm::vec3 pos) {
      camPos = pos;

      updateViewMat();
    }


    void setPointAt(glm::vec3 target) {
      camFront = normalize(target - camPos);
      updateViewMat();
    }

    void setDir(glm::vec3 dir) {
      camFront = normalize(dir);
      updateViewMat();
    }

    void moveZ(float time) {
      translateCamPos(time * glm::vec3(0, 0, 1));
    }

    void moveNegZ(float time) {
      moveZ(-1 * time);
    }

    void moveLookAtDir(float time) {
      glm::vec3 lookArDir = normalize(-1.0f * camPos);
      translateCamPos(time * 0.1f * lookArDir);


    }

    void moveX(float time) {
      translateCamPos(time * glm::vec3(1, 0, 0));
    }

    void zoom(float percentage) {
      glm::vec3 newPos = lookAtPos * (percentage / 100.0f) + camPos * (1.0f - percentage / 100.0f);
      setCamPos(newPos);
    }


    void moveNegX(float time) {
      moveX(-1 * time);
    }

    void moveY(float time) {
      translateCamPos(time * glm::vec3(0, 1, 0));
    }

    void moveNegY(float time) {
      moveY(-1 * time);
    }

    void setFoVDeg(float deg) {
      FOVdegree = deg;
    }

    void updateDir();

    void setPitchAndYaw(float p, float y) {
      pitch = p;
      yaw = y;
    if (pitch > 89.0f)
      pitch = 89.0f;
    if (pitch< - 89.0f)
      pitch = -89.0f;

      updateDir();
    }

    void addPitchAndYaw(float p, float y) {
      setPitchAndYaw(pitch + p, yaw + y);
    }

    glm::vec3 getCamRight() {
      return normalize(cross(camFront, camUp));
    }

    void setLookAt(glm::vec3 r) {
      lookAtPos = r;
      updateViewMat();
    }

    glm::vec3 getCamUp() {
      return camUp;
    }

    glm::vec3 getCamFront() {
      return camFront;
    }


	mat4 getViewMat() {
    return viewMat;
  }

    glm::vec3 getCamPos() {
      return camPos;
    }

    void printState();

private:
    glm::vec3 camPos;
    glm::vec3 camFront;
    glm::vec3 camUp;

    mat4 viewMat;

    float FOVdegree;
    float speed;
    float pitch;
    float yaw;

    void updateViewMat() {
      viewMat = lookAt(camPos, lookAtPos, camUp);
    }
};


#endif //OMEGAENGINE_CAMERA_H
