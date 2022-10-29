//
// Created by Yifei Li on 3/4/21.
// Email: liyifei@csail.mit.edu
//

#ifndef OMEGAENGINE_RENDERER_H
#define OMEGAENGINE_RENDERER_H

#include "Macros.h"
#include "Shader.h"
#include "Camera.h"
#include "VertexArrayObject.h"
#include "../simulation/Simulation.h"
#include "../optimization/OptimizationTaskConfigurations.h"
#include "Viewer.h"

class Renderer {
    unsigned static int gridVAO, gridVBO, gridIBO, fixedPointMeshVAO;
    static int gridLength;
    static bool gridInitialized, fixedPointInitialized;
public:


    static inline void fillArrayWithFloat(float *arr, float f, int startIdx) {
      arr[startIdx] = f;
    }

    static void fillArrayWithVec3(float *arr, Vec3d v, int startIdx) {
      for (int i = 0; i < 3; i++) {
        arr[startIdx + i] = v[i];
      }
    }

    static void useClothShaderAndSetUniforms(Simulation *msSystem, Shader *clothShader, Viewer &window);

    static void useSimpleShaderAndSetUniforms(Simulation *msSystem, Shader *simpleShader, Viewer &window);

    static void
    fillTriangleToVertexArr(Triangle &m, float *vertexArr, int offset, int numFields, Vec3d color, Vec3d center,
                            bool useVertexNormal = true);

    static void vertexArrAddAtrribInfo(VertexArrayObject &vertexVAO, std::vector<int> &fieldComponents) {
      int count = 0;
      for (int i = 0; i < fieldComponents.size(); i++) {
        vertexVAO.addVertexAttrib(i, fieldComponents[i], count * sizeof(float));
        count += fieldComponents[i];
      }
    }

    static Vec3d dirVecToAngleDegrees(Vec3d x) {
      Vec3d ret;
      for (int i = 0; i < 3; i++) {
        ret[i] = std::acos(x[i]);
      }
      return ret;
    }


    static void
    renderMesh(Simulation *msSystem, Shader *clothShader, Viewer &window, Simulation::FileMesh &model, Vec3d color,
               std::vector<Vec3d> center, Vec3d initialDir = Vec3d(0, 0, 0), Vec3d pointDir = Vec3d(0, 0, 0),
               bool shading = false, bool lighting = false);

    static void renderPrimitive(Primitive& p, Shader* shader, Vec3d center);
    static void renderPrimitives(std::vector<Primitive*>& primitives, Shader* clothShader,  int recordIdx, Vec3d center);
    static void renderLines(Simulation *msSystem, Shader *simpleShader, Viewer &window, const std::vector<std::pair<Vec3d, Vec3d>> &lines);
    static void renderLines(Simulation *msSystem, Shader *simpleShader, Viewer &window, const std::vector<std::pair<Vec3d, Vec3d>> &lines, Vec3d color);
    static void  renderNormals(Simulation *msSystem, Shader *clothShader, Viewer &window, Simulation::FileMesh &model, std::vector<std::pair<Vec3d, Vec3d>> &pointsAndDirs, Vec3d initialDir);
    static void
    renderMesh(Simulation *msSystem, Shader *clothShader, Viewer &window, Simulation::FileMesh &model, Vec3d color,
               Vec3d center, Vec3d initialDir = Vec3d(0, 0, 0), Vec3d pointDir = Vec3d(0, 0, 0), bool shading = false,
               bool lighting = false) {
      std::vector<Vec3d> singleton = {center};
      renderMesh(msSystem, clothShader, window, model, color, singleton, initialDir, pointDir, shading, lighting);
    }

    static void renderFixedPoints(Simulation *msSystem, Shader *clothShader, Viewer &window, std::vector<Vec3d> &points,
                                  Vec3d color, bool renderClips = false, bool useHashColor = false);

    static void renderGrid(Simulation *msSystem, Shader *simpleShader, Viewer &window);

    static void renderSplines(Simulation *msSystem, Shader *clothShader, Viewer &window, std::vector<Spline> &splines,
                              Vec3d color);

    static void
    stepAndRenderSystem(Simulation *msSystem, Shader *clothShader, Shader *simpleShader, Viewer &viewer,
                        std::vector<Simulation::ForwardInformation> &recordToUse, int recordIdx,
                        bool isGroundTruthSystem = false);
};


#endif //OMEGAENGINE_RENDERER_H
