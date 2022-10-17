//
// Created by Yifei Li on 9/24/20.
//

#include "Simulation.h"

//#define DEBUG_EXPLOSION
//#define  DEBUG_SELFCOLLISION
volatile bool Simulation::gravityEnabled = true;
volatile bool Simulation::bendingEnabled = true;
volatile bool Simulation::staticEnabled = false;
volatile bool Simulation::contactEnabled = true;
volatile bool Simulation::selfcollisionEnabled = true;
volatile bool Simulation::enableConstantForcefield = false;

volatile bool Simulation::windEnabled = false;
double Simulation::forwardConvergenceThreshold = 1e-7;
int Simulation::PD_TOTAL_ITER = 1000;
double Simulation::backwardConvergenceThreshold = 1e-4 * 0.5;
double Simulation::windNorm = 0.15;
double Simulation::windFrequency = 14;
double Simulation::windPhase = 0;


double *Simulation::k_stiff_arr[Constraint::CONSTRAINT_NUM] = {&Spring::k_stiff, &AttachmentSpring::k_stiff,
                                                               &Triangle::k_stiff, &TriangleBending::k_stiff};

Simulation::BackwardTaskInformation Simulation::taskConfigDefault = {.dL_dk_pertype = {false, false, false, false},
        .dL_density = false, .dL_dfext = false, .dL_dfwind = false, .adddr_dd = false,
        .dL_dcontrolPoints = false, .dL_dmu = false,
        .mu_primitives = std::vector<int>(0, 0)};


double Simulation::calculateTriangleDeformation(VecXd &x_new) {
  double areaTotal = 0.0;
  double areaRestTotal = 0.0;
  for (Triangle &t : mesh) {
    areaTotal += t.getArea(x_new);
    areaRestTotal += t.area_rest;
  }

  return areaTotal / areaRestTotal;
}

double Simulation::calculateMaxTriangleDeformation(VecXd& x_new) {
  double maxDeformation = 0.0;
  for (Triangle& t : mesh) {
    double deformation = t.getArea(x_new) / t.area_rest;
    maxDeformation = std::max(maxDeformation, deformation);
  }

  return maxDeformation;
}

const double Simulation::fillForces(VecXd &f_int, VecXd &f_ext, const VecXd &v,
                                    const VecXd &x, double t_now) {
  // calculate forces
  f_int.setZero();
  // Internal forces calculation not needed for PD
  f_ext.setZero();
  // externalForces
  double windFactor = 1.0;


  switch (sceneConfig.windConfig) {
    case WindConfig::WIND_SIN_AND_FALLOFF:
    case WindConfig::WIND_SIN: {
      windFactor = (std::sin(windFrequency * t_now + windPhase) + 1.0) / 2.0;
      break;
    }
    case WindConfig::NO_WIND: {
      windFactor = 0.0;
      break;
    }
    case WindConfig::WIND_CONSTANT: {
      windFactor = 1.0;
      break;
    }

    case WindConfig::WIND_FACTOR_PER_STEP: {
      windFactor = perstepWindFactor[forwardRecords.size()];
      break;
    }

  }

  if (enableConstantForcefield) {
    f_ext += external_force_field;
  }

  //  double windFactor = 1.0;
  for (int i = 0; i < particles.size(); i++) {
    Particle &p = particles[i];
    Vec3d f_i(0, 0, 0);
    if (gravityEnabled)
      f_i += gravity * p.mass;

    if (windEnabled) {
      Vec3d windf_i = (wind * windNorm * windFactor);
      if (sceneConfig.windConfig == WindConfig::WIND_SIN_AND_FALLOFF ||
          (sceneConfig.windConfig == WindConfig::WIND_FACTOR_PER_STEP)) {
        windf_i = windf_i.cwiseProduct(windFallOff.segment(i * 3, 3));
      }
      f_i += windf_i;
    }


    f_ext.segment(p.idx * 3, 3) += f_i;
  }



  return windFactor;


}

void Simulation::updateCurrentPosVelocityVec() {
  v_n.setZero();
  x_n.setZero();

  for (const Particle &p : particles) {
    v_n.segment(p.idx * 3, 3) = p.velocity;
    x_n.segment(p.idx * 3, 3) = p.pos;
  }

}

std::pair<VecXd, VecXd> Simulation::getCurrentPosVelocityVec() const {
  VecXd velocity(particles.size() * 3);
  VecXd pos(particles.size() * 3);
  velocity.setZero();
  pos.setZero();

  bool encounteredNAN = false;
  int nanCount = 0;
  for (const Particle &p : particles) {
    velocity.segment(p.idx * 3, 3) = p.velocity;
    pos.segment(p.idx * 3, 3) = p.pos;
    if (std::isnan(p.pos.norm())) {
      encounteredNAN = true;
      nanCount++;
    }
  }

  if (encounteredNAN)
    std::printf("WARNING: encountered NAN in getCurrentPosVelocityVec() %d\n", nanCount);

  return std::make_pair(pos, velocity);
}


Simulation::PrimitiveCollisionInformation
Simulation::isInContactWithObstacle(const Vec3d &pos, const Vec3d &v_in, const VecXd &x_prim_new,
                                    const VecXd &v_prim_new) const {
  PrimitiveCollisionInformation info{};
  info.collides = false;
  info.primitiveId = -1;
  for (int i = 0; i < primitives.size(); i++) {
    Primitive *p = primitives[i];
    Vec3d primPos = x_prim_new.segment(3 * i, 3);
    Vec3d primVel = v_prim_new.segment(3 * i, 3);
    if (p->isEnabled) {
      // test t=0
      if (p->isInContact(primPos, pos, v_in, info.normal, info.dist, info.v_out)) {
        info.collides = true;
        info.primitiveId = i;

        return info;
      }

      if (p->isInContact(primPos, pos + v_in * sceneConfig.timeStep * 0.5, v_in, info.normal, info.dist, info.v_out)) {
        info.collides = true;
        info.primitiveId = i;

        return info;
      }

      // test t=timestep
      if (p->isInContact(primPos, pos + v_in * sceneConfig.timeStep, v_in, info.normal, info.dist, info.v_out)) {
        info.collides = true;
        info.primitiveId = i;

        return info;
      }
    }
  }


  return info;
}


Simulation::SelfCollisionInformation
Simulation::isSelfCollision(const Particle &a, const Particle &b, const Vec3d &x_a, const Vec3d &x_b, const Vec3d &v_a,
                            const Vec3d &v_b) const {
  SelfCollisionInformation info{};
  info.collides = false;


  double collisionThresh = a.radii + b.radii;
  Vec3d posDiff = x_a - x_b;
  Vec3d v0 = posDiff;
  Vec3d v = v_a - v_b;
  Vec3d p0 = v0, p1 = v0 + sceneConfig.timeStep * v;
  double minDist = std::min(p0.norm(), p1.norm());
  double tMid = -2 * (v.dot(v0)) / v.squaredNorm();
  if ((tMid >= 0) && (tMid <= sceneConfig.timeStep)) {
    minDist = std::min(minDist, (v0 + tMid * v).norm());
  }

  if (minDist < collisionThresh) {
    info.collides = true;
    info.particleId1 = std::min(a.idx, b.idx);
    info.particleId2 = std::max(a.idx, b.idx);
    info.normal = ((info.particleId1 == a.idx) ? 1 : -1) * posDiff.normalized();
  }

  return info;
}




std::pair<Simulation::collisionInfoPair, std::vector<std::vector<Simulation::SelfCollisionInformation>>>
Simulation::collisionDetection(const VecXd &x_n, const VecXd &v, const VecXd &x_prim, const VecXd &v_prim) {
  timeSteptimer.tic("CollisionInitVecMap");

  std::vector<PrimitiveCollisionInformation> infos;
  std::vector<SelfCollisionInformation> selfinfos, selfinfosCopy;

  std::vector<std::vector<SelfCollisionInformation>> layers;
  std::map<int, std::set<int>> selfCollisionMap; // store particle ids

  std::vector<PrimitiveCollisionInformation> fullCollisionArr(particles.size());


  timeSteptimer.toc();
  timeSteptimer.tic("CollisionInitTable");
  MatXi primitiveCollisionTable(primitives.size(), particles.size());
  primitiveCollisionTable.setZero();

  timeSteptimer.toc();
  timeSteptimer.tic("CollisionInitMat");
  MatXi selfCollisionTable(particles.size(), particles.size());
  selfCollisionTable.setZero();


  timeSteptimer.toc();
  timeSteptimer.tic("ExtColDetect");


  if (contactEnabled) {

    //primitive collisions.........
#pragma omp parallel for if (OPENMP_ENABLED)
    for (int i = 0; i < particles.size(); i++) {
      const Particle &p = particles[i];
      fullCollisionArr[i] = isInContactWithObstacle(x_n.segment(p.idx * 3, 3), v.segment(p.idx * 3, 3), x_prim, v_prim);
    }

#pragma omp parallel for if (OPENMP_ENABLED)
    for (int i = 0; i < particles.size(); i++) {
      if (fullCollisionArr[i].collides) {
        const Particle &p = particles[i];
#pragma omp critical
        {
          fullCollisionArr[i].particleId = p.idx;
          primitiveCollisionTable(fullCollisionArr[i].primitiveId, p.idx) = 1;
          infos.emplace_back(fullCollisionArr[i]);
        }
      }
    }

    timeSteptimer.toc();


    if (selfcollisionEnabled) {
      timeSteptimer.tic("SelfColDetectInit");
      Vec3d maxDim = particles[0].pos, minDim = particles[0].pos;
      double maxRadii = particles[0].radii;
      for (int id = 0; id < particles.size(); id++) {
        Vec3d pos = x_n.segment(id * 3, 3);
        maxRadii = std::max(maxRadii, particles[id].radii);
        for (int i = 0; i < 3; i++) {
          maxDim[i] = std::max(maxDim[i], pos[i]);
          minDim[i] = std::min(minDim[i], pos[i]);
        }
      }

      Vec3d dim = maxDim - minDim;
      int axis = 0;
      for (int i = 1; i < 3; i++)
        if (dim[i] > dim[axis])
          axis = i;

      int cellNum = std::max(std::min(512, (int) (dim[axis] / (maxRadii * 2))), 1);
      double cellDim = dim[axis] / cellNum;
      int sweepCellRadius = std::ceil(maxRadii * 2 / cellDim) + 2;
      std::vector<std::set<int>> cells(cellNum, std::set<int>());

      timeSteptimer.toc();
      timeSteptimer.tic("SelfColDetectInitCell");
      for (int i = 0; i < particles.size(); i++) {
        int cellIdx = std::min((int) ((x_n.segment(i * 3, 3)[axis] - minDim[axis]) / cellDim), cellNum - 1);
        cells[cellIdx].insert(i);
      }
//      std::printf("finished inserting cells..");

      timeSteptimer.toc();
      timeSteptimer.tic("SelfColDetectInit");
#pragma omp parallel for if (OPENMP_ENABLED)
      for (int cellId1 = 0; cellId1 < cellNum; cellId1++) {
        for (int cellId2 = cellId1; cellId2 < std::min(cellId1 + sweepCellRadius + 2, cellNum); cellId2++) {
          for (int p1Idx : cells[cellId1]) {
            for (int p2Idx: cells[cellId2]) {
              if ((cellId1 == cellId2) && (p1Idx < p2Idx))
                continue; // self-block self-block test avoid duplicate tests
              if (!pointpointConnectionTable[p1Idx][p2Idx]) {
                Vec3d x_i = x_n.segment(p1Idx * 3, 3);
                Vec3d x_j = x_n.segment(p2Idx * 3, 3);
                double dist = (x_i - x_j).norm();
                if (dist > 1.0)
                  continue;
                SelfCollisionInformation info = isSelfCollision(particles[p1Idx], particles[p2Idx], x_i,
                                                                x_j, v.segment(p1Idx * 3, 3),
                                                                v.segment(p2Idx * 3, 3));
                if (info.collides) {
                  int infoId = 0;
#pragma omp critical
                  {
                    selfinfos.emplace_back(info);
                    infoId = selfinfos.size() - 1;
                    if (selfCollisionMap.find(info.particleId1) == selfCollisionMap.end()) {
                      selfCollisionMap[info.particleId1] = {};
                    }
                    if (selfCollisionMap.find(info.particleId2) == selfCollisionMap.end()) {
                      selfCollisionMap[info.particleId2] = {};
                    }
                    selfCollisionMap[info.particleId1].insert(info.particleId2);
                    selfCollisionMap[info.particleId2].insert(info.particleId1);
                    selfCollisionTable(info.particleId1, info.particleId2) = infoId;
                    selfCollisionTable(info.particleId2, info.particleId1) = infoId;


                  }
                }

              }
            }
          }
        }
      }


    }
  }

  timeSteptimer.toc();


  timeSteptimer.tic("contactSorting");

  Simulation::collisionInfoPair detections = std::make_pair(infos, selfinfos);

  if (contactEnabled && selfcollisionEnabled) {
    layers = contactSorting(detections, selfCollisionMap, selfCollisionTable);
  }
  timeSteptimer.toc();

  return std::make_pair(detections, layers);

}

void checkInfoConsistency(int particleId, int otherId, int infoIdx, Simulation::SelfCollisionInformation &info) {
#ifdef  DEBUG_SELFCOLLISION
  if (infoIdx == -1) {
    std::printf("ERROR: info already cleared in table %d-%d!\n", particleId, otherId);
    exit(8);
  }
  bool assertion = (((info.particleId1 == particleId) && (info.particleId2 == otherId)) ||
                    ((info.particleId1 == otherId) && (info.particleId2 == particleId)));
  if (!assertion) {
    std::printf("ERROR: info inconsistent in table %d-%d!\n", particleId, otherId);
    std::printf("%d %d \n", particleId, otherId);
    std::printf("%d %d\n", info.particleId1, info.particleId2);
    exit(8);
  }
#endif
}

void removeInfoFromDataStructures(int particleId, int otherId, int infoIdx, Simulation::SelfCollisionInformation &info,
                                  std::map<int, std::set<int>> &selfCollisionMap, MatXi &selfCollisionTable) {
#ifdef  DEBUG_SELFCOLLISION
  checkInfoConsistency(particleId, otherId, infoIdx, info);
  { //map table consistency check
    bool found1 = selfCollisionMap[otherId].find(particleId) != selfCollisionMap[otherId].end();
    bool found2 = selfCollisionMap[particleId].find(otherId) != selfCollisionMap[particleId].end();
    bool notCleared = (selfCollisionTable(particleId, otherId) >= 0) && (selfCollisionTable(otherId, particleId) >= 0);
    if (!found1)
      std::printf("ERROR: NOT FOUND %d in %d!\n", otherId, particleId);
    if (!found2)
      std::printf("ERROR: NOT FOUND %d in %d!\n", particleId, otherId);
    if (!notCleared)
      std::printf("ERROR: TABLE_CENTER IS ALREADY CLEARED %d-%d!\n", particleId, otherId);
    if (!(found1 && found2 && notCleared))
      exit(8);

  }
#endif

  // remove both from tables
  selfCollisionTable(particleId, otherId) = -1;
  selfCollisionTable(otherId, particleId) = -1;

  // remove both from maps
  selfCollisionMap[otherId].erase(particleId);
  selfCollisionMap[particleId].erase(otherId);

}

std::vector<std::vector<Simulation::SelfCollisionInformation>>
Simulation::contactSorting(Simulation::collisionInfoPair &detections, std::map<int, std::set<int>> &selfCollisionMap,
                           MatXi &selfCollisionTable) {
  std::vector<PrimitiveCollisionInformation> &primitiveCollisions = detections.first;

  std::vector<SelfCollisionInformation> &selfCollisions = detections.second;
  std::vector<int> layerNumbers(selfCollisions.size(), -1);

  std::vector<std::vector<SelfCollisionInformation>> layers;

#ifdef  DEBUG_SELFCOLLISION
  std::printf("All self collisions: %zu\n", selfCollisions.size());
  for (SelfCollisionInformation &info : selfCollisions)
    std::printf("%d-%d ", info.particleId1, info.particleId2);
  std::printf("\n-----------------------\n");

  for (int i = 0; i < particles.size(); i++) {
    if (selfCollisionMap.find(i) != selfCollisionMap.end()) {
      std::printf("collision with %d:", i);
      for (int d : selfCollisionMap[i]) {
        std::printf("%d-", d);
      }
      std::printf("\n");
    }
  }
#endif

  int processedSelfCollisions = 0;
  int maxLayerIdx = 0;
  std::set<int> frontier, newFrontier, involvedInLayer;

  // collisions with primitives go into first layer
  for (const PrimitiveCollisionInformation &info : primitiveCollisions) {
    if (!selfCollisionMap[info.particleId].empty())
      frontier.insert(info.particleId);
    involvedInLayer.insert(info.particleId);
  }

  // Greedily add lonely selfcollisions ( collisions that look like O-O ) to be processed in one layer
  for (auto it = selfCollisionMap.begin(); it != selfCollisionMap.end(); it++) {
    int particleId = it->first;
    if (selfCollisionMap[particleId].size() != 1) continue;
    int otherId = *(it->second.begin());
    if (selfCollisionMap[otherId].size() != 1) continue;
    if (frontier.find(particleId) != frontier.end()) continue;
    if (frontier.find(otherId) != frontier.end()) continue;
    int infoIdx = selfCollisionTable(particleId, otherId);
    SelfCollisionInformation &info = selfCollisions[infoIdx];
    checkInfoConsistency(particleId, otherId, infoIdx, info);

    // remove from datastructure
    removeInfoFromDataStructures(particleId, otherId, infoIdx, info, selfCollisionMap, selfCollisionTable);

    // add
#ifdef  DEBUG_SELFCOLLISION
    std::printf("%d-%d", particleId, otherId);
#endif
    processedSelfCollisions++;
    layerNumbers[infoIdx] = 0;
#ifdef  DEBUG_SELFCOLLISION
    if (!selfCollisionMap[otherId].empty()) {
      std::printf("ERROR: NOT INDEPENDENT SELF-COLLISION\n");
      exit(0);
    }
#endif
    involvedInLayer.insert(particleId);
    involvedInLayer.insert(otherId);
  }


  int currentLayer = 1;
  while (processedSelfCollisions != selfCollisions.size()) {
#ifdef  DEBUG_SELFCOLLISION
    std::printf("*********************\nEXPLOIT NEW FRONTIER: start size %d\n*********************\n", frontier.size());
#endif

    //exploit the frontier
    while (!frontier.empty()) {
#ifdef  DEBUG_SELFCOLLISION
      std::printf("====================\nExploring frontier at layer %d:", currentLayer);
      for (int particleId : frontier) std::printf("%d ", particleId);
      std::printf("\n");
#endif
      newFrontier.clear();
      involvedInLayer.clear();
      maxLayerIdx = std::max(currentLayer, maxLayerIdx);

      for (int particleId : frontier) {
        if (selfCollisionMap[particleId].empty()) continue;
        if (involvedInLayer.find(particleId) != involvedInLayer.end()) continue; // already involved in layer

        for (int otherId : selfCollisionMap[particleId]) {
          // eligibility check
          if (involvedInLayer.find(otherId) != involvedInLayer.end()) continue; // already involved in layer
          int infoIdx = selfCollisionTable(particleId, otherId);
          SelfCollisionInformation &info = selfCollisions[infoIdx];
          checkInfoConsistency(particleId, otherId, infoIdx, info);

          // remove from datastructure
          removeInfoFromDataStructures(particleId, otherId, infoIdx, info, selfCollisionMap, selfCollisionTable);


#ifdef  DEBUG_SELFCOLLISION
          std::printf("%d-%d", particleId, otherId);
#endif
          processedSelfCollisions++;
          involvedInLayer.insert(particleId);
          involvedInLayer.insert(otherId);
          layerNumbers[infoIdx] = currentLayer;
          if (!selfCollisionMap[otherId].empty())
            newFrontier.insert(otherId);
          break;
        }
      }
#ifdef  DEBUG_SELFCOLLISION
      std::printf("\n");
#endif

      currentLayer++;
      frontier = newFrontier;
    }
#ifdef  DEBUG_SELFCOLLISION
    std::printf("Finished current exploitation\n");
#endif
    if (processedSelfCollisions != selfCollisions.size()) { // sorting not finished. Explore new frontier
#ifdef  DEBUG_SELFCOLLISION
      std::printf("Sorting not finished. Exploring new frontier\n");
#endif
      // cleanup maps by removing empty nodes
      for (auto it = selfCollisionMap.begin(); it != selfCollisionMap.end();) {
        if (selfCollisionMap[it->first].empty()) {
          selfCollisionMap.erase(it++);
        } else {
          ++it;
        }
      }

      // explore new frontiers, prioritize chain head
      for (auto it = selfCollisionMap.begin(); it != selfCollisionMap.end(); it++) {
        if (selfCollisionMap[it->first].size() == 1) {
          frontier.insert(it->first);
          break;
        }
      }

      if (frontier.empty()) { // there is a self-collision loop in this case
        if (!selfCollisionMap.empty())
          frontier.insert(selfCollisionMap.begin()->first);
      }

      if (frontier.empty()) {
        std::printf("ERROR: NOTHING TO EXPLORE BUT NOT ALL COLLISIONS ARE PROCESSED!\n");
        exit(8);
      }
    }
  }

#ifdef  DEBUG_SELFCOLLISION
  std::printf("Finished sorting, total layers: %d\n", maxLayerIdx);
  std::printf("Init Layers\n");
#endif
  layers.clear();
  for (int i = 0; i < maxLayerIdx + 1; i++) {
    layers.emplace_back(std::vector<SelfCollisionInformation>());
  }

#ifdef  DEBUG_SELFCOLLISION
  std::vector<std::set<int>> involvedVertices;
  for (int i = 0; i < maxLayerIdx + 1; i++) {
    involvedVertices.emplace_back(std::set<int>());
  }
#endif
  for (int i = 0; i < selfCollisions.size(); i++) {
    int layerId = layerNumbers[i];
#ifdef  DEBUG_SELFCOLLISION
    if (layerId == -1) {
      std::printf("ERROR: selfInfo is not processed %d-%d\n", selfCollisions[i].particleId1, selfCollisions[i].particleId2);
      exit(8);
    }
    if (layerId > maxLayerIdx) {
      std::printf("ERROR: selfInfo has invalid layerId %d\n", layerId);
      exit(8);
    }
    if (involvedVertices[layerId].find(selfCollisions[i].particleId1) != involvedVertices[layerId].end()) {
      std::printf("ERROR: particle %d already involved in layerd %d\n", selfCollisions[i].particleId1, layerId);
      exit(8);
    }
    if (involvedVertices[layerId].find(selfCollisions[i].particleId2) != involvedVertices[layerId].end()) {
      std::printf("ERROR: particle %d already involved in layerd %d\n", selfCollisions[i].particleId2, layerId);
      exit(8);
    }
    involvedVertices[layerId].insert(selfCollisions[i].particleId1);
    involvedVertices[layerId].insert(selfCollisions[i].particleId2);
#endif
    selfCollisions[i].layerId = layerId;
    layers[layerId].emplace_back(selfCollisions[i]);
  }

#ifdef  DEBUG_SELFCOLLISION
  std::printf("finished producing layers\n");
#endif
  return layers;
}


std::pair<VecXd, VecXd>
Simulation::calculateDryFrictionVector(const VecXd &f, completeCollisionInfo &detectionInfos) {
  VecXd r(3 * particles.size());
  VecXd r_prim(3 * primitives.size());
  r_prim.setZero();
  r.setZero();

if (!contactEnabled)
  return std::make_pair(r, r_prim);

  std::vector<PrimitiveCollisionInformation> &infos = detectionInfos.first.first;
  if (contactEnabled) {
    for (PrimitiveCollisionInformation &info : infos) {
      if (info.primitiveId != -1) {
        int pIdx = info.particleId;
        Primitive *prim = primitives[info.primitiveId];

          Vec3d d = f.segment(pIdx * 3, 3) - particles[pIdx].mass * info.v_out;
          info.d = d;
          Vec3d r_i = calcualteDryFrictionForce(info.normal, info.d, prim->mu, info.type);
          r.segment(pIdx * 3, 3) += r_i;
          info.r = r_i;


      }
    }


    if (selfcollisionEnabled) {
      int layerCount = 0;
      for (std::vector<SelfCollisionInformation> &selfInfos: detectionInfos.second) {
        for (SelfCollisionInformation &info : selfInfos) {
          Vec3d f_iA = f.segment(info.particleId1 * 3, 3) + r.segment(info.particleId1 * 3, 3);
          Vec3d f_iB = f.segment(info.particleId2 * 3, 3) + r.segment(info.particleId2 * 3, 3);
          double m_A = particles[info.particleId1].mass;
          double m_B = particles[info.particleId2].mass;
          Vec3d d = (f_iA / m_A - f_iB / m_B);
          info.d = d;

          double clothFrictionalCoeff = 0.1;
          double k = (m_A * m_B) / (m_A + m_B); //
          Vec3d r_i = k * calcualteDryFrictionForce(info.normal, info.d, clothFrictionalCoeff, info.type);
          info.r = r_i;
          r.segment(info.particleId1 * 3, 3) += r_i;
          r.segment(info.particleId2 * 3, 3) -= r_i;
        }

        layerCount++;

      }

    }
  }

  return std::make_pair(r, r_prim);
}


// ((dr_df, dr_dfprim), dr_ddensity)
std::pair<std::pair<SpMat, SpMat>, VecXd> Simulation::calculatedr_df(const completeCollisionInfo &infos,
                                                                     bool calculatePrimitiveGradient = false,
                                                                     bool calculateDensityGradient = false) const {
  int threeM = 3 * particles.size();
  SpMat dr_df(threeM, threeM);
  TripleVector dr_dftriplets;
  SpMat drprim_dfprim(threeM, threeM);
  VecXd dr_drho(threeM);
  dr_drho.setZero();
  dr_df.setZero();
  drprim_dfprim.setZero();

  if (contactEnabled) {
    //primitive collisions

    for (const PrimitiveCollisionInformation &info : infos.first.first) {
      if (info.primitiveId != -1) {
        int pIdx = info.particleId;
        Primitive *prim = primitives[info.primitiveId];
          Mat3x3d dri_dd = calculatedri_dfi(info.normal, info.d, prim->mu);
          insertIntoTriplets<3, 3>(dr_dftriplets, dri_dd, 3 * pIdx, 3 * pIdx);
          if (calculateDensityGradient) {
            Vec3d dd_ddensity = -particles[pIdx].area * info.v_out;
            dr_drho.segment(info.particleId * 3, 3) += dri_dd * dd_ddensity;
          }
      }
    }

    dr_df.setFromTriplets(dr_dftriplets.begin(), dr_dftriplets.end());

    if (selfcollisionEnabled) {
      int layerCount = 0;
      double rho = sceneConfig.fabric.density;
      for (const std::vector<SelfCollisionInformation> &selfInfos: infos.second) { // has to use second, because 1. layer matters 2. info.d and info.r is only updated in second
        layerCount++;
        SpMat dr_df_last = dr_df;
        TripleVector dr_df_delta;

        for (const SelfCollisionInformation &info : selfInfos) {
          int nA = info.particleId1;
          int nB = info.particleId2;
          double m_A = particles[nA].mass;
          double m_B = particles[nB].mass;
          double clothFrictionalCoeff = 0.1;

          double k = (m_A * m_B) / (m_A + m_B);
          Mat3x3d dcalc_dd = calculatedri_dfi(info.normal, info.d, clothFrictionalCoeff);
          Mat3x3d dr_dd = k * dcalc_dd;

          Mat3x3d dr_dfiA = dr_dd / m_A;
          Mat3x3d dr_dfiB = -dr_dd / m_B;

          insertIntoTriplets<3, 3>(dr_df_delta, dr_dfiA, 3 * nA, 3 * nA); // drA_dfiA
          insertIntoTriplets<3, 3>(dr_df_delta, dr_dfiB, 3 * nA, 3 * nB);  // drA_dfiB
          insertIntoTriplets33(dr_df_delta, -dr_dfiA, 3 * nB, 3 * nA); // drB_dfiA
          insertIntoTriplets33(dr_df_delta, -dr_dfiB, 3 * nB, 3 * nB);  // drB_dfiB

          MatXd dfiA_df = dr_df_last.block(3 * nA, 0, 3, threeM);
          MatXd dfiB_df = dr_df_last.block(3 * nB, 0, 3, threeM);

          MatXd dr_dfiA_x_dfiA_df = dr_dfiA * dfiA_df; // 3x3m
          MatXd dr_dfiB_x_dfiB_df = dr_dfiB * dfiB_df; // 3x3m
          MatXd dr_dfprev = dr_dfiA_x_dfiA_df + dr_dfiB_x_dfiB_df;
          MatXd negdr_dfprev = -dr_dfprev;

          insertIntoTriplets(dr_df_delta, dr_dfprev, 3, threeM, 0, 0, 3 * nA, 0);
          insertIntoTriplets(dr_df_delta, negdr_dfprev, 3, threeM, 0, 0, 3 * nB, 0);

        }

        dr_df.setFromTriplets(dr_df_delta.begin(), dr_df_delta.end());
        dr_df += dr_df_last;


      }


    }

  }

  return std::make_pair(std::make_pair(dr_df, drprim_dfprim), dr_drho);
}

std::vector<VecXd> Simulation::calculatedr_dmu(const std::vector<PrimitiveCollisionInformation> &infos,
                                               const std::vector<int> &primIds) const {
  std::vector<int> primIdToIdxMap(primitives.size(), -1);
  std::vector<VecXd> dr_dmu;
  for (int i = 0; i < primIds.size(); i++) {
    primIdToIdxMap[primIds[i]] = i;
    dr_dmu.emplace_back(VecXd(particles.size() * 3));
    dr_dmu[i].setZero();
  }

  if (contactEnabled) {
    //primitive collisions
    for (const PrimitiveCollisionInformation &info : infos) {
      if (info.primitiveId != -1) {
        int pIdx = info.particleId;
        int loc = primIdToIdxMap[info.primitiveId];

        if (loc != -1) {

          Primitive *prim = primitives[info.primitiveId];

            Vec3d dridmui = calculatedri_dmu(info.normal, info.d, prim->mu);
            dr_dmu[loc].segment(3 * pIdx, 3) += dridmui;

        }
      }

    }


  }


  return dr_dmu;
}


void Simulation::stepPrimitives(VecXd &x_n, VecXd &v_n, VecXd &delta_v) {
//  std::printf("entered step:\n");
  VecXd v_new = v_n + delta_v;
  VecXd x_new = x_n + v_new * sceneConfig.timeStep;
  for (int i = 0; i < primitives.size(); i++) {
    Primitive *prim = primitives[i];
    if (contactEnabled && prim->isEnabled && (!prim->isStaitc)) {
      prim->velocity = v_new.segment(i * 3, 3);
      prim->center = x_new.segment(i * 3, 3);

    } else {
    }
  }

}


void printVec(std::string label, Vec3d a) {
  std::printf("%s: %.2f %.2f %.2f\n", label.c_str(), a[0], a[1], a[2]);
}


Vec3d Simulation::calcualteDryFrictionForce(const Vec3d &n, const Vec3d &f_i, double mu, CollisionType& type) const {
  Vec3d r_i;
  r_i.setZero();
  double sign_dist = f_i.dot(n); // contact force magnitude
  Vec3d f_N = n * sign_dist; // contact force
  Vec3d f_T = f_i - f_N; //
  double dT_norm = f_T.norm();
  bool printCases = false;

  if (sign_dist >= 0.0) {
    // take off
    if (printCases)
      std::printf("c1: takeoff");
    type = CollisionType::TAKE_OFF;
  } else {
    r_i += -f_N; // r_N += -f_N
    if (dT_norm <= mu * std::abs(sign_dist)) { // stick
      r_i += -f_T;
      if (printCases)
        std::printf("c2: stick");
      type = CollisionType::STICK;

    } else {
      if (printCases)
        std::printf("c3: slide");
      r_i += -mu * std::abs(sign_dist) * f_T.normalized();
//      r_i += mu * sign_dist * f_T.normalized();
      type = CollisionType::SLIDE;

    }
  }

  return r_i;
}

// backward r_i --> f_i
Vec3d Simulation::calculatedri_dmu(const Vec3d &n, const Vec3d &f_i, double mu) const {
  Vec3d dri_dmu;
  dri_dmu.setZero();
  double dN_signed_dist = f_i.dot(n); // contact force magnitude
  Vec3d f_N = n * dN_signed_dist; // contact force
  Vec3d f_T = f_i - f_N; //
  double dT_norm = f_T.norm();
  if (dN_signed_dist < 0.0) {
    if (dT_norm > mu * std::abs(dN_signed_dist)) { //slip
      dri_dmu += -std::abs(dN_signed_dist) * f_T.normalized();
    }
  }

  return dri_dmu;
}

Mat3x3d Simulation::calculatedri_dfi(const Vec3d &n, const Vec3d &f_i, double mu) const {
  // finite difference checked 02/04/2021
  Mat3x3d dri_dfi;
  Mat3x3d I_three = Mat3x3d::Identity();
  dri_dfi.setZero();
  double sign_dist = f_i.dot(n); // contact force magnitude
  Vec3d f_N = n * sign_dist; // contact force
  Vec3d f_T = f_i - f_N; //
  double dT_norm = f_T.norm();
  bool printCases = false;

  // f_N = f_i.dot(n) * n
  Mat3x3d dfN_dfi = n * n.transpose();
  Mat3x3d dfT_dfi = I_three - dfN_dfi;

  if (sign_dist >= 0.0) {
    // take off do nothing
  } else {
    //  r_i += -f_N;
    dri_dfi += -1.0 * dfN_dfi;
    if (dT_norm <= mu * std::abs(sign_dist)) { // stick
      // r_i += -f_T;
      dri_dfi += -1.0 * dfT_dfi;

    } else {
      //
      Vec3d a = f_T.normalized();
      double b = sign_dist;
      Mat3x3d da_dfT = (I_three - a * a.transpose()) / f_T.norm();
      Mat3x3d da_df = da_dfT * dfT_dfi;
      Vec3d db_df = n;
      dri_dfi += mu * (b * da_df + a * db_df.transpose());

    }
  };

  return dri_dfi;

}

double Simulation::evaluateEnergy(const VecXd &x_new) const {
  double inertia = 0.5 * (x_new - s_n).transpose() * M * (x_new - s_n);
  double energy = 0;

#pragma omp parallel for if (OPENMP_ENABLED)
  for (int i = 0; i < sysMat[currentSysmatId].constraints.size(); i++) {
    Constraint *c = sysMat[currentSysmatId].constraints[i];
    c->evaluateEnergy(x_new);
  }

  for (int i = 0; i < sysMat[currentSysmatId].constraints.size(); i++) {
    energy += sysMat[currentSysmatId].constraints[i]->energyBuffer;
  }

  energy = energy * sceneConfig.timeStep * sceneConfig.timeStep + inertia;
  return energy;
}

double Simulation::evaluateSystemEnergy(const VecXd &v, const VecXd &x) const {
  double deformE = 0;
  for (Constraint *c : sysMat[currentSysmatId].constraints) {
    deformE += c->evaluateEnergy(x);
  }
  double potentialE = 0.5 * v.transpose() * M * v;
  std::printf("systemE: %.8f deformE: %.8f potentialE: %.8f\n", deformE + potentialE, deformE, potentialE);
  return deformE + potentialE;

}





void Simulation::stepPrimitives() {
  for (Primitive *p : primitives) {
    p->step(sceneConfig.timeStep);
  }


}



double Simulation::stepFixPoints(double t) {

  double t_splinefraction = t / (sceneConfig.timeStep * sceneConfig.stepNum);
  switch (Simulation::sceneConfig.trajectory) {
    case TrajectoryConfigs::NO_TRAJECTORY: {
      break;
    }

    case TrajectoryConfigs::FIXED_POINT_TRAJECTORY: {
      for (int fixedPointIdx = 0; fixedPointIdx < sysMat[currentSysmatId].fixedPoints.size(); fixedPointIdx++) {
        sysMat[currentSysmatId].fixedPoints[fixedPointIdx].pos = fixedPointTrajectory[forwardRecords.size() -
                                                                                      1].segment(fixedPointIdx * 3, 3);
      }
       break;
    }

    case TrajectoryConfigs::TRAJECTORY_DRESS_TWIRL: {
      Vec3d rotationCenter = restShapeMidPoint;
      for (int fixedPointIdx = 0; fixedPointIdx < sysMat[currentSysmatId].fixedPoints.size(); fixedPointIdx++) {
        Vec3d posLast = sysMat[currentSysmatId].fixedPoints[fixedPointIdx].pos;
        rotationCenter[1] = posLast[1];
        Eigen::AngleAxis rotMat(0.02, Vec3d(0, 1, 0));
        Vec3d relativePos = posLast - rotationCenter;
        Vec3d relativePosRot = rotMat * relativePos;

        sysMat[currentSysmatId].fixedPoints[fixedPointIdx].pos =
                relativePosRot + rotationCenter; // posLast.cross(Vec3d(0, 1, 0)) * 0.012;
      }

      break;
    }


    case TrajectoryConfigs::PER_STEP_TRAJECTORY: {
      for (int fixedPointIdx = 0; fixedPointIdx < sysMat[currentSysmatId].fixedPoints.size(); fixedPointIdx++) {
        sysMat[currentSysmatId].fixedPoints[fixedPointIdx].pos = rlFixedPointPos.segment(fixedPointIdx * 3, 3);
      }

      perstepTrajectory.emplace_back(rlFixedPointPos);
      break;
    }



    default: {
      for (Spline &s : sysMat[currentSysmatId].controlPointSplines) {
        sysMat[currentSysmatId].fixedPoints[s.pFixed].pos = s.evalute(t_splinefraction);
      }
      break;
    }

  }

  return t_splinefraction;
}

void Simulation::stepNN(int idx, const VecXd& x,const VecXd& v,const VecXd& fixedPointPos) {

  sceneConfig.trajectory = TrajectoryConfigs::PER_STEP_TRAJECTORY;
  x_n = x;
  v_n = v;

  for ( Particle &p : particles) {
    p.velocity = v_n.segment(p.idx * 3, 3);
    p.pos = x_n.segment(p.idx * 3, 3);
  }

  int fixedPointDofs = fixedPointPos.rows();
  int requiredDofs = sysMat[currentSysmatId].fixedPoints.size() * 3;
  if (fixedPointDofs != requiredDofs) {
    Logging::logWarning("require" + std::to_string(requiredDofs) + " fixed point dofs but input fixed point dof is " + std::to_string(fixedPointDofs) + "\n");
  }
  rlFixedPointPos = fixedPointPos;


  step();
  forwardRecords[forwardRecords.size()-1].stepIdx = idx;

}
void Simulation::step() {
   timeSteptimer = Timer();
  timeSteptimer.enabled = true;
  timeSteptimer.ticStart();
  timeSteptimer.tic("init");
  if (explosionEncountered) {
    forwardRecords.emplace_back(forwardRecords[forwardRecords.size() - 1]);
    return;
  }

  for (int i = sysMat.size() - 1; i >= 0; i--) {
    if (forwardRecords.size() >= sysMat[i].startFrameNum) {
      if ((currentSysmatId != i) || (forwardRecords.size() == 1)) {
        currentSysmatId = i;
        projections = VecXd(sysMat[currentSysmatId].constraintNum);
        projections.setZero();
        for (int j = 0; j < Constraint::CONSTRAINT_NUM; j++) {
          projections_pertype[j] = VecXd(sysMat[currentSysmatId].constraintNum_pertype[j]);
          projections_pertype[j].setZero();
        }
      }

      break;
    }

  }

  explosionEncountered = false;
  bool printOptimizationDetails = false;
  int TOTAL_DOF = particles.size() * 3;
  double CONVERGE_EPSILON = forwardConvergenceThreshold;
#ifdef  DEBUG_EXPLOSION
  std::vector<std::pair<VecXd, VecXd>> fAndRs;
  std::vector<std::pair<VecXd, VecXd>> x_newAndErrors;
#endif
  // save previous state into x_n, v_n
  updateCurrentPosVelocityVec();

  stepPrimitives();

  std::pair<collisionInfoPair, std::vector<std::vector<SelfCollisionInformation>>> detectionInfos;

  VecXd x_new(3 * particles.size());
  VecXd v_new(3 * particles.size());
  VecXd f_int(3 * particles.size()), f_ext(3 * particles.size());
  VecXd f(3 * particles.size()); // this is f in contact force calculation, but it's different than f_int + f_ext
  VecXd r(3 * particles.size());
  ForwardInformation returnRecord;
  returnRecord.t = forwardRecords[forwardRecords.size() - 1].t + sceneConfig.timeStep;
  returnRecord.stepIdx = forwardRecords.size();
  returnRecord.sysMatId = currentSysmatId;
  returnRecord.splines = sysMat[currentSysmatId].controlPointSplines;
  r.setZero();

  double windFactor = fillForces(f_int, f_ext, v_n, x_n, returnRecord.t);
  s_n = x_n + sceneConfig.timeStep * v_n + sceneConfig.timeStep * sceneConfig.timeStep * M_inv * f_ext;

  returnRecord.x_prev = x_n;
  returnRecord.v_prev = v_n;
  returnRecord.s_n = s_n;
  returnRecord.windFactor = windFactor;
  returnRecord.windParams.segment(0, 3) = wind * windNorm;
  returnRecord.windParams[3] = windFrequency;
  returnRecord.windParams[4] = windPhase;
  returnRecord.totalConverged = forwardRecords[forwardRecords.size() - 1].totalConverged;


  double curEnergy = 0;
  if (printOptimizationDetails) {
    curEnergy = evaluateSystemEnergy(v_n, x_n);
    double curError = evaluateEnergy(x_n);
    std::printf("-------current energy: %.6f error: %.6f\n",
                curEnergy,
                curError);
  }


  VecXd f_primitives(3 * primitives.size());
  VecXd v_n_primitives(3 * primitives.size());
  VecXd x_n_primitives(3 * primitives.size());
  VecXd delta_v_primitives(3 * primitives.size());
  VecXd xnew_n_primitives(3 * primitives.size());
  VecXd vnew_n_primitives(3 * primitives.size());
  VecXd A_t_times_p_pertype[Constraint::CONSTRAINT_NUM];

  f_primitives.setZero();
  delta_v_primitives.setZero();
  for (int i = 0; i < primitives.size(); i++) {
    f_primitives.segment(3 * i, 3) = primitives[i]->getForces(sceneConfig.timeStep);
    delta_v_primitives.segment(3 * i, 3) = f_primitives.segment(3 * i, 3) / primitives[i]->mass * sceneConfig.timeStep;
    x_n_primitives.segment(3 * i, 3) = primitives[i]->center;
    v_n_primitives.segment(3 * i, 3) = primitives[i]->velocity;
  }


  vnew_n_primitives = v_n_primitives + delta_v_primitives;
  xnew_n_primitives = x_n_primitives + sceneConfig.timeStep * vnew_n_primitives;
  for (int i = 0; i < primitives.size(); i++) {
    Primitive *prim = primitives[i];
    if (contactEnabled && prim->isEnabled && (!prim->isStaitc)) {
      prim->velocity = vnew_n_primitives.segment(i * 3, 3);
      prim->center = xnew_n_primitives.segment(i * 3, 3);

    }
  }

  double t_spline = 0;
  if (!debugSkipfixedpointstep)
    t_spline = stepFixPoints(returnRecord.t);


#pragma omp parallel for if (OPENMP_ENABLED)
  for (int i = 0; i < particles.size(); i++) {
    Particle &p = particles[i];
    p.pos = s_n.segment(3 * p.idx, 3);
    p.velocity = (s_n - x_n).segment(p.idx * 3, 3) / sceneConfig.timeStep;

  }


  timeSteptimer.toc();


  {

    timeSteptimer.tic("PD init");
    Eigen::VectorXd b(3 * particles.size());
    b.setZero();
    double newEnergy = 0;
    curEnergy = 1000000;
    double min_xdiff = ((s_n - x_n).norm() * (1.0 / particles.size()));
    int min_xdiffiter = 0;

    VecXd M_times_sn = M * s_n;
    VecXd P_times_xn = sysMat[currentSysmatId].P * x_n;
    VecXd x_new_lastconverging = x_n, v_new_lastconverging = v_n;


    timeSteptimer.toc();
    PD_TOTAL_ITER = (-std::log10(forwardConvergenceThreshold)) * 150;

    for (int iterIdx = 0; iterIdx < PD_TOTAL_ITER; iterIdx++) {

      timeSteptimer.tic("iter init");
      std::pair<Eigen::VectorXd, Eigen::VectorXd> posVelVec = getCurrentPosVelocityVec();
      VecXd &v_now = posVelVec.second;
      VecXd &x_now = posVelVec.first;
      timeSteptimer.toc();

      timeSteptimer.tic("projection");
      projections.setZero();
      for (int i = 0; i < Constraint::CONSTRAINT_NUM; i++) {
        projections_pertype[i].setZero();
      }

#pragma omp parallel for if (OPENMP_ENABLED)
      for (int i = 0; i < sysMat[currentSysmatId].constraints.size(); i++) {
        Constraint *c = sysMat[currentSysmatId].constraints[i];

        projections.segment(c->c_idx, c->constraintNum) = c->project(x_now);

        projections_pertype[c->constraintType].segment(c->c_weightless_idx, c->constraintNum) = projections.segment(
                c->c_idx, c->constraintNum);
      }

      timeSteptimer.toc();

      timeSteptimer.tic("At_p");

      if (calcualteSeperateAt_p) {
#pragma omp parallel for if (OPENMP_ENABLED)
        for (int i = 0; i < Constraint::CONSTRAINT_NUM; i++) {
          returnRecord.At_p_weightless_pertype[i] =
                  sysMat[currentSysmatId].A_t_pertype[i] * projections_pertype[i] / std::sqrt(*(k_stiff_arr[i]));
        }
      }

      timeSteptimer.toc();
      timeSteptimer.tic("calc b");
      b = (sysMat[currentSysmatId].A_t * projections) * (sceneConfig.timeStep * sceneConfig.timeStep) + M_times_sn;
      timeSteptimer.toc();

      enum PD_VERSION {
          VELOCITY_BASED, // formulation from LY 20, includes dry frictional contact
          POSITION_BASED // formulation from Bouaziz 14, does not include friction calculation
      };
      double deltav_prim_changes = 0;
      std::pair<VecXd, VecXd> collisionResults;
      PD_VERSION pdVersion = VELOCITY_BASED;
      switch (pdVersion) {
        case POSITION_BASED: {
          x_new = sysMat[currentSysmatId].solver.solve(b);
          newEnergy = evaluateEnergy(x_new);
          v_new = (x_new - x_n) / sceneConfig.timeStep;

          for (Particle &p  : particles) {
            p.pos = x_new.segment(p.idx * 3, 3);
            p.velocity = v_new.segment(p.idx * 3, 3);

          }
          break;
        }

        case VELOCITY_BASED: {
          timeSteptimer.tic("b_tilde and f");
          VecXd b_tilde = (b - P_times_xn) / sceneConfig.timeStep;
          f = b_tilde - sysMat[currentSysmatId].C * v_now;
          VecXd r_prim(3 * primitives.size());
          timeSteptimer.toc();

          if (contactEnabled) {
            if (iterIdx == 0) {
              detectionInfos = collisionDetection(x_n, v_now, xnew_n_primitives, v_n_primitives);
            }
            timeSteptimer.tic("calc r");
            collisionResults = calculateDryFrictionVector(f, detectionInfos);
            r = collisionResults.first;
            r_prim = collisionResults.second;
            timeSteptimer.toc();
          } else {
            r.setZero();
            r_prim.setZero();
          }
          timeSteptimer.tic("solve and update");
          v_new = sysMat[currentSysmatId].solver.solve(b_tilde + r);
          x_new = v_new * sceneConfig.timeStep + x_n;

          timeSteptimer.toc();


          bool testvbased_vs_xbased = false;
          if (testvbased_vs_xbased) {
            VecXd x_new2 = sysMat[currentSysmatId].solver.solve(b + sceneConfig.timeStep * r);
            VecXd diff = x_new - x_new2;
            std::printf("xnew1: %.6f xnew2: %.6f diff: %.6f error: %.6f\n", x_new.norm(), x_new2.norm(), diff.norm(),
                        std::abs(diff.dot(x_now.cwiseInverse()) / diff.rows()));
          }

#ifdef  DEBUG_EXPLOSION
          x_newAndErrors.emplace_back(std::make_pair(x_new, P * x_new - (b_tilde + r)));
            fAndRs.emplace_back(std::make_pair(f, r));
#endif

          timeSteptimer.tic("step primitives");
          if (false) {
            VecXd delta_v_primitives_new = (f_primitives * sceneConfig.timeStep + r_prim).cwiseProduct(
                    m_primitivesinv);
            deltav_prim_changes = (delta_v_primitives_new - delta_v_primitives).norm();
            delta_v_primitives = delta_v_primitives_new;

            vnew_n_primitives = v_n_primitives + delta_v_primitives;
            xnew_n_primitives = x_n_primitives + sceneConfig.timeStep * vnew_n_primitives;
            for (int i = 0; i < primitives.size(); i++) {
              Primitive *prim = primitives[i];
              if (contactEnabled && prim->isEnabled && (!prim->isStaitc)) {
                prim->velocity = vnew_n_primitives.segment(i * 3, 3);
                prim->center = xnew_n_primitives.segment(i * 3, 3);

              }
            }

          }
          timeSteptimer.toc();
          timeSteptimer.tic("update");


//            #pragma omp parallel for if (OPENMP_ENABLED)
          for (int i = 0; i < particles.size(); i++) {
            Particle &p = particles[i];
            p.velocity = v_new.segment(p.idx * 3, 3);
            p.pos = x_new.segment(p.idx * 3, 3);
          }
          timeSteptimer.toc();

          break;


        }
      }

      timeSteptimer.tic("Convergence Test and Cleanup");
      double x_diff = ((x_new - x_now).norm() * (1.0 / particles.size()));
      if (x_diff < min_xdiff) {
        min_xdiff = x_diff;
        min_xdiffiter = iterIdx;
        x_new_lastconverging = x_new;
        v_new_lastconverging = v_new;
      }
      bool converged = x_diff < CONVERGE_EPSILON;
      bool finished = converged || (iterIdx == PD_TOTAL_ITER - 1);

      if (converged) {
        // has converged
        if (printOptimizationDetails)
          std::printf("pd converging at iteration %d with error: %.10f thresh: %.10f\n", iterIdx, x_diff,  CONVERGE_EPSILON);
        totalIter += iterIdx + 1;
        returnRecord.converged = true;
        returnRecord.totalConverged++;

        returnRecord.convergeIter = iterIdx + 1;
        returnRecord.cumulateIter = returnRecord.convergeIter +
                                    (forwardRecords.empty() ? 0 : forwardRecords[forwardRecords.size() -
                                                                                 1].cumulateIter);
        timeSteptimer.toc();
        break;
      } else {
        curEnergy = newEnergy;
        if (iterIdx == PD_TOTAL_ITER - 1) {
          returnRecord.converged = false;
          returnRecord.convergeIter = PD_TOTAL_ITER;
          returnRecord.cumulateIter = returnRecord.convergeIter +
                                      (forwardRecords.empty() ? 0 : forwardRecords[forwardRecords.size() -
                                                                                   1].cumulateIter);

          bool revertToLastConverging = true;
          if (revertToLastConverging) {
            x_new = x_new_lastconverging;
            v_new = v_new_lastconverging;
#pragma omp parallel for if (OPENMP_ENABLED)
            for (int i = 0; i < particles.size(); i++) {
              Particle &p = particles[i];
              p.velocity = v_new.segment(p.idx * 3, 3);
              p.pos = x_new.segment(p.idx * 3, 3);
            }
          }
          if (printOptimizationDetails)
            if (printVerbose)
              std::printf("pd not converged at last iteration %d\n", iterIdx);
        }
        timeSteptimer.toc();
      }
    }



  }


  timeSteptimer.ticEnd();
  VecXd x_fixedpoints(3 * sysMat[currentSysmatId].fixedPoints.size());
  x_fixedpoints.setZero();
  for (FixedPoint &p : sysMat[currentSysmatId].fixedPoints) {
    x_fixedpoints.segment(p.idx * 3, 3) = p.pos;
  }

  returnRecord.x = x_new;
  returnRecord.v = v_new;
  returnRecord.f = f;
  returnRecord.r = r;
  returnRecord.timer = timeSteptimer.getReportMicroseconds();
  returnRecord.accumTimer = Timer::addTimer(returnRecord.timer.timeMicroseconds,
                                            forwardRecords[forwardRecords.size() - 1].accumTimer);

  returnRecord.totalRuntime =
          forwardRecords[forwardRecords.size() - 1].totalRuntime + returnRecord.timer.totalMicroseconds;
  returnRecord.x_prim = xnew_n_primitives;
  returnRecord.v_prim = vnew_n_primitives;
  returnRecord.vpre_prim = v_n_primitives;
  returnRecord.f_prim = f_primitives;
  returnRecord.x_fixedpoints = x_fixedpoints;
  returnRecord.collisionInfos = detectionInfos;
  returnRecord.simDurartionFraction = t_spline;
  returnRecord.avgDeformation = calculateTriangleDeformation(x_new);
  returnRecord.maxDeformation = calculateMaxTriangleDeformation(x_new);

  forwardRecords.push_back(returnRecord);

  for (Primitive *p : allPrimitivesToRender) {
    p->updateForwardRecord();
  }
  explosionEncountered = false;

  if (printOptimizationDetails) {
    double systemEnergy = evaluateSystemEnergy(v_new, x_new);
    std::printf("stepping finished with system energy: %.8f error-energy: %.5f totalIter: %d\n",
                systemEnergy, evaluateEnergy(x_new), totalIter);

    if (systemEnergy > 1000000.0) {
      explosionEncountered = true;
      std::printf("explosion!!!\n");
      std::printf("norms: x:%.4f v:%.4f f:%.4f r:%.4f\n", x_new.norm(), v_new.norm(), f.norm(), r.norm());
    }
  }


}


VecXd Simulation::solveDirect(VecXd& dL_dxnew, double t_2, SpMat& dproj_dxnew_t, SystemMatrix& currentSysMat, SpMat& dr_df_plusI_t, SpMat&  dr_df_t) {
  timeSteptimer.tic("solveDirect"); // solve
  SpMat delta_P_T = t_2 * dproj_dxnew_t * currentSysMat.A * dr_df_plusI_t - currentSysMat.C_t * dr_df_t;

  SpMat P_N_T = currentSysMat.P - delta_P_T;
  factorizeDirectSolverSparseLU(P_N_T, solverSparseLU, "factorize sparseLU");
  VecXd u_star = solverSparseLU.solve(dL_dxnew);
  timeSteptimer.toc();
  return u_star;
}


Simulation::BackwardInformation
Simulation::stepBackwardNN(Simulation::BackwardTaskInformation &taskInfo, VecXd &dL_dxnew, VecXd &dL_dvnew,
                           const ForwardInformation &forwardInfo_new, bool isStart, const VecXd &dL_dxinit,
                           const VecXd &dL_dvinit) {
  Simulation::BackwardInformation backwardInfoNew = backwardInfoDefault;
  backwardInfoNew.dL_dx = dL_dxnew;
  backwardInfoNew.dL_dv = dL_dvnew;

  return stepBackward(taskInfo, backwardInfoNew, forwardInfo_new, isStart, dL_dxinit, dL_dvinit);
}


Simulation::BackwardInformation
Simulation::stepBackward(Simulation::BackwardTaskInformation &taskInfo, Simulation::BackwardInformation &gradient_new,
                         const ForwardInformation &forwardInfo_new,
                         bool isStart, const VecXd &dL_dxinit, const VecXd &dL_dvinit) {

  if (gradientClipping) {
    double dL_dx_maxnorm = gradientClippingThreshold;
    if ( gradient_new.dL_dx.norm() > dL_dx_maxnorm * particles.size()) {
//      Logging::logColor("gradient clipped at " + std::to_string(forwardInfo_new.stepIdx) + ": norm " + d2str(gradient_new.dL_dx.norm(), 4), Logging::MAGENTA);
      gradient_new.dL_dx = gradient_new.dL_dx  * dL_dx_maxnorm * particles.size() / gradient_new.dL_dx.norm();
    }
  }
  int TOTAL_DOF = particles.size() * 3;
  bool printApproximations = false;
  int backwardIdx = forwardRecords.size() - forwardInfo_new.stepIdx;

  VecXd dL_dx(TOTAL_DOF), dL_dv(TOTAL_DOF);
  VecXd x_new = forwardInfo_new.x;
  VecXd v_new = forwardInfo_new.v;
  VecXd f_new = forwardInfo_new.f;
  VecXd &dL_dxnew = gradient_new.dL_dx;
  VecXd &dL_dvnew = gradient_new.dL_dv;
  int goodMatrix = gradient_new.goodMatrixCounter;
  int badMatrix = gradient_new.badMatrixCounter;
  double dL_dknew = 0;
  double dL_dknew_pertype[Constraint::CONSTRAINT_NUM] = {0};
  VecXd dL_dfext_vec(3 * particles.size());
  SystemMatrix &currentSysMat = sysMat[forwardInfo_new.sysMatId];
  VecXd dL_dm(9 * currentSysMat.fixedPoints.size());
  std::vector<std::pair<int, double>> dL_dmu;
  dL_dfext_vec.setZero();
  dL_dm.setZero();
  dL_dx = dL_dxinit;
  dL_dv = dL_dvinit;
  BackwardInformation ret = backwardInfoDefault;
  SpMat dr_df_plusI(3 * particles.size(), 3 * particles.size()),
          dr_df_plusI_t(3 * particles.size(), 3 * particles.size()),
          dr_df(3 * particles.size(), 3 * particles.size()),
          dr_df_t(3 * particles.size(), 3 * particles.size());
  dr_df_plusI.setIdentity();
  dr_df_plusI_t.setIdentity();
  dr_df.setZero();
  dr_df_t.setZero();
  VecXd dr_dd(3 * particles.size());
  dr_dd.setZero();

  VecXd u_star_prev(3 * particles.size()), u_star(3 * particles.size());
  u_star_prev.setZero();
  u_star.setZero();

  Timer timeSteptimer;
  timeSteptimer.enabled = true;
  timeSteptimer.ticStart();
  timeSteptimer.tic("contact_grad");

  if (contactEnabled) {
    bool calculatePrimitiveGradient = false;
    std::pair<std::pair<SpMat, SpMat>, VecXd> drall_dfall = calculatedr_df(forwardInfo_new.collisionInfos,
                                                                           calculatePrimitiveGradient,
                                                                           taskInfo.dL_density &&
                                                                           taskInfo.adddr_dd);
    dr_df = drall_dfall.first.first;
    dr_df_t = dr_df.transpose();
    dr_dd = drall_dfall.second;
    dr_df_plusI += dr_df;
    dr_df_plusI_t = dr_df_plusI.transpose();

  }

  timeSteptimer.toc();
  double timeStep = sceneConfig.timeStep;
  double t_2 = timeStep * timeStep;

  {

    timeSteptimer.tic("PD init");


    // step1: dl/dx add dL/vnew * dvnew/dx yellow
    dL_dx += dL_dvnew * (-1.0 / sceneConfig.timeStep); //yellow
    dproj_dxnew = SpMat(currentSysMat.constraintNum, 3 * particles.size());
    dproj_dxnew.setZero();
    TripleVector triplets;
    timeSteptimer.toc();
    timeSteptimer.tic("projection");
#pragma omp parallel for if (OPENMP_ENABLED)
    for (int i = 0; i < sysMat[currentSysmatId].constraints.size(); i++) {
      Constraint *c = sysMat[currentSysmatId].constraints[i];
      c->projectBackwardPrecompute(x_new);
    }

    for (Constraint *c : sysMat[currentSysmatId].constraints) {
      c->projectBackward(x_new, triplets);
    }

    dproj_dxnew.setFromTriplets(triplets.begin(), triplets.end());
    dproj_dxnew_t = dproj_dxnew.transpose();

    timeSteptimer.toc();
    ret.rho = 0; // (P_inv * delta_P).toDense().eigenvalues().cwiseAbs().maxCoeff();

    // solve


    {
      timeSteptimer.tic("delta_P_T");
      int MAX_ITER_NUM = 400;
      u_star_prev.setZero();
      timeSteptimer.toc();

      if (backwardGradientForceDirectSolver) {
        u_star = solveDirect(dL_dxnew, t_2, dproj_dxnew_t, currentSysMat, dr_df_plusI_t, dr_df_t);
      } else {
        timeSteptimer.tic("solveIterative");
        for (int iterIdx = 0; iterIdx < MAX_ITER_NUM; iterIdx++) {
          // step1: dl/dx add dL/dxnew * dxnew/dx
          VecXd dr_df_t_x_u_star_prev = dr_df_t * u_star_prev;

          VecXd deltaU = t_2 * dproj_dxnew_t *
                         (currentSysMat.A * (dr_df_t_x_u_star_prev + u_star_prev)) -
                         currentSysMat.C_t * dr_df_t_x_u_star_prev;
          VecXd rhs = dL_dxnew + deltaU;
          u_star = currentSysMat.solver.solve(rhs);
          bool converged =
                  (std::abs((u_star - u_star_prev).norm() / (particles.size() * 1.0))) < backwardConvergenceThreshold;
          bool isLastIter = (iterIdx + 1 == MAX_ITER_NUM);
          if (converged || isLastIter) {
            timeSteptimer.toc(); // solveIterative
            ret.backwardIters = iterIdx + 1;
            ret.backwardTotalIters = ret.backwardIters + gradient_new.backwardTotalIters;
            ret.convergedAccum = gradient_new.convergedAccum;
            if (converged) {
              ret.converged = true;
              ret.convergedAccum++;
            } else {
              ret.converged = false;
              timeSteptimer.toc();
              // direct solve
              u_star = solveDirect(dL_dxnew, t_2, dproj_dxnew_t, currentSysMat, dr_df_plusI_t, dr_df_t);
            }
            break;
          }

          u_star_prev = u_star;
        }
      }

    }




    // step2: red
    dL_dx += M * u_star;

    // step3: blue dl/dv = dL/dnew * dxnew/dv  blue edg
    dL_dv += sceneConfig.timeStep * (dr_df_plusI * M).transpose() * u_star;

    // step4:dl/dx add dL/v * dv/dx green edge
    if (!isStart) {
      dL_dx += dL_dv * 1.0 / sceneConfig.timeStep;
    }

  }


  timeSteptimer.tic("gradients");
  if (taskInfo.dL_dmu) {
    bool bruteCalc = true;
    std::vector<VecXd> dr_dmu = calculatedr_dmu(forwardInfo_new.collisionInfos.first.first,
                                                taskInfo.mu_primitives);
    for (int i = 0; i < taskInfo.mu_primitives.size(); i++) {
      double dL_dmui = dr_dmu[i].transpose() * sceneConfig.timeStep * u_star;
      dL_dmu.emplace_back(gradient_new.dL_dmu[i].first, dL_dmui + gradient_new.dL_dmu[i].second);
    }
    ret.dL_dmu = dL_dmu;

  }

  //spline parameters
  ret.dL_dsplines = gradient_new.dL_dsplines;
  ret.dL_dxfixed = forwardInfo_new.x_fixedpoints;
  ret.dL_dxfixed_accum = forwardInfo_new.x_fixedpoints;
  ret.dL_dxfixed.setZero();
  ret.dL_dxfixed_accum.setZero();


  if (taskInfo.dL_dcontrolPoints && (!currentSysMat.controlPointSplines.empty())) {
    int sysMatId = forwardInfo_new.sysMatId;
    // 3m x s
    SpMat rhs_xfixed = sceneConfig.timeStep * sceneConfig.timeStep * dr_df_plusI * sysMat[sysMatId].A_t_dp_dxfixed; // 3m x s
    ret.dL_dxfixed = rhs_xfixed.transpose() * u_star;
    perStepGradient.emplace_back(ret.dL_dxfixed);
    if (forwardInfo_new.stepIdx == 1) {
      std::reverse(perStepGradient.begin(), perStepGradient.end());
    }



    if (gradient_new.dL_dxfixed_accum.rows() == ret.dL_dxfixed_accum.rows() ) {
      ret.dL_dxfixed_accum = ret.dL_dxfixed + gradient_new.dL_dxfixed_accum;

    }
    for (int splineIdx = 0; splineIdx < sysMat[sysMatId].controlPointSplines.size(); splineIdx++) {
      Spline &s = sysMat[sysMatId].controlPointSplines[splineIdx];
      MatXd dxfixed_dspline = s.dxfixed_dcontrolPoints(forwardInfo_new.simDurartionFraction);
      SpMat dxfixed_dspline_sparse = dxfixed_dspline.sparseView().pruned();
      SpMat rhs_spline = sceneConfig.timeStep * sceneConfig.timeStep * dr_df_plusI *
              (sysMat[sysMatId].A_t_dp_dxfixed.block(0, s.pFixed * 3, 3 * particles.size(), 3) *
                         dxfixed_dspline_sparse); // 3m x s
      rhs_spline = rhs_spline.pruned();
      VecXd deltaGrad = rhs_spline.transpose() * u_star;

      ret.dL_dsplines[sysMatId][splineIdx] += deltaGrad;
    }
  }

  if (taskInfo.dL_density) {
    VecXd dMy_dd = Area * (forwardInfo_new.x_prev + sceneConfig.timeStep * forwardInfo_new.v_prev +
                           sceneConfig.timeStep * sceneConfig.timeStep * gravity_n);
    VecXd df_dd = Area * (forwardInfo_new.v_prev + sceneConfig.timeStep * gravity_n);
    VecXd rhs = dMy_dd + sceneConfig.timeStep * dr_df * df_dd - Area * forwardInfo_new.x +
                sceneConfig.timeStep * dr_dd;
    ret.dL_ddensity = gradient_new.dL_ddensity + u_star.dot(rhs);
  }

  for (int i = 0; i < Constraint::CONSTRAINT_NUM; i++) {

    if (taskInfo.dL_dk_pertype[i]) {
        VecXd dA_t_times_p_dk = forwardInfo_new.At_p_weightless_pertype[i];
        VecXd A_t_A_weightless_times_xnew = currentSysMat.A_t_times_A_pertype[i] * x_new;
        VecXd df_dk = sceneConfig.timeStep * dA_t_times_p_dk - sceneConfig.timeStep * A_t_A_weightless_times_xnew;

        VecXd rhs = sceneConfig.timeStep * sceneConfig.timeStep * dA_t_times_p_dk +
                    sceneConfig.timeStep * dr_df * df_dk -
                    sceneConfig.timeStep * sceneConfig.timeStep * A_t_A_weightless_times_xnew;
        double dL_dk_new = u_star.dot(rhs);

        VecXd dL_dk_perelem = u_star.cwiseProduct(rhs);

        ret.dL_dk_pertype[i] = gradient_new.dL_dk_pertype[i] + dL_dk_new;


    }
  }


  if (taskInfo.dL_dfext) {
    dL_dfext_vec = sceneConfig.timeStep * sceneConfig.timeStep * dr_df_plusI.transpose() * u_star *
                   forwardInfo_new.windFactor;
    ret.dL_dfext = gradient_new.dL_dfext;
    for (int i = 0; i < particles.size(); i++) {
      Vec3d delta = dL_dfext_vec.segment(i * 3, 3);
      if (sceneConfig.windConfig == WindConfig::WIND_SIN_AND_FALLOFF) {
        delta = delta.cwiseProduct(windFallOff.segment(i * 3, 3));
      }
      ret.dL_dfext += delta;
    }
  }


  if (taskInfo.dL_dconstantForceField) {
    ret.dL_dconstantForceField = gradient_new.dL_dconstantForceField +  sceneConfig.timeStep * sceneConfig.timeStep * dr_df_plusI.transpose() * u_star;


  }

  if (taskInfo.dL_dwindFactor) {
    dL_dfext_vec = sceneConfig.timeStep * sceneConfig.timeStep * dr_df_plusI.transpose() * u_star;
    ret.dL_dwindtimestep = gradient_new.dL_dwindtimestep;
    ret.dL_dwindtimestep[forwardInfo_new.stepIdx] = 0;
    for (int i = 0; i < particles.size(); i++) {
      ret.dL_dwindtimestep[forwardInfo_new.stepIdx] += dL_dfext_vec.segment(i * 3, 3).dot(
              (wind * windNorm).cwiseProduct(windFallOff.segment(i * 3, 3)));
    }
  }

  if (taskInfo.dL_dfwind) {
     if ((sceneConfig.windConfig != WindConfig::WIND_SIN) &&
        (sceneConfig.windConfig != WindConfig::WIND_SIN_AND_FALLOFF))
      std::printf("WARNING: Calculating gradient for sin wind model but config is not sin wind!\n");

    dL_dfext_vec = sceneConfig.timeStep * sceneConfig.timeStep * dr_df_plusI.transpose() * u_star;
    ret.dL_dwind = gradient_new.dL_dwind;
    Mat3x5d dfext_dwind;
    dfext_dwind.setZero();
    dfext_dwind.block<3, 3>(0, 0) = Mat3x3d::Identity() * forwardInfo_new.windFactor;
     Vec3d windForce = forwardInfo_new.windParams.segment(0, 3);
    double cos_atplusb = std::cos(
            forwardInfo_new.windParams[3] * forwardInfo_new.t + forwardInfo_new.windParams[4]);
    dfext_dwind.col(3) = windForce * cos_atplusb * 0.5 * forwardInfo_new.t;
    dfext_dwind.col(4) = windForce * cos_atplusb * 0.5;
    Mat5x3d dfext_dwind_T = dfext_dwind.transpose();

    Vec3d dL_dfext_total;
    dL_dfext_total.setZero();
    for (int i = 0; i < particles.size(); i++) {
      if (sceneConfig.windConfig == WindConfig::WIND_SIN_AND_FALLOFF) {
        dL_dfext_total += dL_dfext_vec.segment(i * 3, 3).cwiseProduct(windFallOff.segment(i * 3, 3));
      } else {
        dL_dfext_total += dL_dfext_vec.segment(i * 3, 3);

      }
    }
      Vec5d dL_dwind = dfext_dwind_T * dL_dfext_total;

      ret.dL_dwind += dL_dwind;


  }

  timeSteptimer.toc(); // gradients
  timeSteptimer.ticEnd();
  ret.timer = timeSteptimer.getReportMicroseconds();
  ret.accumSolvePerformanceReport = Timer::addPerf(ret.timer.solvePerfReport,
                                                   gradient_new.accumSolvePerformanceReport);
  ret.accumTimer = Timer::addTimer(ret.timer.timeMicroseconds, gradient_new.accumTimer);
  ret.totalRuntime = gradient_new.totalRuntime + ret.timer.totalMicroseconds;

  ret.dL_dx = dL_dx;
  ret.dL_dv = dL_dv;
  ret.loss = gradient_new.loss;


  return ret;
}


Vec3d Simulation::getInitParticlePos(int i, int j) const {

  double gridSizeX = sceneConfig.fabric.clothDimX / (sceneConfig.fabric.gridNumX - 1);
  double gridSizeY = sceneConfig.fabric.clothDimY / (sceneConfig.fabric.gridNumY - 1);
  Vec3d origin = Vec3d(-(sceneConfig.fabric.gridNumY - 1) / 4.0 * gridSizeY, 15, 0);
  Vec3d gridP = Vec3d(j * gridSizeY, -i * gridSizeX, 0);

  return gridP + origin;
}


int Simulation::gridIndicesToParticle(int a, int b) const {
  if ((a < 0) || (b < 0) || (a >= Simulation::sceneConfig.fabric.gridNumY) ||
      (b >= Simulation::sceneConfig.fabric.gridNumX)) {
    return -1;
  }
  int idx = a * Simulation::sceneConfig.fabric.gridNumX + b;

  return idx;
};

void Simulation::initScene() {
  primitives.clear();
  primitives.reserve(10);
  allPrimitivesToRender.clear();

  switch (sceneConfig.primitiveConfig) {

    case Y0PLANE: {
      primitives.push_back(&bowl);

      for (Particle &p : particles) {
        p.velocity_init = p.velocity = Vec3d(0, -10, 0);
      }

      break;
    }

    case PLANE_BUST_WEARHAT: {
      primitives.push_back(&sphere_head);
      break;
    }

    case PLANE_AND_SPHERE: {
      primitives.push_back(&sphere2);
      break;
    }


    case BIG_SPHERE: {
      primitives.push_back(&veryBigSphere);
      break;
    }

    case SLOPE: {
      primitives.push_back(&slope);
      break;
    }

    case SLOPE_SIMPLIEFIED: {
      primitives.push_back(&slope);
      break;
    }


    case FOOT: {
      sockLeg.createNewMesh(Vec3d(0, 0, 0), sceneConfig.sockLegOrientation.normalized(), 5, 4),
              primitives.push_back(&sockLeg);
      break;
    }



    case NONE:
    default: {

    }

  }


  for (Primitive *p : primitives) {
    allPrimitivesToRender.push_back(p);
  }

  m_primitives = VecXd(3 * primitives.size());
  m_primitivesinv = VecXd(3 * primitives.size());
  Vec3d one;
  one.setOnes();

  std::function<void(std::vector<Primitive *> &)> enablePrimitives = [&enablePrimitives](
          std::vector<Primitive *> &prims) {
      for (Primitive *p : prims) {
        p->isEnabled = true;
        p->isStaitc = true;
        p->gravityEnabled = false;
        if (p->isPrimitiveCollection) {
          enablePrimitives(p->primitives);
        }
      }
  };

  enablePrimitives(primitives);
  switch (sceneConfig.primitiveConfig) {
    case Y0PLANE: {

      plane1.center = Vec3d(0, 0, 0);
      bowl.mu = 0;
    break;
    }

    case PLANE_AND_SPHERE: {
      Vec3d centerLowPoint = 0.5 * (restShapeMinDim + restShapeMaxDim);
      centerLowPoint[1] = restShapeMinDim[1];
      plane1.center = plane1.centerInit = centerLowPoint - Vec3d(0, sphere2.radius * 2 + 0.1, 0);
      sphere2.mu = 0.9;
      sphere2.center = sphere2.centerInit =
              plane1.center + Vec3d(sphere2.radius * 0.3, sphere2.radius, sphere2.radius * 0.1);
      sphere2.rotates = false;
      break;
    }

    case BIG_SPHERE: {
      Vec3d centerLowPoint = 0.5 * (restShapeMinDim + restShapeMaxDim);
      centerLowPoint[1] = restShapeMinDim[1];
      veryBigSphere.center =  veryBigSphere.centerInit = Vec3d(-0.50,-16.00,0.00);
      veryBigSphere.mu = 0.0;
      veryBigSphere.discretized = true;
      break;
    }



    case FOOT: {
      Capsule &foot = *(sockLeg.foot);
      Capsule &leg = *(sockLeg.leg);
      Vec3d centerHighPoint = 0.5 * (restShapeMinDim + restShapeMaxDim);
      centerHighPoint[1] = restShapeMaxDim[1];
      sockLeg.mu = 0;
      sockLeg.center = sockLeg.centerInit = centerHighPoint + Vec3d(0, 3, -4);

      break;
    }






    case PLANE_BUST_WEARHAT: {
      Vec3d centerLowPoint = 0.5 * (restShapeMinDim + restShapeMaxDim);
      centerLowPoint[1] = restShapeMinDim[1];

      plane1.center = plane1.centerInit = centerLowPoint - Vec3d(0, 0.5, 0) - Vec3d(0, 0, 4);
      sphere_head.mu = 0.1;
      sphere_head.rotates = sphere_head.gravityEnabled =  false;
      sphere_head.isStaitc = true;
      plane1.mu = 0.8;
      sphere_head.center = sphere_head.centerInit = plane1.center + Vec3d(0, sphere_head.radius + 0.5, -4);
      plane1.center[1] -= 2;
      break;
    }

    case SLOPE: {
      double zDist = std::abs(slope.center.z() - slope.upperLeft.z());
      double zGap = 5;
      double yGap = -2;
      Vec3d shift = (slope.lowerRight - slope.upperRight) * 0.5;

      Vec3d referencePoint = restShapeMinDim;
      referencePoint[0] = (restShapeMaxDim[0] + restShapeMinDim[0]) * 0.5;
      referencePoint[2] -= 1.0;

      slope.center = slope.centerInit =
              referencePoint + shift + Vec3d(0, -2, 0);
      slope.mu = 0.2;

      break;
    }

    case SLOPE_SIMPLIEFIED: {
      double zDist = std::abs(slope.center.z() - slope.upperLeft.z());
      double zGap = 5;
      double yGap = -2;
      Vec3d shift = (slope.lowerRight - slope.upperRight) * 0.4;

      Vec3d referencePoint = particles[285].pos_rest; // center of the SLOPE_FAB

      slope.center = slope.centerInit =
              referencePoint + shift + Vec3d(0, -0.3, 0);
      slope.mu = 0.2; //0.4 will hang 0.2 skip
      slopeGoal.mu = 0.9;
      break;
    }

    case NONE:
    default: {

    }

  }

  for (int i = 0; i < primitives.size(); i++) {
    m_primitives.segment(i * 3, 3) = one * primitives[i]->mass;
    m_primitivesinv.segment(i * 3, 3) = one / primitives[i]->mass;
  }
  for (Primitive *p : allPrimitivesToRender) {
    p->reset();
  }

  // Set scene-dependent fabric settings
  switch (sceneConfig.trajectory) {
    case CORNERS_1_WEARHAT:
    case CORNERS_2_WEARHAT: {
      {

        if (sceneConfig.primitiveConfig != PLANE_BUST_WEARHAT) {
          std::printf(
                  "ERROR: Trajectory setting is CORNERS_2_WEARHAT but primitive config is not PLANE_BUST_WEARHAT\n");
          break;
        }

        Vec3d bustCenter = sphere_head.center + Vec3d(0, sphere_head.radius * 0.6, 0);
        Vec3d hatCenter = (restShapeMinDim + restShapeMaxDim) * 0.5;
        Vec3d translation = bustCenter - hatCenter;
        Vec3d point00End = sysMat[0].fixedPoints[0].pos_rest + translation;

        sysMat[0].controlPointSplines[0].segments[0].yUp = 15;
        sysMat[0].controlPointSplines[0].moveEndPoint(0, point00End);

        if (sceneConfig.trajectory != CORNERS_1_WEARHAT) {
          sysMat[0].controlPointSplines[1].segments[0].yUp = 15;
          Vec3d point0XEnd = sysMat[0].fixedPoints[1].pos_rest + translation;
          sysMat[0].controlPointSplines[1].moveEndPoint(0, point0XEnd);
        }


      }
      break;
    }

    case CORNERS_2_WEARSOCK: {
      {
        if ((sysMat[0].fixedPoints.size() < 2) || (sysMat[0].controlPointSplines.size() < 2)) {
          std::printf("ERROR: Trajectory setting is CORNERS_2_WEARSOCK but there are <2 control splines\n");
          break;
        }

        if (sceneConfig.primitiveConfig != FOOT) {
          std::printf(
                  "ERROR: Trajectory setting is CORNERS_2_WEARSOCK but primitive config is not FOOT\n");
          break;
        }

        Vec3d footTop = sockLeg.center;
        footTop[1] += sockLeg.leg->length + sockLeg.leg->radius * 2;
        Vec3d sockTop;
        sockTop[0] = (restShapeMinDim[0] + restShapeMaxDim[0]) * 0.5;
        sockTop[1] = restShapeMaxDim[1];
        sockTop[2] = restShapeMinDim[2] + sockLeg.leg->radius;

        Vec3d translation = footTop - sockTop;

        for (Spline &s: sysMat[0].controlPointSplines) {
          s.segments[0].yUp = -28;
          s.moveEndPoint(0, sysMat[0].fixedPoints[s.pFixed].pos_rest + translation);
        }

        break;

      }
    }



    case NO_TRAJECTORY:
    case FIXED_POINT_TRAJECTORY:
    case CORNERS_2_UP:
    default: {
      break;
    }
  }


}


void Simulation::createConstraints() {
  if (bendingEnabled)
    createBendingConstraints();

  int setNum = 1;
  if (sceneConfig.attachmentPoints == AttachmentConfigs::CUSTOM_ARRAY) {
    setNum = sceneConfig.customAttachmentVertexIdx.size();
  }
  for (int setIdx = 0; setIdx < setNum; setIdx++) {
    for (Spring &s : springs) {
      sysMat[setIdx].constraints.push_back(&s);
    }
    for (Triangle &t : mesh)
      sysMat[setIdx].constraints.push_back(&t); // triangle strain

    for (TriangleBending &b : bendingConstraints)
      sysMat[setIdx].constraints.push_back(&b);

    for (AttachmentSpring &s : sysMat[setIdx].attachments) {
      sysMat[setIdx].constraints.push_back(&s);
    }
  }


}

void Simulation::createBendingConstraints() {
  std::map<std::pair<int, int>, std::vector<int>> edgeTriangleMap;

  for (int triIdx = 0; triIdx < mesh.size(); triIdx++) {
    Triangle &t = mesh[triIdx];

    for (int v1Idx = 0; v1Idx < 3; v1Idx++) {
      for (int v2Idx = v1Idx + 1; v2Idx < 3; v2Idx++) {
        int minIdx = std::min(t.idxArr[v1Idx], t.idxArr[v2Idx]), maxIdx = std::max(t.idxArr[v1Idx], t.idxArr[v2Idx]);
        int otherIdx = t.idxArr[(0 + 1 + 2) - (v1Idx + v2Idx)];
        std::pair<int, int> edgeKey(minIdx, maxIdx);

        if (edgeTriangleMap.find(edgeKey) == edgeTriangleMap.end()) {
          std::vector<int> single = {otherIdx};
          edgeTriangleMap[edgeKey] = single;
        } else {
          edgeTriangleMap[edgeKey].emplace_back(otherIdx);

        }
      }
    }
  }

  for (auto const&[vertexPair, idxArr] : edgeTriangleMap) {
    if (idxArr.size() > 1) {
      if (idxArr.size() > 2) {
        std::printf(" ERROR: Detected an edge shared with more than two triangles. This is a non-manifold!\n");
        exit(5);
      }
      bendingConstraints.emplace_back(vertexPair.first, vertexPair.second, idxArr[0], idxArr[1], particles);

    }
  }


}


void Simulation::clearConstraintsElementsAndRecords() {
  particles.clear();
  springs.clear();
  bendingConstraints.clear();
  mesh.clear();
  for (SystemMatrix &matrices : sysMat) {
    matrices.attachments.clear();
    matrices.fixedPoints.clear();
    matrices.constraints.clear();
    matrices.controlPointSplines.clear();

  }
  forwardRecords.clear();
  particleTriangleMap.clear();
  pointpointConnectionTable.clear();
}


std::vector<Vec3d> Simulation::rotatePointsAroundCenter(std::vector<Vec3d> &particleIn, Rotation rotation) {
  Vec3d minDim = particleIn[0], maxDim = particleIn[0];
  for (const Vec3d &p : particleIn) {
    for (int i = 0; i < 3; i++) {
      minDim[i] = std::min(minDim[i], p[i]);
      maxDim[i] = std::max(maxDim[i], p[i]);
    }
  }

  std::vector<Vec3d> out;
  for (const Vec3d &p : particleIn) {
    Vec3d rotated = rotation * (p - minDim);
    out.emplace_back(rotated);
  }

  return out;
}

void Simulation::createClothMeshFromModel(std::vector<Vec3d> &particleIn, std::vector<Vec3i> &meshIn) {
  if (particles.size() > 20000) {
    std::printf("Model has %zu vertices and %zu triangles. The program might take a long time to run\n", particleIn.size(),
                meshIn.size());
  }

  clearConstraintsElementsAndRecords();

  rotatePointsAccordingToConfig(particleIn);


  Vec3d minDim = particleIn[0], maxDim = particleIn[0];
  for (Vec3d &p : particleIn) {
    for (int i = 0; i < 3; i++) {
      minDim[i] = std::min(minDim[i], p[i]);
      maxDim[i] = std::max(maxDim[i], p[i]);
    }
  }


  std::printf("Bounding box min:%s max:%s\n", vec2str(minDim, 3).c_str(), vec2str(maxDim, 3).c_str());

  Vec3d dim = maxDim - minDim;

  double scale = sceneConfig.fabric.keepOriginalScalePoint ? 1.0 : std::max(std::max(dim[0], dim[1]), dim[2]) /
                                                                   sceneConfig.fabric.clothDimX;


  if (sceneConfig.fabric.keepOriginalScalePoint) {
    restShapeMaxDim = maxDim;
    restShapeMinDim = minDim;
  } else {
    restShapeMaxDim = maxDim - minDim;
    restShapeMaxDim /= scale;
    restShapeMinDim = Vec3d(0, 0, 0);
  }


  Vec3d translation = sceneConfig.fabric.keepOriginalScalePoint ? Vec3d(0, 0, 0) : restShapeMaxDim / 2.0;
  restShapeMinDim -= translation;
  restShapeMaxDim -= translation;
  meshInitTransofrmMinDim = minDim;
  meshInitTransofrmScale = scale;
  restShapeMidPoint = 0.5 * (restShapeMinDim + restShapeMaxDim);

  for (Vec3d &p : particleIn) {
    if (std::isnan(p[0]) || std::isnan(p[1]) || std::isnan(p[2])) {
      std::printf("encountered NAN pos (%.3f, %.3f, %.3f) when loading vertex number %zu from file\n", p[0], p[1], p[2],
                  particles.size());
    }
    Vec3d normalizedPos = sceneConfig.fabric.keepOriginalScalePoint ? p : (p - minDim) / scale - restShapeMaxDim;
    int idx = particles.size();
    particles.emplace_back(1, normalizedPos, normalizedPos,
                           Vec3d(0, 0, 0), Vec2i(0, 0),
                           idx);
    particleTriangleMap.emplace_back(std::vector<int>());
  }

  pointpointConnectionTable.resize(particles.size(), std::vector<bool>(particles.size(), false));


  for (Vec3i &t : meshIn) {
    mesh.emplace_back(t[0], t[1], t[2], particles, true);
    particleTriangleMap[t[0]].emplace_back(mesh.size() - 1);
    particleTriangleMap[t[1]].emplace_back(mesh.size() - 1);
    particleTriangleMap[t[2]].emplace_back(mesh.size() - 1);
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        pointpointConnectionTable[t[i]][t[j]] = true;
      }
  }


  for (int i = 0; i < particles.size(); i++) {
    assert(particles[i].idx == i);
  }

  // Create attachments
  createAttachments(true);


  updateBackwardDefaultInfo(); // resize vectors whose length is parameter-dependent



}


void Simulation::createAttachments(bool isModel) {
  if (isModel)  {
    switch (sceneConfig.attachmentPoints) {
      case LEFT_RIGHT_CORNERS_2: {
        //Add attachments
        Vec3d middleUpGoalPoint = Vec3d((restShapeMinDim[0] + restShapeMaxDim[0]) / 2.0, restShapeMaxDim[1],
                                        (restShapeMinDim[2] + restShapeMaxDim[2]) / 2.0);
        Vec3d upperLeftGoalPoint = Vec3d(restShapeMinDim[0], restShapeMaxDim[1],
                                         (restShapeMinDim[2] + restShapeMaxDim[2]) / 2.0);
        Vec3d upperRightGoalPoint = Vec3d(restShapeMaxDim[0], restShapeMaxDim[1],
                                          (restShapeMinDim[2] + restShapeMaxDim[2]) / 2.0);
        Vec3d lowerLeftGoalPoint = Vec3d(restShapeMinDim[0], restShapeMinDim[1],
                                         (restShapeMinDim[2] + restShapeMaxDim[2]) / 2.0);
        Vec3d lowerRightGoalPoint = Vec3d(restShapeMaxDim[0], restShapeMinDim[1],
                                          (restShapeMinDim[2] + restShapeMaxDim[2]) / 2.0);
        std::vector<std::pair<Vec3d, Particle *>> goalPoints = {{upperLeftGoalPoint,  &particles[0]},
                                                                {upperRightGoalPoint, &particles[0]}};



        for (Particle &p : particles) {
          for (std::pair<Vec3d, Particle *> &search : goalPoints) {
            Vec3d goalPos = search.first;
            if ((p.pos_rest - goalPos).norm() < (search.second->pos_rest - goalPos).norm()) {
              search.second = &p;
            }
          }
        }

        sysMat[0].controlPointSplines.reserve(goalPoints.size());
        sysMat[0].attachments.reserve(goalPoints.size());
        sysMat[0].fixedPoints.reserve(goalPoints.size());
        for (std::pair<Vec3d, Particle *> &search : goalPoints) {
          Particle *bestParticle = search.second;
          sysMat[0].fixedPoints.emplace_back(bestParticle->pos_rest, sysMat[0].fixedPoints.size());
          sysMat[0].attachments.emplace_back(bestParticle->idx, &particles, sysMat[0].attachments.size(),
                                             &(sysMat[0].fixedPoints));

          sysMat[0].controlPointSplines.emplace_back(bestParticle->pos_rest, bestParticle->pos_rest, 10,
                                                     bestParticle->idx);

        }

        break;
      }


      default: {
        break;
      }
    }
  } else {
    std::vector<std::pair<int, int>> gridXYs;
    switch (sceneConfig.attachmentPoints) {
      case LEFT_RIGHT_CORNERS_2: {
      
        std::pair<int, int> p1XY = std::make_pair( 0, 0);
        std::pair<int, int> p2XY = std::make_pair( 0,
                                                  sceneConfig.fabric.gridNumX - 1);
        std::pair<int, int> p3XY = std::make_pair(sceneConfig.fabric.gridNumY - 1, 0);
        std::pair<int, int> p4XY = std::make_pair(sceneConfig.fabric.gridNumY - 1, sceneConfig.fabric.gridNumX - 1);
        gridXYs = {p1XY, p2XY};
        break;
      }


      default: {
      }
    }

    for (std::pair<int, int> gridXY : gridXYs) {
      int pIdx = gridIndicesToParticle(gridXY.first, gridXY.second);
      Vec3d pos = particles[pIdx].pos;
      int fixedpointIdx = sysMat[0].fixedPoints.size();
      sysMat[0].fixedPoints.emplace_back(pos, fixedpointIdx);
      sysMat[0].attachments.emplace_back(pIdx, &particles, fixedpointIdx, &sysMat[0].fixedPoints);

    }

    switch (sceneConfig.trajectory) {
      case TrajectoryConfigs::CORNERS_2_UP: {
        {
          std::pair<int, int> p1XY = std::make_pair(  0, 0);


          Vec3d point00End = getInitParticlePos(sceneConfig.fabric.gridNumY - 1, 0);

          Vec3d point0XEnd = getInitParticlePos(sceneConfig.fabric.gridNumY - 1, sceneConfig.fabric.gridNumX - 1);
          double yUp = 8;

          sysMat[0].controlPointSplines.emplace_back(sysMat[0].fixedPoints[0].pos_rest, point00End, yUp, 0);

          if (sceneConfig.trajectory != CORNERS_1_WEARHAT) {
            std::pair<int, int> p2XY = std::make_pair(  0,
                                                      sceneConfig.fabric.gridNumX - 1);

            sysMat[0].controlPointSplines.emplace_back(sysMat[0].fixedPoints[1].pos_rest, point0XEnd, yUp, 1);
          }


        }
        break;
      }
      case TrajectoryConfigs::NO_TRAJECTORY:
      default: {

      }

    }
  }

  switch (sceneConfig.attachmentPoints) {
    case AttachmentConfigs::CUSTOM_ARRAY: {
        sysMat.resize(sceneConfig.customAttachmentVertexIdx.size());
        for (int setIdx = 0; setIdx < sceneConfig.customAttachmentVertexIdx.size(); setIdx++) {
          std::pair<double, std::vector<int>> &config = sceneConfig.customAttachmentVertexIdx[setIdx];
          sysMat[setIdx].startFrameNum = (int) (config.first * sceneConfig.stepNum);
          sysMat[setIdx].attachments.resize(config.second.size());
          sysMat[setIdx].fixedPoints.resize(config.second.size());
          for (int idx = 0; idx < config.second.size(); idx++) {
            int pIdx = config.second[idx];
            Particle &p = particles[pIdx];
            sysMat[setIdx].fixedPoints[idx] = FixedPoint(p.pos_rest, idx);
            sysMat[setIdx].attachments[idx] = AttachmentSpring(p.idx, &particles, idx, &(sysMat[setIdx].fixedPoints));
          }

           for (AttachmentSpring &a : sysMat[setIdx].attachments) {
            Logging::logColor(std::to_string(a.p1_idx) + ", ", Logging::GREEN);
           }
          std::printf("\n");

          //Add trajectory points
          for (FixedPoint &p : sysMat[setIdx].fixedPoints) {
            sysMat[setIdx].controlPointSplines.emplace_back(p.pos_rest, p.pos_rest, 10, p.idx);
          }
        }

        break;
      }

      default: {
        // do nothing
      }


  }

}

void Simulation::updateCollisionRadii() {
  // calculate particle radii: min length of connected edges * 0.5

  for (Particle &p : particles) {
    std::vector<int> &tris = particleTriangleMap[p.idx];
    Vec3d pos = p.pos_rest;
    double minEdge = 100;
    for (int triIdx : tris) {
      Triangle &t = mesh[triIdx];
      int p2 = t.p0_idx;
      int p3 = t.p1_idx;
      if (p2 == p.idx)
        p2 = t.p2_idx;
      if (p3 == p.idx)
        p3 = t.p2_idx;

      double e1 = (particles[p2].pos_rest - pos).norm();
      double e2 = (particles[p3].pos_rest - pos).norm();
      minEdge = std::min(minEdge, e1);
      minEdge = std::min(minEdge, e2);

    }

    p.radii = minEdge / 2.0 - 0.01;
  }

  bool checkSelfCollisionAtInit = false;


  if (checkSelfCollisionAtInit)
    for (int i = 0; i < particles.size(); i++) {
      for (int j = i + 1; j < particles.size(); j++) {

        Particle &a = particles[i];
        Particle &b = particles[j];
        double collisionThresh = a.radii + b.radii;
//  std::printf("eps: %.5f\n",COLLISION_EPSILON);
        Vec3d posDiff = a.pos - b.pos;

        if (posDiff.norm() < a.radii + b.radii) {
          std::printf("WARNING: self-collision at initialization between particle %d %d\n", a.idx, b.idx);
        }

      }
    }


}

void Simulation::restoreToSingleRecordFromCurrentState() {
  std::pair<VecXd, VecXd> posVelVec = getCurrentPosVelocityVec();

  x_n = posVelVec.first;
  v_n = posVelVec.second;


  VecXd empty(3 * particles.size());
  empty.setZero();
  VecXd x_fixedpoints(3 * sysMat[0].fixedPoints.size());
  for (int i = 0; i < sysMat[0].fixedPoints.size(); i++)
    x_fixedpoints.segment(i * 3, 3) = sysMat[0].fixedPoints[i].pos;


  VecXd x_primitives(3 * primitives.size());
  for (int i = 0; i < primitives.size(); i++)
    x_primitives.segment(i * 3, 3) = primitives[i]->center;


  ForwardInformation singleRecord = {};
  singleRecord.t = 0;
  singleRecord.sysMatId = 0;
  singleRecord.totalRuntime = 0;
  singleRecord.stepIdx = 0;
  singleRecord.converged = true;
  singleRecord.convergeIter = 0;
  singleRecord.cumulateIter = 0;
  singleRecord.windParams.segment(0, 3) = wind * windNorm;
  singleRecord.windParams[3] = windFrequency;
  singleRecord.windParams[4] = windPhase;

  singleRecord.x = x_n;
  singleRecord.v = v_n;
  singleRecord.f = empty;
  singleRecord.x_fixedpoints = x_fixedpoints;
  singleRecord.x_prim = x_primitives;
  singleRecord.avgDeformation = calculateTriangleDeformation(x_n);

  if (!sysMat[0].controlPointSplines.empty())
    singleRecord.splines = sysMat[0].controlPointSplines;

  forwardRecords = {singleRecord};

}

void Simulation::loadSceneMeshes() {
  arrow1.path = std::string(SOURCE_PATH) + "/src/assets/meshes/" + "scene/arrow1.obj";
  arrow1.dim = 3.0;
  arrow1.flipWinding = false;
  arrow2.path = std::string(SOURCE_PATH) + "/src/assets/meshes/" + "scene/arrow2.obj";
  arrow2.dim = 0.3;
  arrow2.flipWinding = false;


  arrow3.path = std::string(SOURCE_PATH) + "/src/assets/meshes/" + "scene/arrow2.obj";
  arrow3.dim = 0.1;
  arrow3.flipWinding = false;
  clip.path = std::string(SOURCE_PATH) + "/src/assets/meshes/" + "scene/clip.obj";
  clip.dim = 0.8;
  clip.flipWinding = false;

  std::vector<FileMesh *> meshes = {&arrow1, &arrow2, &arrow3, &clip};
  for (FileMesh *mesh : meshes) {

    MeshFileHandler::loadOBJFile(mesh->path.c_str(), mesh->points, mesh->triangles);

    Vec3d minDim = mesh->points[0], maxDim = mesh->points[0];
    for (Vec3d &p : mesh->points) {
      for (int i = 0; i < 3; i++) {
        minDim[i] = std::min(minDim[i], p[i]);
        maxDim[i] = std::max(maxDim[i], p[i]);
      }
    }

    Vec3d dim = maxDim - minDim;
    double scale = std::max(std::max(dim[0], dim[1]), dim[2]) / mesh->dim;
    for (Vec3d &p : mesh->points) {
      p = (p - minDim - dim / 2) / scale;
    }

    mesh->normals.resize(mesh->points.size(), Vec3d(0, 0, 0));

    for (Vec3i &t : mesh->triangles) {
      Vec3d normal;
      if (mesh->flipWinding) {
        normal = Triangle::getNormal(mesh->points[t[2]], mesh->points[t[1]], mesh->points[t[0]]);
      } else {
        normal = Triangle::getNormal(mesh->points[t[0]], mesh->points[t[1]], mesh->points[t[2]]);;
      }

      for (int i = 0; i < 3; i++) {
        mesh->normals[t[i]] += normal;
      }
    }

    for (Vec3d &n : mesh->normals) {
      n.normalize();
    }
  }
}

void Simulation::createClothMesh() {

  Triangle::k_stiff = sceneConfig.fabric.k_stiff_stretching;
  TriangleBending::k_stiff = sceneConfig.fabric.k_stiff_bending;
  if (sceneConfig.fabric.isModel) {
    std::string path = std::string(SOURCE_PATH) + "/src/assets/meshes/" + sceneConfig.fabric.name;
    modelPoints.clear();
    modelTris.clear();
    MeshFileHandler::loadOBJFile(path.c_str(), modelPoints, modelTris);
    if (sceneConfig.fabric.custominitPos) {
      int prevParticleSize = modelPoints.size();
      bool isTxt = true;
      if (isTxt)
        modelPoints = MeshFileHandler::loadPosFile_txt(sceneConfig.fabric.initPosFile);
      else {
        std::vector<Vec3i> dummy;
        MeshFileHandler::loadOBJFile(sceneConfig.fabric.initPosFile.c_str(), modelPoints, dummy);
      }
      int newParticleSize = modelPoints.size();
      if (prevParticleSize != newParticleSize) {
        std::printf("WARNING: Custom Mesh Init Pos loading from %s error: mesh vertex num is %d but loaded %d vertex\n",
                    sceneConfig.fabric.initPosFile.c_str(), prevParticleSize, newParticleSize);
      } else {
        std::printf("successfully loaded %d vertex init pos from %s\n", newParticleSize,
                    sceneConfig.fabric.initPosFile.c_str());
      }
    }
    createClothMeshFromModel(modelPoints, modelTris);

  } else {

    createClothMeshFromConfig();
  }


  windFallOff = VecXd(3 * particles.size());
  windFallOff.setOnes();

  createConstraints();
  restoreToSingleRecordFromCurrentState();
  updateAreaMatrix();
  updateMassMatrix();
  initializePrefactoredMatrices();
  updateCollisionRadii();

  // TODO: delete after debug
  double stretchingForce = 0;
  for (Triangle &t : mesh) {
    stretchingForce += t.stretchingForce(x_n).norm();
  }
  std::printf("stretching force after initialization: %.5f\n", stretchingForce);

}

void Simulation::createClothMeshFromConfig() {


  clearConstraintsElementsAndRecords();

  auto createSpring = [=](int a, int b) {
      if ((a < 0) || (b < 0))
        return;
      AssertDebug(a < particles.size());
      AssertDebug(b < particles.size());
      springs.emplace_back(a, b, particles, Spring::k_stiff,
                           (particles[a].pos_rest - particles[b].pos_rest).norm());
  };

  int totalParticles = sceneConfig.fabric.gridNumY * sceneConfig.fabric.gridNumX;
  pointpointConnectionTable.resize(totalParticles, std::vector<bool>(totalParticles, false));

  auto createTriangle = [&](int a, int b, int c) {
      if ((a < 0) || (b < 0) || (c < 0)) {
        return false;
      }
      mesh.emplace_back(c, b, a, particles);
      Vec3i t(a, b, c);
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          pointpointConnectionTable[t[i]][t[j]] = true;

      int triIdx = mesh.size() - 1;
      particleTriangleMap[a].emplace_back(triIdx);
      particleTriangleMap[b].emplace_back(triIdx);
      particleTriangleMap[c].emplace_back(triIdx);

      return true;
  };


  std::vector<Vec3d> particleIn;
  for (int i = 0; i < sceneConfig.fabric.gridNumY; i++) {
    for (int j = 0; j < sceneConfig.fabric.gridNumX; j++) {
      particleIn.emplace_back(getInitParticlePos(i, j));
    }
  }



  if (sceneConfig.fabric.custominitPos) {
    int prevParticleSize = modelPoints.size();
    bool isTxt = true;
    if (isTxt)
      particleIn = MeshFileHandler::loadPosFile_txt(sceneConfig.fabric.initPosFile);
    else {
      std::vector<Vec3i> dummy;
      MeshFileHandler::loadOBJFile(sceneConfig.fabric.initPosFile.c_str(), modelPoints, dummy);
    }
    int newParticleSize = modelPoints.size();
    if (prevParticleSize != newParticleSize) {
      std::printf("WARNING: Custom Mesh Init Pos loading from %s error: mesh vertex num is %d but loaded %d vertex\n",
                  sceneConfig.fabric.initPosFile.c_str(), prevParticleSize, newParticleSize);
    } else {
      std::printf("successfully loaded %d vertex init pos from %s\n", newParticleSize,
                  sceneConfig.fabric.initPosFile.c_str());
    }
  }

  rotatePointsAccordingToConfig(particleIn);

  Vec3d minDim = particleIn[0], maxDim = particleIn[0];
  for (Vec3d &p : particleIn) {
    for (int i = 0; i < 3; i++) {
      minDim[i] = std::min(minDim[i], p[i]);
      maxDim[i] = std::max(maxDim[i], p[i]);
    }
  }

  Vec3d dim = maxDim - minDim;

  if (!sceneConfig.fabric.keepOriginalScalePoint) {
    for (Vec3d &p : particleIn) {
      p -= minDim; // move to cecnter
      p -= dim / 2; // center
    }
  }


  std::printf("p0 is located at %.2f %.2f %.2f\n", particleIn[0][0], particleIn[0][1], particleIn[0][2]);

  restShapeMaxDim = maxDim - minDim;
  restShapeMinDim = Vec3d(0, 0, 0);

  if (!sceneConfig.fabric.keepOriginalScalePoint) {
    restShapeMinDim -= restShapeMaxDim / 2.0;
    restShapeMaxDim -= restShapeMaxDim / 2.0;
  }

  meshInitTransofrmMinDim = minDim;
  meshInitTransofrmScale = 1.0;
  restShapeMidPoint = 0.5 * (maxDim + minDim);


  std::printf("creating mesh from config...\n");
  for (int i = 0; i < sceneConfig.fabric.gridNumY; i++) {
    for (int j = 0; j < sceneConfig.fabric.gridNumX; j++) {
      int id = gridIndicesToParticle(i, j);
      Vec3d mypos = particleIn[id];
      particles.emplace_back(0.0,
                             mypos,
                             mypos,
                             Vec3d(0, 0, 0),
                             Vec2i(i, j), particles.size());
      particleTriangleMap.emplace_back(std::vector<int>());
      assert(particles[particles.size() - 1].idx == particles.size() - 1);
      int thisIdx = gridIndicesToParticle(i, j);
      int leftIdx = gridIndicesToParticle(i, j - 1);
      int rightIdx = gridIndicesToParticle(i, j + 1);
      int downIdx = gridIndicesToParticle(i + 1, j);
      int upIdx = gridIndicesToParticle(i - 1, j);
      int upRightIdx = gridIndicesToParticle(i - 1, j + 1);
      int upLeftIdx = gridIndicesToParticle(i - 1, j - 1);

      createTriangle(thisIdx, upIdx, upRightIdx);
      createTriangle(upIdx, thisIdx, leftIdx);

      AssertDebug(gridIndicesToParticle(i, j) == particles[particles.size() - 1].idx);
    }
  }
  std::printf("creating mesh from config...finished\n");



  // Create fixed points and attachments
  {
    createAttachments(false);



  }




  updateBackwardDefaultInfo(); // resize vectors whose length is parameter-dependent

  std::printf("backwardInfoDefault.dL_dcontrolpoints is sized to %zu\n", backwardInfoDefault.dL_dsplines.size());
  for (int i = 0; i < particles.size(); i++) {
    assert(particles[i].idx == i);
  }
}

void Simulation::updateParticleNormals(const VecXd &x_now) {
  for (Particle &p : particles) {
    p.normal.setZero();

  }
  for (Triangle &t : mesh) {
    Vec3d p0 = x_now.segment(t.p0_idx * 3, 3);
    Vec3d p1 = x_now.segment(t.p1_idx * 3, 3);
    Vec3d p2 = x_now.segment(t.p2_idx * 3, 3);
    t.normal = t.getNormal(p0, p1, p2);

    t.p0()->normal += t.normal;
    t.p1()->normal += t.normal;
    t.p2()->normal += t.normal;
  }


  for (Particle &p : particles) {
    p.normal.normalize();
  }
}

Simulation *
Simulation::createSystem(SceneConfiguration sceneConfig,
                         Vec3d center, bool runBackward) {
  Logging::logWarning("==========================\nCreating system for demo" + sceneConfig.name);
  checkFolderExistsAndCreate(OUTPUT_PARENT_FOLDER);
  Simulation *msSystem = new Simulation(center);
  msSystem->sceneConfig = sceneConfig;
  msSystem->sceneConfig.timeStep = sceneConfig.timeStep;
  msSystem->runBackward = runBackward;
  msSystem->uniqueID = time(NULL);




  msSystem->createClothMesh();
  msSystem->initScene();
  msSystem->restoreToSingleRecordFromCurrentState();


  msSystem->myConfig = {.x0 = msSystem->forwardRecords[0].x, .v0 = msSystem->forwardRecords[0].v, .k_stiff = Triangle::k_stiff};


  return msSystem;

}

void Simulation::resetParticlesAndPrimitivesToRestPose() {
  // reset particles
  for (Particle &p : particles) {
    p.pos = p.pos_init;
    p.velocity = p.velocity_init;

    if (std::isnan(p.pos.norm())) {
      std::printf("WARNING: NAN pos(%.2f,%.2f,%.2f) encountered when resetting particle %d\n", p.pos[0], p.pos[1],
                  p.pos[2], p.idx);
    }
  }

  for (FixedPoint &p: sysMat[0].fixedPoints) {
    p.pos = p.pos_rest;
    if (std::isnan(p.pos.norm())) {
      std::printf("WARNING: NAN pos(%.2f,%.2f,%.2f) encountered when resetting fixed point particle %d\n", p.pos[0],
                  p.pos[1],
                  p.pos[2], p.idx);
    }
  }

  //reset primitives
  for (Primitive *p : primitives) {
    p->reset();
  }
}

void Simulation::resetSystem() {
  // reset particle state and mass matrix
  resetParticlesAndPrimitivesToRestPose();

  // reset record
  restoreToSingleRecordFromCurrentState();
  // reset states
  currentSysmatId = 0;
  perStepGradient.clear();
  perstepTrajectory.clear();
  explosionEncountered = false;
};

void Simulation::appendPerStepGradient(VecXd& x) {
  perStepGradient.emplace_back(x);
}

void Simulation::resetSystem(Vec3d windForce) {
  resetSystem();
  wind = windForce.normalized();
  windNorm = windForce.norm();
};


void Simulation::resetSystem(std::vector<Spline> &controlPoints) {
  resetSystem();
  this->sysMat[0].controlPointSplines = controlPoints;


}

void Simulation::resetSystem(double k_stiff) {
  resetSystem();
  // reset constraints
  Triangle::k_stiff = k_stiff;
  for (Triangle &t : mesh) {
    t.constrainWeightSqrt = std::sqrt(t.area_rest * k_stiff);
  }

  // reset prefactorized matrix
  initializePrefactoredMatrices();


};

void Simulation::setParticlePosVelToVec(const std::pair<VecXd, VecXd> &x0v0) {
  for (Particle &p : particles) {
    p.pos = x0v0.first.segment(p.idx * 3, 3);
    p.velocity = x0v0.second.segment(p.idx * 3, 3);
  }
}

void Simulation::resetSystem(const std::pair<VecXd, VecXd> &x0v0) {
  setParticlePosVelToVec(x0v0);
  restoreToSingleRecordFromCurrentState();
  explosionEncountered = false;

};


void Simulation::updateAreaMatrix() {
  Area = Eigen::SparseMatrix<double>(particles.size() * 3, particles.size() * 3);
  Area.setZero();
  Area_inv = Eigen::SparseMatrix<double>(particles.size() * 3, particles.size() * 3);
  Area_inv.setZero();

  std::vector<Triplet> triplets, tripletsInv;
  std::vector<double> area_per_particles(particles.size(), 0);


  for (Triangle &t  : mesh) {
    double A_i = t.area_rest; // will be distributed to each vertex
    int i0 = t.p0_idx;
    int i1 = t.p1_idx;
    int i2 = t.p2_idx;

    double A_avg = A_i / 3.0;
    area_per_particles[i0] += A_avg;
    area_per_particles[i1] += A_avg;
    area_per_particles[i2] += A_avg;
  }

  for (Particle &p : particles) {
    p.area = area_per_particles[p.idx];
    for (int dim = 0; dim < 3; dim++) {
      int i0 = 3 * p.idx + dim;
      triplets.emplace_back(i0, i0, area_per_particles[p.idx]);
      tripletsInv.emplace_back(i0, i0, 1.0 / area_per_particles[p.idx]);
    }
  }


  Area.setFromTriplets(triplets.begin(), triplets.end());
  Area_inv.setFromTriplets(tripletsInv.begin(), tripletsInv.end());

}

void Simulation::updateMassMatrix() {
  M = Area * sceneConfig.fabric.density; // m = d * A
  M_inv = Area_inv * (1.0 / sceneConfig.fabric.density);
  for (Particle &p  : particles) {
    p.mass = p.area * sceneConfig.fabric.density;

  }

  gravity_n = VecXd(3 * particles.size());
  gravity_n.setZero();
  if (gravityEnabled) {
    for (int i = 0; i < particles.size(); i++)
      gravity_n.segment(3 * i, 3) = gravity;
  }

  bool printMassInfo = false;
  if (printMassInfo) {
    std::vector<std::pair<int, double>> sortedMass;
    for (Particle &p  : particles) {
      sortedMass.emplace_back(p.idx, p.mass);
    }
    std::printf("MassInfo:\n");
    std::sort(sortedMass.begin(), sortedMass.end(),
              [](std::pair<int, double> a, std::pair<int, double> b) { return a.second > b.second; });
    int count = 0;
    for (std::pair<int, double> &p  : sortedMass) {
      std::printf("%d:%.5f ", p.first, p.second);
      count++;
      if (count % 10 == 0)
        std::printf("\n");
    }
    std::printf("\n");
  }

  factorizeDirectSolverLLT(M, Msolver, "Msolver pre factorization");
}


void Simulation::initializePrefactoredMatrices() {

  bool printInitDetails = false;
  std::printf(
          "precompute matrices + prefactorization... density is: %.3f SceneConfigtimestep is :1/%d timeStepIs:%.3f : stiffness is:(%.2f,%.2f,%.2f,%.2f)\n",
          sceneConfig.fabric.density, (int) (1.0 / sceneConfig.timeStep), sceneConfig.timeStep,
          *(k_stiff_arr[0]), *(k_stiff_arr[1]), *(k_stiff_arr[2]), *(k_stiff_arr[3])

  );

  std::printf("precompute matrices + prefactorization...");
  for (int sysMatId = 0; sysMatId < sysMat.size(); sysMatId++) {
    std::vector<Triplet> triplets;
    std::vector<std::vector<Triplet>> triplets_pertype;

    sysMat[sysMatId].constraintNum = 0;
    for (int i = 0; i < Constraint::CONSTRAINT_NUM; i++) {
      sysMat[sysMatId].constraintNum_pertype[i] = 0;
      triplets_pertype.emplace_back(std::vector<Triplet>());
      triplets_pertype[i].reserve(5);

    }
    for (int i = 0; i < sysMat[sysMatId].constraints.size(); i++) {
      Constraint *s = sysMat[sysMatId].constraints[i];

      s->addConstraint(triplets, sysMat[sysMatId].constraintNum, true);
      s->addConstraint(triplets_pertype[s->constraintType],
                       sysMat[sysMatId].constraintNum_pertype[s->constraintType], false);
    }



      for (int i = 0; i < Constraint::CONSTRAINT_NUM; i++) {
        if (sysMatId == 0) {
          projections.setZero(sysMat[sysMatId].constraintNum, 1);
          projections_pertype[i] = VecXd(sysMat[sysMatId].constraintNum_pertype[i]);
          projections_pertype[i].setZero();
        }

        sysMat[sysMatId].A_pertype[i] = SpMat(sysMat[sysMatId].constraintNum_pertype[i], 3 * particles.size());
        sysMat[sysMatId].A_pertype[i].setFromTriplets(triplets_pertype[i].begin(), triplets_pertype[i].end());
        sysMat[sysMatId].A_t_pertype[i] = sysMat[sysMatId].A_pertype[i].transpose();
        sysMat[sysMatId].A_t_times_A_pertype[i] = sysMat[sysMatId].A_t_pertype[i] * sysMat[sysMatId].A_pertype[i];
      }

      dproj_dxnew = SpMat(sysMat[sysMatId].constraintNum, 3 * particles.size());
      dproj_dxnew.setZero();
      sysMat[sysMatId].A = SpMat(sysMat[sysMatId].constraintNum, 3 * particles.size());


      I = SpMat(3 * particles.size(), 3 * particles.size());
      I.setIdentity();
      sysMat[sysMatId].A.setFromTriplets(triplets.begin(), triplets.end());


      sysMat[sysMatId].A = sysMat[sysMatId].A.pruned();

      sysMat[sysMatId].A_t = sysMat[sysMatId].A.transpose(); // 3m x constrainNum
      sysMat[sysMatId].C = sysMat[sysMatId].A_t * sysMat[sysMatId].A * (sceneConfig.timeStep * sceneConfig.timeStep);
      sysMat[sysMatId].C = sysMat[sysMatId].C.pruned(1e-15);
      sysMat[sysMatId].C_t = sysMat[sysMatId].C.transpose();
      sysMat[sysMatId].P = sysMat[sysMatId].C + M;
      sysMat[sysMatId].P = factorizeDirectSolverLLT(sysMat[sysMatId].P, sysMat[sysMatId].solver,
                                                    "Msolver pre factorization");


      if (runBackward) {
        MatXd dp_dfixedpos = MatXd(sysMat[sysMatId].constraintNum, 3 * sysMat[sysMatId].fixedPoints.size());

        dp_dfixedpos.setZero();

        for (AttachmentSpring &s: sysMat[sysMatId].attachments) {
          dp_dfixedpos.block<3, 3>(s.c_idx, s.pfixed_idx * 3) += s.dp_dfixedPose();
        }

        SpMat dp_dfixedpossparse = dp_dfixedpos.sparseView();
        dp_dfixedpossparse = dp_dfixedpossparse.pruned();
        sysMat[sysMatId].A_t_dp_dxfixed = (sysMat[sysMatId].A_t * dp_dfixedpossparse);

      }

      std::printf("\n");





  }


}

void
Simulation::loadWindSim2RealAnimationSequence(std::string folderName, std::vector<std::string> files,
                                              bool isRyanWhite) {

  std::printf("isRyanWhite: %s\n", isRyanWhite ? "True" : "False");
  sceneConfig.attachmentPoints = CUSTOM_ARRAY;
  std::vector<std::pair<double, std::vector<int>>> ryanWhiteClipIndices =
          {{0, {10, 11, 12, 13, 14, 510, 26, 27, 28, 29, 30, 460, 461, 462, 463, 464, 495}}};
  std::vector<int> row2 = {496, 497, 459, 458, 457, 456, 31, 32, 24, 23, 22, 21, 511, 512, 9, 8, 7, 6};
  ryanWhiteClipIndices[0].second.insert(ryanWhiteClipIndices[0].second.end(), row2.begin(), row2.end());

  sceneConfig.customAttachmentVertexIdx = {{0.0, {2, 3}}};
  if (isRyanWhite) {
    sceneConfig.customAttachmentVertexIdx = ryanWhiteClipIndices;
    sceneConfig.fabric.clothDimX = sceneConfig.fabric.clothDimY = 11;
  }

  sceneConfig.trajectory = TrajectoryConfigs::FIXED_POINT_TRAJECTORY;
  sceneConfig.primitiveConfig = PrimitiveConfiguration::NONE;
  sceneConfig.orientation = Orientation::FRONT;
  sceneConfig.fabric.density = 0.15;
  sceneConfig.fabric.k_stiff_stretching = 200;
  sceneConfig.fabric.k_stiff_bending = 0.01;
  sceneConfig.primitiveConfig = NONE;
  sceneConfig.fabric.color = COLOR_IBM_ORANGE40;
  sceneConfig.camPos = Vec3d(0.602, 5.867, 15.435);
  sceneConfig.camFocusPos = Vec3d(1.386, -0.535, -1.019);
  sceneConfig.camFocusPointType = CameraFocusPointType::POINT;
  sceneConfig.windConfig = WindConfig::NO_WIND;
  sceneConfig.name = "windsim2real_" + folderName;


  Triangle::k_stiff = sceneConfig.fabric.k_stiff_stretching;
  TriangleBending::k_stiff = sceneConfig.fabric.k_stiff_bending;
  windFallOff = VecXd(3 * particles.size());
  windFallOff.setOnes();
  Simulation::sceneConfig.timeStep = isRyanWhite ? 1 / 120.0 : 1 / 60.0;
  wind = Vec3d(1, 0.1, 1.0).normalized() * 0.1;

  std::string fullFolderPath = std::string(SOURCE_PATH) + "/src/assets/animation/" + folderName + "/";

  modelPoints.clear();
  modelTris.clear();
  std::string firstFilefName = fullFolderPath + files[0];
  MeshFileHandler::loadOBJFile(firstFilefName.c_str(), modelPoints, modelTris);

  if (!isRyanWhite) {
    std::printf("flipping yz data for arcsim\n");
    // for arcsim data, their y and z axis needs to be flipped:
    for (Vec3d &v : modelPoints) {
      double zOriginal = v[2];
      v[2] = v[1];
      v[1] = zOriginal;
    }
  }

  sceneConfig.fabric.isModel = true;
  sceneConfig.fabric.keepOriginalScalePoint = false;
  sceneConfig.fabric.name = firstFilefName;

  createClothMeshFromModel(modelPoints, modelTris);
  windFallOff = VecXd(3 * particles.size());
  windFallOff.setOnes();

  Vec3d windFocusPoint = Vec3d(0, -1, 0);
  for (int i = 0; i < particles.size(); i++) {
    double distSquared = (windFocusPoint - particles[i].pos).norm();

    windFallOff.segment(i * 3, 3) = Vec3d::Ones() * std::min(1.0 / distSquared, 1.0);
  }


  createConstraints();
  restoreToSingleRecordFromCurrentState();
  updateAreaMatrix();
  updateMassMatrix();
  initializePrefactoredMatrices();
  updateCollisionRadii();
  initScene();
  VecXd x_i(3 * particles.size()), empty(3 * particles.size());
  empty.setZero();
  for (int i = 1; i < files.size(); i++) {
    std::string fName = fullFolderPath + files[i];
    modelPoints.clear();
    modelTris.clear();
    MeshFileHandler::loadOBJFile(fName.c_str(), modelPoints, modelTris);
    if (!isRyanWhite) {
      for (Vec3d &v : modelPoints) {
        double zOriginal = v[2];
        v[2] = v[1];
        v[1] = zOriginal;
      }
    }

    for (int pIdx = 0; pIdx < particles.size(); pIdx++) {
      x_i.segment(pIdx * 3, 3) =
              (modelPoints[pIdx] - meshInitTransofrmMinDim) / meshInitTransofrmScale - restShapeMaxDim;
    }
    ForwardInformation record = {};
    record.t = sceneConfig.timeStep * i;
    record.x = x_i;
    record.v = (x_i - forwardRecords[forwardRecords.size() - 1].x) / sceneConfig.timeStep;
    record.f = empty;
    record.f_prim = VecXd(0);
    record.x_prim = VecXd(0);
    record.v_prim = VecXd(0);
    record.stepIdx = i;
    record.x_fixedpoints = VecXd(3 * sysMat[0].fixedPoints.size());
    VecXd clipPosition(3 * sysMat[0].fixedPoints.size());
    clipPosition.setZero();
    for (AttachmentSpring &s: sysMat[0].attachments) {
      record.x_fixedpoints.segment(3 * s.pfixed_idx, 3) = record.x.segment(3 * s.p1_idx, 3);
    }

    forwardRecords.emplace_back(record);

  }


  if (isRyanWhite) { // interpolate data
    for (int pass = 0; pass < 2; pass++) {
      for (int i = forwardRecords.size() - 1; i >= 1; i--) {
        ForwardInformation newRecord = forwardRecords[i];
        newRecord.x = (forwardRecords[i].x + forwardRecords[i - 1].x) * 0.5;
        newRecord.x_fixedpoints = (forwardRecords[i].x_fixedpoints + forwardRecords[i - 1].x_fixedpoints) * 0.5;
        forwardRecords.insert(forwardRecords.begin() + i, newRecord);

      }
    }

    for (int i = 1; i < forwardRecords.size(); i++) {
      forwardRecords[i].v = (forwardRecords[i].x - forwardRecords[i - 1].x) / sceneConfig.timeStep;
      forwardRecords[i].stepIdx = i;
    }

  }


  fixedPointTrajectory.clear();
  for (ForwardInformation &forwardInfo : forwardRecords) {
    fixedPointTrajectory.emplace_back(forwardInfo.x_fixedpoints);
  }


//      VecXd v_init = (forwardRecords[1].x - forwardRecords[0].x) / sceneConfig.timeStep;
//
//      for (int i = 0; i < particles.size(); i++) {
//        particles[i].velocity_init = v_init.segment(i * 3, 3);
//      }
  std::printf("finished loading animation. forwardRecords now have %zu records\n", forwardRecords.size());

  groundTruthForwardRecords = forwardRecords;
  perstepWindFactor = VecXd(forwardRecords.size());
  perstepWindFactor.setOnes();


}

Simulation::SelfCollisionInformation
Simulation::isSelfCollision(const Triangle &a, const Particle &p3, const Vec3d v_a, const Vec3d v_b) const {
  SelfCollisionInformation info{};
  info.collides = false;
  double COLLISION_EPSILON = 0.4;

  Particle *p0 = a.p0();
  Particle *p1 = a.p1();
  Particle *p2 = a.p2();

  Vec3d x1 = p0->pos - p3.pos;
  Vec3d x2 = p1->pos - p3.pos;
  Vec3d x3 = p2->pos - p3.pos;
  Vec3d v = v_a - v_b;

  return info;
}

double
Simulation::calculateLossAndGradient(LossType &lossType, Simulation::LossInfo &lossInfo, VecXd &dL_dx, VecXd &dL_dv,
                                     int idx, bool calculateLoss) {
  double L = 0;
  int lastIdx = ((int) forwardRecords.size()) - 1;
  int N = forwardRecords.size();
  dL_dx.setZero();
  dL_dv.setZero();

  switch (lossType) {
    case (MATCH_TRAJECTORY): {
      if (forwardRecords.size() != lossInfo.targetSimulation.size()) {
        std::printf(
                "WARNING: calculate trajectory loss frame number mismatch, but records has size %zu while target has size %zu\n",
                forwardRecords.size(), lossInfo.targetSimulation.size());
      } else {
//            std::printf("%d ForwardRecordSz: %zu target: %zu calcLoss?: %d\n",idx,  forwardRecords.size(), lossInfo.targetSimulation.size(), calculateLoss);
      }

      if (forwardRecords[idx].x.rows() != lossInfo.targetSimulation[idx].x.rows()) {
        std::printf(
                "WARNING: calculate trajectory loss dimension mismatch, but records has size %zu while target has size %zu\n",
                forwardRecords[idx].x.rows(), lossInfo.targetSimulation[idx].x.rows());
      }
      double k_constant = 1.0 / (N * ((int) particles.size()));
      if (calculateLoss) {
        for (int i = 0; i < forwardRecords.size(); i++) {
          L += k_constant * (forwardRecords[i].x - lossInfo.targetSimulation[i].x).squaredNorm();
          if (std::isnan(L)) {
            std::printf(
                    "WARNING: Loss becomes NAN after adding subloss from %d in MATCH_TRAJECTORY; record:%.3f target:%.3f\n",
                    i, forwardRecords[i].x.norm(), lossInfo.targetSimulation[i].x.norm());
          }
        }

        dL_dx = k_constant * 2 * (forwardRecords[lastIdx].x - lossInfo.targetSimulation[lastIdx].x);
      } else {
        dL_dx = k_constant * 2 * (forwardRecords[idx].x - lossInfo.targetSimulation[idx].x);
      }

      break;
    }

    case (MATCH_VELOCITY): {
      if (forwardRecords.size() != lossInfo.targetSimulation.size()) {
        std::printf(
                "WARNING: calculate trajectory loss frame number mismatch, but records has size %zu while target has size %zu\n",
                forwardRecords.size(), lossInfo.targetSimulation.size());
      } else {
//            std::printf("%d ForwardRecordSz: %zu target: %zu calcLoss?: %d\n",idx,  forwardRecords.size(), lossInfo.targetSimulation.size(), calculateLoss);
      }

      if (forwardRecords[idx].v.rows() != lossInfo.targetSimulation[idx].v.rows()) {
        std::printf(
                "WARNING: calculate trajectory loss dimension mismatch, but records has size %zu while target has size %zu\n",
                forwardRecords[idx].v.rows(), lossInfo.targetSimulation[idx].v.rows());
      }
      double k_constant = 1.0 / (N * ((int) particles.size()));
      if (calculateLoss) {
        for (int i = 0; i < forwardRecords.size(); i++) {
          L += k_constant * (forwardRecords[i].v - lossInfo.targetSimulation[i].v).squaredNorm();
          if (std::isnan(L)) {
            std::printf(
                    "WARNING: Loss becomes NAN after adding subloss from %d in MATCH_TRAJECTORY; record:%.3f target:%.3f\n",
                    i, forwardRecords[i].v.norm(), lossInfo.targetSimulation[i].v.norm());
          }
        }

        dL_dv = k_constant * 2 * (forwardRecords[lastIdx].v - lossInfo.targetSimulation[lastIdx].v);
      } else {
        dL_dv = k_constant * 2 * (forwardRecords[idx].v - lossInfo.targetSimulation[idx].v);
      }

      break;
    }

    case (MATCH_TRAJECTORY_MAX): {
      if (forwardRecords.size() != lossInfo.targetSimulation.size()) {
        std::printf(
                "WARNING: calculate trajectory loss frame number mismatch, but records has size %zu while target has size %zu\n",
                forwardRecords.size(), lossInfo.targetSimulation.size());
      } else {
//            std::printf("%d ForwardRecordSz: %zu target: %zu calcLoss?: %d\n",idx,  forwardRecords.size(), lossInfo.targetSimulation.size(), calculateLoss);
      }

      if (forwardRecords[idx].x.rows() != lossInfo.targetSimulation[idx].x.rows()) {
        std::printf(
                "WARNING: calculate trajectory loss dimension mismatch, but records has size %zu while target has size %zu\n",
                forwardRecords[idx].x.rows(), lossInfo.targetSimulation[idx].x.rows());
      }
      double k_constant = 1.0 / (((int) particles.size()));
      int maxFrameId = 0;
      for (int i = 0; i < forwardRecords.size(); i++) {
        int lossFrame = k_constant * (forwardRecords[i].x - lossInfo.targetSimulation[i].x).squaredNorm();
        if (lossFrame > L) {
          L = lossFrame;
          maxFrameId = i;
        }
      }
      if (idx == maxFrameId) {
        dL_dx = k_constant * 2 * (forwardRecords[idx].x - lossInfo.targetSimulation[idx].x);
      }

      break;
    }

    case (DRESS_ANGLE): {
      double PI = 3.1415936535;

//      double targetAngleRadian = lossInfo.targetDressAngle / 180.0 * PI;
      Vec3d refPoint = restShapeMidPoint;
      refPoint[1] = restShapeMaxDim[1];

      double targetHeight =
              restShapeMinDim[1] + (restShapeMaxDim[1] - restShapeMinDim[1]) * lossInfo.targetTwirlHeight;


      double k = 1.0 / lossInfo.loopPoints.size();

      for (int pIdx : lossInfo.loopPoints) {
        Vec3d pos = forwardRecords[lastIdx].x.segment(pIdx * 3, 3);
        double penaltyDir = (targetHeight > pos[1]) ? 1.0 : -1.0;
        L += (pos[1] - targetHeight) * (pos[1] - targetHeight) * k;
        if (idx == lastIdx) {
          dL_dx[pIdx * 3 + 1] += 2 * (pos[1] - targetHeight) * k;
        }

        /*
         * Loss function definition at submission
        L += (targetHeight - pos[1]) * penaltyDir * k;
        if (idx == lastIdx) {
          dL_dx[pIdx * 3 + 1] += -penaltyDir * k;
        }

         */
      }


      break;
    }

    case (MATCHSHAPE_TRANSLATION_INVARINT): {
      //tranlation-invariant shape matching
      VecXd targetShape = lossInfo.targetShape;
      VecXd curShapeTranslationFree = forwardRecords[lastIdx].x;
      for (int i = 0; i < particles.size(); i++) {
        targetShape.segment(i * 3, 3) -= targetShape.segment(0, 3);
        curShapeTranslationFree.segment(i * 3, 3) -= curShapeTranslationFree.segment(0, 3);
      }

      int numParticles = particles.size();
      L += 1.0 / numParticles * (curShapeTranslationFree - targetShape).squaredNorm();

      if (idx == lastIdx) {
        dL_dx = 2 * (curShapeTranslationFree - targetShape);
        dL_dx.segment(0, 3).setZero();
        for (int i = 1; i < particles.size(); i++) {
          dL_dx.segment(0, 3) += dL_dx.segment(i * 3, 3);
        }

        dL_dx *= 1.0 / numParticles;
      }


      break;
    }

    case (MULTISTEP_MATCHSHAPE): {
      for (std::pair<int, VecXd> &frameIdAndShape : lossInfo.targetFrameShape) {
        int frameIdx = frameIdAndShape.first;

        VecXd &x_target = frameIdAndShape.second;
        VecXd &x_current = forwardRecords[frameIdx].x;
        int n_particles = particles.size();

        if (calculateLoss) {
          L += (x_current - x_target).squaredNorm() / n_particles;
        }
        if (idx == frameIdx) {
          dL_dx = 2 * (x_current - x_target) / n_particles;
        }
      }
      break;
    }

    case (MATCHSHAPE_WITH_TRANSLATION): {
      //tranlation-invariant shape matching
      Vec3d targetTranslation = lossInfo.targetTranslation;
      VecXd x_target(3 * particles.size());
      for (const Particle &p : particles) {
        x_target.segment(3 * p.idx, 3) = p.pos_init + targetTranslation;
      }
      VecXd &x_current = forwardRecords[lastIdx].x;
      int n_particles = particles.size();
      if (calculateLoss) {
        L += (x_current - x_target).squaredNorm() / n_particles;
      }
      if (idx == lastIdx) {
        dL_dx = 2 * (x_current - x_target) / n_particles;
      }
      break;
    }

    case (ASSISTED_DRESSING_KEYPOINTS): {
      int totalKeyPoints = lossInfo.targetPosPairs.size();
      {
        // Point correspondence loss
        Vec3d targetTranslation = lossInfo.targetTranslation;
        VecXd targetShape(3 * particles.size());
        for (CorresPondenceTargetInfo &pIdxTargetLocPair: lossInfo.targetPosPairs) {
          VecXd &curShape = forwardRecords[pIdxTargetLocPair.frameIdx].x;
          Vec3d targetPos = pIdxTargetLocPair.targetPos;
          int maxDistpIdx = pIdxTargetLocPair.particleIndices[0];
          Vec3d maxDistPos = curShape.segment(maxDistpIdx * 3, 3);
          double maxDist = (maxDistPos - targetPos).squaredNorm();
          for (int particleIndice : pIdxTargetLocPair.particleIndices) {
            Vec3d currentPos = curShape.segment(particleIndice * 3, 3);
            double dist = (currentPos - targetPos).squaredNorm();
            if (dist > maxDist) {
              maxDistPos = currentPos;
              maxDist = dist;
              maxDistpIdx = particleIndice;
            }
          }

          if (calculateLoss) {
            L += maxDist / totalKeyPoints;
          }
          if (pIdxTargetLocPair.frameIdx == idx) {
            dL_dx.segment(maxDistpIdx * 3, 3) = 2 * (maxDistPos - targetPos) / totalKeyPoints;
          }

        }


      }

      break;
    }

  }

  if (calculateLoss) {
    if (forwardRecords.size() >= 2) {
      dL_dx += dL_dv * 1 / sceneConfig.timeStep;
    }
  }


  return L;

}

void
Simulation::resetSystemWithParams(Simulation::BackwardTaskInformation &taskConfiguration,
                                  Simulation::ParamInfo &param) {
  bool PmatrixChanged = false;
  bool constraintChanged = false;
  for (int i = 0; i < Constraint::CONSTRAINT_NUM; i++) {
    if (taskConfiguration.dL_dk_pertype[i]) {
      constraintChanged = true;
      *(k_stiff_arr[i]) = param.k_pertype[i];
    }

  }

  if (constraintChanged) {
    for (SystemMatrix &sys : sysMat) {
      for (Constraint *c: sys.constraints) {
        c->setConstraintWeight();
      }
    }

  }
  if (taskConfiguration.dL_dfext) {
    Simulation::wind = param.f_ext.normalized();
    Simulation::windNorm = param.f_ext.norm();
  }

  if (taskConfiguration.dL_dfwind) {
    Vec3d windForce = param.f_extwind.segment(0, 3);
    Simulation::wind = windForce.normalized();
    Simulation::windNorm = windForce.norm();
    Simulation::windFrequency = param.f_extwind[3];
    Simulation::windPhase = param.f_extwind[4];
  }

  if (taskConfiguration.dL_dconstantForceField) {
    external_force_field = param.f_constantForceField;
  }

  if (taskConfiguration.dL_dwindFactor) {
    perstepWindFactor = param.f_ext_timestep;
  }


  if (taskConfiguration.dL_dcontrolPoints) {
    for (int sysMatId = 0; sysMatId < sysMat.size(); sysMatId++) {
      sysMat[sysMatId].controlPointSplines = param.controlPointSplines[sysMatId];
    }
  }


  if (taskConfiguration.dL_dmu) {
    for (std::pair<int, double> &primInfo : param.mu) {
      primitives[primInfo.first]->mu = primInfo.second;
    }
  }


  if (taskConfiguration.dL_density) {
    sceneConfig.fabric.density = param.density;
    updateMassMatrix();
    PmatrixChanged = true;
  }


  PmatrixChanged = constraintChanged || PmatrixChanged;
  if (PmatrixChanged) {
    std::printf("Factorize System Matrix...");
    initializePrefactoredMatrices();
  }

//  std::printf("final resetting...\n");
  resetSystem();


  if (taskConfiguration.dL_dx0) { // for rest shape param, set it after all resets have been finished
    forwardRecords[0].x = param.x0;
    for (Particle &p : particles) {
      p.pos = param.x0.segment(p.idx * 3, 3);
    }
  }
  if (std::isnan(forwardRecords[0].x.norm()) || std::isnan(forwardRecords[0].v.norm())) {
    std::printf("WARNING: NAN encountered after reset: x: %.4f v: %.4f\n", forwardRecords[0].x.norm(),
                forwardRecords[0].v.norm());
  }

  // reset record
  restoreToSingleRecordFromCurrentState();
  // reset states
  currentSysmatId = 0;
  perStepGradient.clear();
  perstepTrajectory.clear();

  explosionEncountered = false;

}

void
Simulation::calculateFiniteDiffLossArr(Simulation::ParamInfo paramPlus, Simulation::ParamInfo paramMinus, double delta,
                                       std::vector<ForwardInformation> &centerForward, std::vector<double> &L_arr,
                                       Simulation::BackwardTaskInformation taskConfiguration, LossType lossType,
                                       Simulation::LossInfo &lossInfo, int FORWARD_STEPS,
                                       const std::function<void(const std::string &)> &setTextBoxCB) {
  VecXd dL_dlastx(3 * particles.size());
  VecXd dL_dlastv(3 * particles.size());
  std::vector<double> L_plus(FORWARD_STEPS + 1, 0), L_minus(FORWARD_STEPS + 1, 0);
  for (int startSepIdx = 0; startSepIdx < FORWARD_STEPS; startSepIdx++) {
    L_arr[startSepIdx] = 0;
  }
  resetSystemWithParams(taskConfiguration, paramPlus);
  int idxSpace = 200; // since full backward records require FORWARD_STEPS^2 steps simulation, we calculate records with spacing instead.
  int startSepIdx = 0;

  while (startSepIdx < FORWARD_STEPS) {

    forwardRecords = centerForward;
    forwardRecords.resize(startSepIdx + 1);
    setParticlePosVelToVec(std::make_pair(forwardRecords[startSepIdx].x, forwardRecords[startSepIdx].v));

    for (int id = startSepIdx; id < FORWARD_STEPS; id++) step();
    L_plus[startSepIdx] = (calculateLossAndGradient(lossType, lossInfo, dL_dlastx, dL_dlastv, 0, true));

    startSepIdx += 5;
//        if (startSepIdx < idxSpace - 100) {
//          startSepIdx += idxSpace;
//        } else {
//          startSepIdx += 15;
//        }
  }

  resetSystemWithParams(taskConfiguration, paramMinus);
  startSepIdx = 0;
  while (startSepIdx < FORWARD_STEPS) {
    forwardRecords = centerForward;
    forwardRecords.resize(startSepIdx + 1);
    setParticlePosVelToVec(std::make_pair(forwardRecords[startSepIdx].x, forwardRecords[startSepIdx].v));

    for (int id = startSepIdx; id < FORWARD_STEPS; id++) step();
    L_minus[startSepIdx] = calculateLossAndGradient(lossType, lossInfo, dL_dlastx, dL_dlastv, 0, true);

    startSepIdx += idxSpace;
  }

  for (int startSepIdx = 0; startSepIdx < FORWARD_STEPS; startSepIdx++) {
    L_arr[startSepIdx] = (L_plus[startSepIdx] - L_minus[startSepIdx]) / (2 * delta);

  }


}

std::vector<Simulation::BackwardInformation>
Simulation::finiteDifferenceBackward(Simulation::BackwardTaskInformation taskConfiguration, LossType lossType,
                                     Simulation::LossInfo &lossInfo, int FORWARD_STEPS, Simulation::ParamInfo guess,
                                     const std::function<void(const std::string &)> &setTextBoxCB) {
  BackwardInformation gradient = backwardInfoDefault;
  std::vector<BackwardInformation> ret;
  resetSystemWithParams(taskConfiguration, guess);
  ParamInfo paramPlus = guess, paramMinus = guess;
  double delta = 1e-6;
  VecXd dL_dlastx(3 * particles.size());
  VecXd dL_dlastv(3 * particles.size());
  VecXd empty(3 * particles.size());
  empty.setZero();
  std::vector<double> L;

  for (int i = 0; i < FORWARD_STEPS; i++) {
    step();
    ret.emplace_back(gradient);
    L.emplace_back(0);
  }
  double forward_thresholdBefore = Simulation::forwardConvergenceThreshold;
  Simulation::forwardConvergenceThreshold = 1e-9;
  std::vector<ForwardInformation> centerForward = forwardRecords;
  for (int constraintTypeIdx = 0; constraintTypeIdx < Constraint::CONSTRAINT_NUM; constraintTypeIdx++) {

    if (taskConfiguration.dL_dk_pertype[constraintTypeIdx]) {
      setTextBoxCB("fd dL_dk-" + std::to_string(constraintTypeIdx) + "\n");
      paramPlus = paramMinus = guess;
      double k_center = guess.k_pertype[constraintTypeIdx];
      paramPlus.k_pertype[constraintTypeIdx] = k_center + delta;
      paramMinus.k_pertype[constraintTypeIdx] = k_center - delta;
      calculateFiniteDiffLossArr(paramPlus, paramMinus, delta, centerForward, L, taskConfiguration, lossType,
                                 lossInfo,
                                 FORWARD_STEPS, setTextBoxCB);
      for (int stepIdx = 0; stepIdx < FORWARD_STEPS; stepIdx++) {
        ret[stepIdx].dL_dk_pertype[constraintTypeIdx] = L[stepIdx];
      }
    }
  }

  if (taskConfiguration.dL_dfext) {
    Vec3d f_center = guess.f_ext;

    for (int dim = 0; dim < 3; dim++) {
      setTextBoxCB("fd dL_dfext[" + std::to_string(dim) + "]\n");
      paramPlus = paramMinus = guess;
      paramPlus.f_ext = f_center;
      paramPlus.f_ext[dim] += delta;
      paramMinus.f_ext = f_center;
      paramMinus.f_ext[dim] -= delta;

      calculateFiniteDiffLossArr(paramPlus, paramMinus, delta, centerForward, L, taskConfiguration, lossType,
                                 lossInfo,
                                 FORWARD_STEPS, setTextBoxCB);

      for (int stepIdx = 0; stepIdx < FORWARD_STEPS; stepIdx++) {
        ret[stepIdx].dL_dfext[dim] = L[stepIdx];
      }
    }
  }

  if (taskConfiguration.dL_dfwind) {
    Vec5d f_center = guess.f_extwind;

    for (int dim = 0; dim < 5; dim++) {
      setTextBoxCB("fd dL_dwind[" + std::to_string(dim) + "]\n");
      paramPlus = paramMinus = guess;
      paramPlus.f_extwind = f_center;
      paramPlus.f_extwind[dim] += delta;
      paramMinus.f_extwind = f_center;
      paramMinus.f_extwind[dim] -= delta;

      calculateFiniteDiffLossArr(paramPlus, paramMinus, delta, centerForward, L, taskConfiguration, lossType,
                                 lossInfo,
                                 FORWARD_STEPS, setTextBoxCB);

      for (int stepIdx = 0; stepIdx < FORWARD_STEPS; stepIdx++) {
        ret[stepIdx].dL_dwind[dim] = L[stepIdx];
      }
    }
  }
  if (taskConfiguration.dL_dcontrolPoints) {
    for (int setidx = guess.controlPointSplines.size() - 1; setidx < guess.controlPointSplines.size(); setidx++) {
      std::vector<Spline> spline_center = guess.controlPointSplines[setidx];
      for (int splineIdx = 0; splineIdx < spline_center.size(); splineIdx++) {
        Spline &s = spline_center[splineIdx];
        int splineParameterNumber = s.getParameterNumber();

        std::printf("spline %d has parameter number %d\n", splineIdx, splineParameterNumber);
        for (int dim = 0; dim < splineParameterNumber; dim++) {
          setTextBoxCB("fd dL_dspline-" + std::to_string(splineIdx) + "dim-" + std::to_string(dim) + "\n");
          paramPlus = paramMinus = guess;
          paramPlus.controlPointSplines[setidx] = spline_center;
          paramMinus.controlPointSplines[setidx] = spline_center;
          VecXd deltaStep(splineParameterNumber);
          std::printf("deltaStep has dimension %zu\n", deltaStep.rows());

          deltaStep.setZero();
          deltaStep[dim] = delta;

          paramPlus.controlPointSplines[setidx][splineIdx].updateControlPoints(deltaStep);
          paramMinus.controlPointSplines[setidx][splineIdx].updateControlPoints(-deltaStep);

          calculateFiniteDiffLossArr(paramPlus, paramMinus, delta, centerForward, L, taskConfiguration, lossType,
                                     lossInfo,
                                     FORWARD_STEPS, setTextBoxCB);

          int paramIdxOffset = Spline::getParameterNumberBeforeIdx(sysMat[0].controlPointSplines, splineIdx);
          std::printf("goes into offset %d\n", paramIdxOffset);
          for (int stepIdx = 0; stepIdx < FORWARD_STEPS; stepIdx++) {
            ret[stepIdx].dL_dsplines[setidx][splineIdx][dim] = L[stepIdx];
          }
        }
      }
    }


  }

  if (taskConfiguration.dL_density) {
    paramPlus = paramMinus = guess;
    paramPlus.density += delta;
    paramMinus.density -= delta;
    calculateFiniteDiffLossArr(paramPlus, paramMinus, delta, centerForward, L, taskConfiguration, lossType,
                               lossInfo,
                               FORWARD_STEPS, setTextBoxCB);
    for (int stepIdx = 0; stepIdx < FORWARD_STEPS; stepIdx++) {
      ret[stepIdx].dL_ddensity = L[stepIdx];
    }
  }


  if (taskConfiguration.dL_dmu) {
    paramPlus = paramMinus = guess;
    paramPlus.mu[0].second += delta;
    paramMinus.mu[0].second -= delta;
    calculateFiniteDiffLossArr(paramPlus, paramMinus, delta, centerForward, L, taskConfiguration, lossType,
                               lossInfo,
                               FORWARD_STEPS, setTextBoxCB);
    for (int stepIdx = 0; stepIdx < FORWARD_STEPS; stepIdx++) {
      ret[stepIdx].dL_dmu[0].second = L[stepIdx];
    }
  }

  Simulation::forwardConvergenceThreshold = forward_thresholdBefore;
  return ret;
}

void Simulation::exportCurrentSimulation(std::string fileName) {
  std::printf("begin to save simulation [exportCurrentSimulation]..");
  std::fflush(stdout);
  std::string parentFolder = std::string(SOURCE_PATH) + "/output/" + fileName + "/";
  checkFolderExistsAndCreate(std::string(SOURCE_PATH) + "/output/");
  checkFolderExistsAndCreate(parentFolder);

  std::string areaTotalStr = "";


  for (int i = 0; i < forwardRecords.size(); i++) {
    double areaTotal = 0.0;
    std::string subFolder = fileName + "/" + std::to_string(i) + "/";
    std::string fullFolder = OUTPUT_PARENT_FOLDER + subFolder;
    checkFolderExistsAndCreate(fullFolder);


    MeshFileHandler::saveOBJFile(subFolder + "0-CLOTH", forwardRecords[i].x,
                                 getParticleNormals(mesh, forwardRecords[i].x), mesh);

    for (Triangle &t : mesh) {
      areaTotal += t.getArea(forwardRecords[i].x);
    }


    areaTotalStr += "Frame " + std::to_string(i) + ":" + d2str(areaTotal, 7) + "\n";


    for (int primId = 0; primId < primitives.size(); primId++) {
      Primitive *prim = primitives[primId];
      VecXd primPos;
      primPos.setZero();
      std::vector<Triangle> primMesh;
      std::string name = Primitive::primitiveTypeStrings[prim->type];
      prim->getMesh(primMesh, primPos, Vec3d(0, 0, 0), i);
      MeshFileHandler::saveOBJFile(subFolder + std::to_string(primId + 1) + "-" + name, primPos,
                                   getParticleNormals(primMesh, primPos), primMesh);
    }

    for (int fixedPointId = 0; fixedPointId < forwardRecords[i].x_fixedpoints.rows() / 3; fixedPointId++) {
      Vec3d clipCenter = forwardRecords[i].x_fixedpoints.segment(fixedPointId * 3, 3);
      std::vector<Vec3i> primMesh;
      VecXd primPos;
      clip.getMesh(primMesh, primPos, clipCenter);
      MeshFileHandler::saveOBJFile(subFolder + std::to_string(primitives.size() + fixedPointId + 1) + "-" + "CLIP",
                                   primPos, primMesh);
    }
    exportFrameInfo(this, forwardRecords[i], fullFolder + "info.txt");

  }

  writeStringToFile(OUTPUT_PARENT_FOLDER + "area.txt", areaTotalStr);
std::printf("finished\n");

}

void Simulation::exportCurrentMeshPos(int frameIdx, std::string fileName) {
  MeshFileHandler::exportMeshPos(forwardRecords[frameIdx].x, fileName);

  VecXd normals(particles.size() * 3);
  MeshFileHandler::saveOBJFile(fileName, forwardRecords[frameIdx].x,
                               getParticleNormals(mesh, forwardRecords[frameIdx].x),
                               mesh);
}

std::vector<Simulation::BackwardInformation>
Simulation::runBackwardTask(Simulation::BackwardTaskInformation taskConfiguration, LossType lossType,
                            Simulation::LossInfo &lossInfo, Simulation::TaskSolveStatistics &taskStatistics,
                            int FORWARD_STEPS, Simulation::ParamInfo guess, bool lossOnly,
                            const std::function<void(const std::string &)> &setTextBoxCB, bool skipForward) {
  std::printf("[runBackwardTask]...");
  setTextBoxCB("backward:resetSystem\n");
  if (!skipForward)
    resetSystemWithParams(taskConfiguration, guess);


  std::printf("forwardConverge set from %.10f to %.10f\n", Simulation::forwardConvergenceThreshold,  taskConfiguration.forwardAccuracyLevel);
  Simulation::forwardConvergenceThreshold = taskConfiguration.forwardAccuracyLevel;
  Simulation::backwardConvergenceThreshold = taskConfiguration.backwardAccuracyLevel;

  setTextBoxCB("backward:runforward\n");

  std::printf("looping over constraints..." );

  if (lossOnly)
    calcualteSeperateAt_p = false;
  else {
    calcualteSeperateAt_p = false;
    for (int type = 0; type < Constraint::CONSTRAINT_NUM; type++)
      calcualteSeperateAt_p |= taskConfiguration.dL_dk_pertype[type];
  }
  std::printf("finished...\n" );

  std::printf("forward started...");
  if (!skipForward) {
    for (int i = 0; i < FORWARD_STEPS; i++) {
      if (i % 20 == 0) {
        std::printf("%d..", i);
        std::fflush(stdout);
      }
      step();

    }

  }
  std::printf("finished...");



  if ((forwardRecords.empty())) {
    std::printf("ERROR: NO RECORDS \n");
    return {BackwardInformation{.loss = 0}};
  }

  int N = forwardRecords.size();

  VecXd dL_dlastx(3 * particles.size());
  VecXd dL_dlastv(3 * particles.size());
  VecXd empty(3 * particles.size());
  empty.setZero();
  setTextBoxCB("backward:calculate loss");
  double L = calculateLossAndGradient(lossType, lossInfo, dL_dlastx, dL_dlastv, N - 1, true);
  setTextBoxCB("backward:finished loss calculation");
  forwardRecords[forwardRecords.size() - 1].loss = L;

  taskStatistics.totalForwardSim++;

  taskStatistics.completeForwardLog.emplace_back(guess, forwardRecords[forwardRecords.size() - 1]);
  calcualteSeperateAt_p = false;

  if (lossOnly) {
    return {BackwardInformation{.loss = L}};
  }

  BackwardInformation derivative = backwardInfoDefault;
  derivative.dL_dx = dL_dlastx;
  derivative.dL_dv = dL_dlastv;
  derivative.loss = L;

  std::printf("init\n");

  VecXd dL_dxinit(3 * particles.size()), dL_dvinit(3 * particles.size());
  std::vector<BackwardInformation> fullBackwardRecords = {derivative};

  if ((FORWARD_STEPS + 1) != forwardRecords.size()) {
    std::printf("WARNING: invariant violated: FORWARD_STEPS:%d fowardRecords: %zu\n", FORWARD_STEPS,
                forwardRecords.size()); // FORWARD_STEPS = forwardRecords.size()) - 1
  }

  std::printf("backward started...");
  for (int idx = FORWARD_STEPS; idx >= 1; idx--) {
    if (idx % 50 == 0) {
      setTextBoxCB("backward:grad " + std::to_string(idx));
    }
    ForwardInformation &record = forwardRecords[idx]; // foward steps from [x,v]_{idx-1} --> [x,v]_{idx}
    if (idx % 20 == 0) {
      std::printf("%d..", idx);
      std::fflush(stdout);
    }

    calculateLossAndGradient(lossType, lossInfo, dL_dxinit, dL_dvinit, idx - 1, false);
    // backward from [x,v]_{idx} --> [x,v]_{idx-1}, calculate dL/dx_{idx-1}
    derivative = stepBackward(taskConfiguration, derivative, record, (idx - 1) == 0, dL_dxinit, dL_dvinit);
    fullBackwardRecords.emplace_back(derivative);
  }
  std::printf("finished...\n");

  std::reverse(fullBackwardRecords.begin(), fullBackwardRecords.end());
  fullBackwardRecords[0].correspondingForwardIdxInStats = taskStatistics.totalForwardSim - 1;

  taskStatistics.totalBackprop++;
  taskStatistics.completeBackwardLog.emplace_back(guess, fullBackwardRecords[0]);
  return fullBackwardRecords;
}

void Simulation::exportOptimizationRecords(Demos demoIdx, std::string experimentNameGiven, bool useGiven) {
  std::string experimentName = useGiven ? experimentNameGiven : (sceneConfig.name + "-" + currentTimestampToStr());
  std::printf("[exportOptimizationRecords] begin to save simulation...");
  std::fflush(stdout);
  std::string parentFolder = OUTPUT_PARENT_FOLDER + experimentName + "/";
  checkFolderExistsAndCreate(OUTPUT_PARENT_FOLDER);
  checkFolderExistsAndCreate(parentFolder);

  writeStringToFile(parentFolder + "task_info.txt", taskInfoToString(taskInfo));
  exportConfig(demoIdx, sceneConfig, parentFolder + "scene-config.txt");

  Simulation::ParamInfo &paramActual = groundtruthParam;
  std::vector<ForwardInformation> &simulationActual = groundTruthForwardRecords;
  if (!simulationActual.empty())
    exportSimulation(experimentName + "/groundtruth", simulationActual);
  writeStringToFile(
          OUTPUT_PARENT_FOLDER + experimentName + "/groundtruth/param.txt",
          parameterToString(taskInfo, groundtruthParam));


  for (int i = 0; i < backwardOptimizationRecords.size(); i++) {
    Simulation::ParamInfo &backwardParam = backwardOptimizationGuesses[i].first;
    std::string iterationFolder =
            OUTPUT_PARENT_FOLDER + experimentName + "/iter" + std::to_string(i);
    checkFolderExistsAndCreate(iterationFolder + "/");
    exportSimulation(experimentName + "/iter" + std::to_string(i), backwardOptimizationRecords[i].first);
    writeStringToFile(iterationFolder + "/param.txt", parameterToString(taskInfo, backwardParam));
    writeStringToFile(iterationFolder + "/param.txt", "Actual:\n" + parameterToString(taskInfo, paramActual));

    writeStringToFile(iterationFolder + "/gradientInfo.txt",
                      backwrdInfoAndGradToString(taskInfo, backwardOptimizationRecords[i].second[0]));
    writeStringToFile(iterationFolder + "/forwardInfo.txt", forwardInfoToString(taskInfo,
                                                                                backwardOptimizationRecords[i].first[
                                                                                        backwardOptimizationRecords[i].first.size() -
                                                                                        1]));
  }


}

void Simulation::exportStatistics(Demos demoIdx, Simulation::TaskSolveStatistics &statistics,
                                  Simulation::BackwardTaskInformation taskInfo, bool writePerf) {
  if (!statistics.configWritten) {
      std::string experimentName = sceneConfig.name + "-randseed-" + std::to_string(taskInfo.srandSeed)  + "-" + currentTimestampToStr() + "-forwardThresh-" +
                                 d2str(std::log10(taskInfo.forwardAccuracyLevel), 1);
    statistics.experimentName = experimentName;
    Logging::logMagenta("Optimization progress is saved to:" + statistics.experimentName + "\n");
  }

  std::string subFolder = std::string(taskInfo.optimizer == Optimizer::LBFGS ? "-LBFGS" : "-GD") + "/";
  std::string parentFolder =
          OUTPUT_PARENT_FOLDER + statistics.experimentName + subFolder;
  checkFolderExistsAndCreate(OUTPUT_PARENT_FOLDER);
  checkFolderExistsAndCreate(parentFolder);
  writeStringToFile(parentFolder + "iters.txt",
                    "Total forward:" + std::to_string(statistics.totalForwardSim) + "\nTotal backprop:" +
                    std::to_string(statistics.totalBackprop));

  if (!statistics.configWritten) {
    writeStringToFile(parentFolder + "task_info.txt", taskInfoToString(taskInfo));
    exportConfig(demoIdx, sceneConfig, parentFolder + "scene-config.txt");
    statistics.configWritten = true;
  }


  // Complete forward log
  int currentForwardInfoSize = statistics.completeForwardLog.size();
  int prevForwardWritten = statistics.forwardWritten;

  for (int i = statistics.forwardWritten; i < currentForwardInfoSize; i++) {
    ForwardInformation &info = statistics.completeForwardLog[i].second;
    ParamInfo &param = statistics.completeForwardLog[i].first;
    appendStringToFile(parentFolder + "forwardLog.txt",
                       "Record " + std::to_string(i) + "\n" + forwardInfoToString(taskInfo, info));

    appendStringToFile(parentFolder + "forwardLog.txt",
                       parameterToString(taskInfo, param));
  }

  statistics.forwardWritten = currentForwardInfoSize;

  // Complete backward log
  int currentBackwardInfoSize = statistics.completeBackwardLog.size();
  int prevBackwardWritten = statistics.backwardWritten;

  for (int i = statistics.backwardWritten; i < currentBackwardInfoSize; i++) {
    BackwardInformation &backwardInfo = statistics.completeBackwardLog[i].second;
    ParamInfo &param = statistics.completeBackwardLog[i].first;

    appendStringToFile(parentFolder + "backwardLog.txt",
                       "Record " + std::to_string(i) + "\n" + backwrdInfoAndGradToString(taskInfo, backwardInfo));
    appendStringToFile(parentFolder + "backwardLog.txt",
                       parameterToString(taskInfo, param) + "Corresponding forward Idx: " +
                       std::to_string(backwardInfo.correspondingForwardIdxInStats) + "\n");

  }
  statistics.backwardWritten = currentBackwardInfoSize;

  // Saving records
  int currentRecordSize = backwardOptimizationRecords.size();
  int prevRecordsSaved = statistics.optimizationRecordsSaved;
  for (int i = statistics.optimizationRecordsSaved; i < currentRecordSize; i++) {
    std::string iterationFolder =
            statistics.experimentName + subFolder + "/iter" + std::to_string(i);
    checkFolderExistsAndCreate(iterationFolder + "/");
    exportSimulation(iterationFolder,
                     backwardOptimizationRecords[i].first);
    writeStringToFile(OUTPUT_PARENT_FOLDER +  iterationFolder + "/param.txt",
                      parameterToString(taskInfo, backwardOptimizationGuesses[i].first));

  }

  statistics.optimizationRecordsSaved = currentRecordSize;

  // Last frame meshes
  std::string lastFrameMeshFolder = statistics.experimentName + subFolder + "last_frame_meshes/";
  checkFolderExistsAndCreate(lastFrameMeshFolder);
  exportStatsSimulations(statistics.completeForwardLog, lastFrameMeshFolder, prevForwardWritten,
                         currentForwardInfoSize);

  if (writePerf) {
    long long totalForwardTime = 0;
    long long totalBackwardTime = 0;
    std::string out = "";

    out += "Demo Name:" + DEMOS_STRINGS[demoIdx] + "\n";
    out += "Collision:" + boolToString("ON", "OFF", contactEnabled) + "\n";
    out += "Self-Collision:" + boolToString("ON", "OFF", selfcollisionEnabled) + "\n";
    out += "Wind:" + boolToString("ON", "OFF", windEnabled) + "\n";
    out += "BackwardSolver:" + boolToString("Jacobi", "Direct", backwardGradientForceDirectSolver) + "\n";
    out += "Total Particles:" + std::to_string(particles.size()) + "\n";
    out += "Fabric Name:" + sceneConfig.fabric.name + "\n";

    out += "======Backward iters =====\n";
    for (int i = 0; i < statistics.completeBackwardLog.size(); i++) {
      BackwardInformation &info = statistics.completeBackwardLog[i].second;
      out += "Total Jacobi iters:" + std::to_string(info.backwardTotalIters) + "\n";
      out += "Jacobi total converged:" + std::to_string(info.convergedAccum) + "\n";
      out += "Total backward props:" + std::to_string(sceneConfig.stepNum) + "\n";

    }


    out += "======Forward Runtime (unit, [s])=====\n";
    for (int i = 0; i < statistics.completeForwardLog.size(); i++) {
      ForwardInformation &info = statistics.completeForwardLog[i].second;
      totalForwardTime += info.totalRuntime;
      out += "iter" + std::to_string(i) + ":" + d2str(info.totalRuntime / 1000000.0, 8) + "\n";
    }


    out += "======Backward Runtime (unit, [s])=====\n";
    for (int i = 0; i < statistics.completeBackwardLog.size(); i++) {
      BackwardInformation &info = statistics.completeBackwardLog[i].second;
      totalBackwardTime += info.totalRuntime;
      out += "iter" + std::to_string(i) + ":" + d2str(info.totalRuntime / 1000000.0, 8) + "\n";
    }


    out += "Total Forward Time:" + d2str(totalForwardTime / 1000000.0, 6) + "\n";
    out += "Total Backward Time:" + d2str(totalBackwardTime / 1000000.0, 6) + "\n";
    out += "Total Time:" + d2str((totalForwardTime + totalBackwardTime) / 1000000.0, 6) + "\n";
    writeStringToFile(
            OUTPUT_PARENT_FOLDER + statistics.experimentName + subFolder +
            "perf.txt", out);
  }
}

void Simulation::exportConfig(int demoIdx, Simulation::SceneConfiguration &config, std::string fileName) {
  std::ofstream myfile;
  std::printf("writing config to file...%s.txt\n", fileName.c_str());
  myfile.open(fileName);
  myfile << "demoName:" << config.name << " " << "\n";
  myfile << "demoIdx:" << demoIdx << " " << "\n";
  myfile << "SceneBbox: min:" << vec2str(config.sceneBbox.min, 4) << " max:"
         << vec2str(config.sceneBbox.max, 4) + "\n";
  myfile << "FPS:" << ((int) std::round(1.0f / config.timeStep)) << "\n";
  myfile << "Frame Number:" << config.stepNum + 1 << "\n";
  myfile << "CameraPos:" << vec2str(config.camPos, 5) << "\n";
  Vec3d lookAtPos = toEigen(getLookAtPos(this, config));
  myfile << "CameraLookAtPos:" << vec2str(lookAtPos, 5) << "\n";
  myfile << "Collision:" << boolToString("ON", "OFF", contactEnabled) << "\n";
  myfile << "Self-Collision:" << boolToString("ON", "OFF", selfcollisionEnabled) << "\n";
  myfile << "Wind:" << boolToString("ON", "OFF", windEnabled) << "\n";
  myfile << "WindMode:" << windConfigStrings[sceneConfig.windConfig] << "\n";


  myfile << "Object Number:" << primitives.size() + 1 + (forwardRecords[0].x_fixedpoints.rows() / 3) << "\n";
  std::vector<std::pair<std::string, Vec3d>> objNameAndColor;
  objNameAndColor.emplace_back("0-CLOTH", config.fabric.color);
  for (int primId = 0; primId < primitives.size(); primId++) {
    Primitive *prim = primitives[primId];
    objNameAndColor.emplace_back(std::to_string(primId + 1) + "-" + Primitive::primitiveTypeStrings[prim->type],
                                 prim->color);

  }

  for (int fixedPointId = 0; fixedPointId < forwardRecords[0].x_fixedpoints.rows() / 3; fixedPointId++) {
    objNameAndColor.emplace_back(std::to_string(fixedPointId + 1 + primitives.size()) + "-CLIP", COLOR_WOOD);
  }

  for (std::pair<std::string, Vec3d> &nameColor : objNameAndColor) {
    myfile << nameColor.first << " color:" << vec2str(nameColor.second) << "\n";

  }
  myfile.close();

}

void Simulation::exportStatsSimulations(std::vector<std::pair<ParamInfo, ForwardInformation>> &records,
                                        std::string subFolder, int prevWrriten, int newWritten) {
  for (int i = prevWrriten; i < newWritten; i++) {
    MeshFileHandler::saveOBJFile(subFolder   + "0-CLOTH_forwardlastframe_iter_" + std::to_string(i), records[i].second.x, mesh);
    if (i == 0)
      for (int primId = 0; primId < primitives.size(); primId++) {
        Primitive *prim = primitives[primId];
        VecXd primPos;
        primPos.setZero();
        std::vector<Triangle> primMesh;
        std::string name = Primitive::primitiveTypeStrings[prim->type];
        prim->getMesh(primMesh, primPos, Vec3d(0, 0, 0), i);
        MeshFileHandler::saveOBJFile(subFolder + std::to_string(primId + 1) + "-" + name, primPos,
                                     getParticleNormals(primMesh, primPos), primMesh);
      }


    exportFrameInfo(this, records[i].second, subFolder + std::to_string(i) + "_" + "info.txt");

  }

}

void Simulation::exportSimulation(std::string fileName, std::vector<ForwardInformation> &records,
                                  bool staticPrimitivesFirstFrameOnly) {
  std::string parentFolder = OUTPUT_PARENT_FOLDER + fileName + "/";

  checkFolderExistsAndCreate(OUTPUT_PARENT_FOLDER);
  checkFolderExistsAndCreate(parentFolder);

  for (int i = 0; i < records.size(); i++) {
    std::string subFolder = fileName + "/"  ;
    std::string fullFolder = OUTPUT_PARENT_FOLDER + subFolder;

    checkFolderExistsAndCreate(fullFolder);
    MeshFileHandler::saveOBJFile(subFolder + std::to_string(i), records[i].x, mesh);
    if ((i == 0) && staticPrimitivesFirstFrameOnly)
      for (int primId = 0; primId < primitives.size(); primId++) {
        Primitive *prim = primitives[primId];
        VecXd primPos;
        primPos.setZero();
        std::vector<Triangle> primMesh;
        std::string name = Primitive::primitiveTypeStrings[prim->type];
        prim->getMesh(primMesh, primPos, Vec3d(0, 0, 0), i);
        MeshFileHandler::saveOBJFile(subFolder + std::to_string(primId + 1) + "-" + name, primPos,
                                     getParticleNormals(primMesh, primPos), primMesh);
      }

    bool exportFixedPointMesh = false;
    if (exportFixedPointMesh)
      for (int fixedPointId = 0; fixedPointId < records[i].x_fixedpoints.rows() / 3; fixedPointId++) {
        Vec3d clipCenter = records[i].x_fixedpoints.segment(fixedPointId * 3, 3);
        std::vector<Vec3i> primMesh;
        VecXd primPos;
        clip.getMesh(primMesh, primPos, clipCenter);
        MeshFileHandler::saveOBJFile(subFolder + std::to_string(primitives.size() + fixedPointId + 1) + "-" + "CLIP",
                                     primPos, primMesh);
      }

    exportFrameInfo(this, records[i], fullFolder + "info.txt");

  }




}

std::string Simulation::taskInfoToString(const Simulation::BackwardTaskInformation &taskInfo) {
  std::string out;
  out += "============Task Configuration:======================\n";

  out += "Forward Accuracy:" + d2str(taskInfo.forwardAccuracyLevel, 11) + "\n";
  out += "Backward Accuracy:" + d2str(taskInfo.backwardAccuracyLevel, 11) + "\n";
  out += "Rand seed:" + std::to_string(taskInfo.randSeed) + "\n";
  out += "Srand seed:" + std::to_string(taskInfo.srandSeed) + "\n";
  out += "Optimizer:";
  if (taskInfo.optimizer == Optimizer::LBFGS)
    out += "LBFGS\n";
  else
    out += "GradientDescent+LineSearch\n";


  for (int i = 0; i < Constraint::CONSTRAINT_NUM; i++)
    out += boolToStringTrueOnly(CONSTRAINT_TYPE_STRINGS[i] + ": ON", taskInfo.dL_dk_pertype[i]);
  out += boolToStringTrueOnly("density: ON", taskInfo.dL_density);
  out += boolToStringTrueOnly("f_ext: ON", taskInfo.dL_dfext);
  out += boolToStringTrueOnly("f_wind: ON", taskInfo.dL_dfwind);
  out += boolToStringTrueOnly("mu: ON", taskInfo.dL_dmu);
  out += boolToStringTrueOnly("spline: ON", taskInfo.dL_dcontrolPoints);

  if (taskInfo.dL_dcontrolPoints) {
    for (int i = 0; i < taskInfo.splineTypes.size(); i++) {
      if (!taskInfo.splineTypes.empty()) {
        out += "Spline set " + std::to_string(i) + ":\n";
        for (int j = 0; j < taskInfo.splineTypes[i].size(); j++) {
          out += "s_" + std::to_string(i) + ":" + SPLINE_TYPE_STRINGS[taskInfo.splineTypes[i][j]] + "\n";
        }
      }
    }
  }


  if (taskInfo.dL_dmu) {
    out += "primitive mu id:\n";
    for (int id : taskInfo.mu_primitives)
      out += std::to_string(id) + ",";
    out += "\n";
  }


  return out;
}

std::string
Simulation::parameterToString(const Simulation::BackwardTaskInformation &taskInfo, Simulation::ParamInfo paramGuess) {
  std::string out;
  out += "============Parameter Info:======================\n";

  for (int i = 0; i < Constraint::ConstraintType::CONSTRAINT_NUM; i++) {
    if (taskInfo.dL_dk_pertype[i]) {
      out += "k_" + CONSTRAINT_TYPE_STRINGS[i] + ":" + d2str(paramGuess.k_pertype[i], 6) + "\n";
    }
  }

  if (taskInfo.dL_density) {
    out += "density:" + d2str(paramGuess.density, 6) + "\n";
  }

  if (taskInfo.dL_dmu) {
    for (std::pair<int, double> pair : paramGuess.mu) {
      out += "mu from prim" + std::to_string(pair.first) + ":" + d2str(pair.second, 6) + "\n";

    }
  }

  if (taskInfo.dL_dx0) {
    out += "x0: norm:" + d2str(paramGuess.x0.norm(), 5) + "\n";
    for (int i = 0; i < paramGuess.x0.rows() / 3; i += 200) {
      out += "pId-" + std::to_string(i) + vecXd2str(paramGuess.x0.segment(i * 3, 3), 5) + "\n";

    }
  }

  if (taskInfo.dL_dfext) {
    out += "f_ext:" + vec2str(paramGuess.f_ext, 6) + "\n";
  }

  if (taskInfo.dL_dfwind) {
    out += "f_wind:" + vec2str(paramGuess.f_extwind, 6) + "\n";
  }


  if (taskInfo.dL_dcontrolPoints) {
    for (int matSetIdx = 0; matSetIdx < paramGuess.controlPointSplines.size(); matSetIdx++) {
      out += "====set" + std::to_string(matSetIdx) + "=====\n";
      for (int i = 0; i < paramGuess.controlPointSplines[matSetIdx].size(); i++) {
        Spline &spline = paramGuess.controlPointSplines[matSetIdx][i];
        for (Spline::Segment &s : spline.segments) {
          out += "t[" + d2str(s.startFraction) + "," + d2str(s.endFraction) + "]:";
          out += "p1:" + vec2str(spline.segments[0].p1, 6);

          switch (spline.type) {
            case Spline::SplineType::ENDPOINT_AND_UP: {
              out += ",up:" + d2str(spline.segments[0].yUp, 4);
              break;
            }
            case Spline::SplineType::ENDPOINT_AND_TANGENTS: {
              out += ",m0:" + vec2str(spline.segments[0].m0, 4);
              out += ",m1:" + vec2str(spline.segments[0].m1, 4);
              break;
            }

            case Spline::SplineType::ENDPOINT: {
              break;
            }
          }
          out += "\n";

        }


      }
    }

  }
  return out;

}

std::string
Simulation::backwrdInfoAndGradToString(const Simulation::BackwardTaskInformation &taskInfo,
                                       Simulation::BackwardInformation grad) {
  std::string out;
  out += "============Backward Iter Info:======================\n";
  out += "Loss:" + d2str(grad.loss, 3) + "\n";
  out += "Number of Converged Iter:" + std::to_string(grad.convergedAccum) + "\n";
  out += "Total Backward Iter:" + std::to_string(grad.backwardTotalIters) + "\n";
  out += "Backward Total Runtime[ms]:" + d2str(grad.totalRuntime / 1000.0, 7) + "\n";
  out += "Backward Non Solve Time[ms]:" +
         std::to_string(grad.accumSolvePerformanceReport.nonSolveTimeMicroseconds / 1000.0) +
         "\n";
  out += "Backward Direct-Solve Time[ms]:" +
         std::to_string(grad.accumSolvePerformanceReport.solveDirectMicroseconds / 1000.0) +
         "\n";
  out += "Backward Iter-Solve Time[ms]:" +
         std::to_string(grad.accumSolvePerformanceReport.solveIterativeMicroseconds / 1000.0) +
         "\n";

  out += "============Timer Breakdown:======================\n";
  for (TimerEntry &breakdown: grad.accumTimer) {
    std::string label = breakdown.first;
    long long duration = breakdown.second;
    double percentage = duration * 1.0 / grad.totalRuntime;
    out += label + "[ms]:" + d2str(duration / 1000.0, 5) + "\t\t|" + d2str(percentage * 100, 3) + "%\n";
  }
  double percentageDirect = grad.accumSolvePerformanceReport.solveDirectMicroseconds * 1.0 / grad.totalRuntime;
  double percentageIter = grad.accumSolvePerformanceReport.solveIterativeMicroseconds * 1.0 / grad.totalRuntime;
  out += "Direct-Solve[ms]:" + d2str(grad.accumSolvePerformanceReport.solveDirectMicroseconds / 1000.0, 5) + "\t|" +
         d2str(percentageDirect * 100, 3) + "%\n";
  out += "Iter-Solve[ms]:" + d2str(grad.accumSolvePerformanceReport.solveIterativeMicroseconds / 1000.0, 5) +
         "\t|" +
         d2str(percentageIter * 100, 3) + "%\n";


  out += "\n============Gradient Info:======================\n";
  for (int i = 0; i < Constraint::ConstraintType::CONSTRAINT_NUM; i++) {
    if (taskInfo.dL_dk_pertype[i]) {
      out += "dL/dk_" + CONSTRAINT_TYPE_STRINGS[i] + ":" + d2str(grad.dL_dk_pertype[i], 5) + "\n";
    }
  }

  if (taskInfo.dL_density) {
    out += "dL/ddensity:" + d2str(grad.dL_ddensity, 5) + "\n";
  }

  if (taskInfo.dL_dmu) {
    for (std::pair<int, double> pair : grad.dL_dmu) {
      out += "dL_dmu_" + std::to_string(pair.first) + ":" + d2str(pair.second, 5) + "\n";

    }
  }

  if (taskInfo.dL_dfext) {
    out += "dL/df_ext:" + vec2str(grad.dL_dfext, 4) + "\n";
  }

  if (taskInfo.dL_dfwind) {
    out += "dL/df_wind:" + vec2str(grad.dL_dwind, 4) + "\n";
  }

  if (taskInfo.dL_dx0) {
    out += "dL/dx0: norm:" + d2str(grad.dL_dx.norm(), 5) + "\n";
    for (int i = 0; i < grad.dL_dx.rows() / 3; i += 200) {
      out += "pId-" + std::to_string(i) + vecXd2str(grad.dL_dx.segment(i * 3, 3), 5) + "\n";

    }
  }


  if (taskInfo.dL_dcontrolPoints) {
    for (int matSetIdx = 0; matSetIdx < grad.dL_dsplines.size(); matSetIdx++) {
      out += "====set" + std::to_string(matSetIdx) + "=====\n";
      for (int i = 0; i < grad.dL_dsplines[matSetIdx].size(); i++) {
        VecXd &dL_dspline = grad.dL_dsplines[matSetIdx][i];
        Spline::SplineType type = taskInfo.splineTypes[matSetIdx][i];

        out += "dL_dp1:" + vecXd2str(dL_dspline.segment(0, 3), 4);

        switch (type) {
          case Spline::SplineType::ENDPOINT_AND_UP: {
            out += ",dL_dup:" + d2str(dL_dspline[3], 4);
            break;
          }
          case Spline::SplineType::ENDPOINT_AND_TANGENTS: {
            out += ",dL_dm0:" + vecXd2str(dL_dspline.segment(3, 3), 4);
            out += ",dL_dm1:" + vecXd2str(dL_dspline.segment(6, 3), 4);
            break;
          }

          case Spline::SplineType::ENDPOINT: {
            break;
          }
        }
        out += "\n";


      }
    }
  }
  return out;
}

VecXd Simulation::getParticleNormals(std::vector<Triangle> mesh, const VecXd &x_now) {
  int particleNum = x_now.rows() / 3;
  VecXd normals(particleNum * 3);
  normals.setZero();


  for (Triangle &t : mesh) {
    Vec3d p0 = x_now.segment(t.p0_idx * 3, 3);
    Vec3d p1 = x_now.segment(t.p1_idx * 3, 3);
    Vec3d p2 = x_now.segment(t.p2_idx * 3, 3);
    t.normal = t.getNormal(p0, p1, p2);

    for (int pId : t.idxArr) {
      normals.segment(pId * 3, 3) += t.normal;
    }
  }

  for (int i = 0; i < particleNum; i++) {
    normals.segment(i * 3, 3).normalize();
  }

  return normals;

}
 
SpMat Simulation::factorizeDirectSolverSparseLU(const SpMat &A, Eigen::SparseLU<SpMat> &lltSolver,
                                                const std::string &warning_msg) {
  lltSolver.compute(A);
  SpMat Afixed = A;
  double regularization = 1e-10;
  bool success = true;
  SpMat I = SpMat(A.rows(), A.cols());
  I.setIdentity();
  while (lltSolver.info() != Eigen::Success) {
    regularization *= 10;
    Afixed = Afixed + regularization * I;
    lltSolver.compute(Afixed);
    success = lltSolver.info();
    if (regularization > 100)
      break;
  }
  if (!success) {
    std::cout << "Warning: " << warning_msg << " adding " << regularization << " identites.(llt solver)"
              << std::endl;
  }

  return Afixed;
}

SpMat Simulation::factorizeDirectSolverLLT(const SpMat &A, Eigen::SimplicialLLT<SpMat> &lltSolver,
                                           const std::string &warning_msg) {
  lltSolver.compute(A);
  SpMat Afixed = A;
  double regularization = 1e-10;
  bool success = true;
  SpMat I = SpMat(A.rows(), A.cols());
  I.setIdentity();
  while (lltSolver.info() != Eigen::Success) {
    regularization *= 10;
    Afixed = Afixed + regularization * I;
    lltSolver.compute(Afixed);
    success = lltSolver.info();
  }
  if (!success) {
//        std::cout << "Warning: " << warning_msg << " adding " << regularization << " identites.(llt solver)"
//                  << std::endl;
  }

  return Afixed;
}
