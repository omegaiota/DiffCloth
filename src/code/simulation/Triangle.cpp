//
// Created by Yifei Li on 9/28/20.
// Email: liyifei@csail.mit.edu
//

#include "Triangle.h"

Eigen::Vector4d Triangle::k = Eigen::Vector4d(123.989220f, 105.181770f, 365.966217f, 44.217571f);
//Eigen::Vector4d Triangle::k =  Eigen::Vector4d(100.0f, 0, 0.0, 0.0);
double Triangle::k_stiff = (k[0] + 2.0 * k[1] + k[2] + k[3]) / 2.0;
//double Triangle::k_stiff_stretching = k[0];
//double Triangle::k_stiff_stretching = 360;

//Eigen::Vector4d Triangle::k = Eigen::Vector4d(k_stiff_stretching, 0, k_stiff_stretching, k_stiff_stretching);
//Eigen::Vector4d Triangle::k =  Eigen::Vector4d(1,1,1,0);


Mat3x2d Triangle::getDeformationGradient(const VecXd &x_vec) const {
    Mat3x2d p;
    p.col(0) = p1_vec3(x_vec) - p0_vec3(x_vec);
    p.col(1) = p2_vec3(x_vec) - p0_vec3(x_vec);
    return p * inv_deltaUV;
}

Mat2x3d Triangle::getDerivative() const {
    Mat2x3d p;
    p.row(0) = Vec3d(-1, 1, 0);
    p.row(1) = Vec3d(-1, 0, 1);
    return inv_deltaUV.transpose() * p;
}




double Triangle::evaluateEnergy(const Mat3x2d &F, const VecXd &x_new) {
  switch (energyType) {
    case QUADRATIC:
      energyBuffer = 0.5 * k_stiff * area_rest * (F - projectToManifold(x_new)).squaredNorm();
      return energyBuffer;
      break;
    default:
    case NON_QUADRATIC:
      Mat2x2d G1 = (F.transpose() * F - I_two) / 2.;

      double m_energy = area_rest * (
              k[0] * G1(0, 0) * G1(0, 0) +
              k[2] * G1(1, 1) * G1(1, 1) +
              k[1] * G1(0, 0) * G1(1, 1) * 2.0f +
              k[3] * G1(0, 1) * G1(0, 1)) / 2.0f;
      energyBuffer = m_energy;
      return m_energy;
  }

}

double Triangle::evaluateEnergy(const VecXd &x_new) {
  return evaluateEnergy(getDeformationGradient(x_new), x_new);
}

Vec9d Triangle::stretchingForce(const VecXd &x_new) {
  Mat3x2d F = getDeformationGradient(x_new);
  Vec3d F_c0 = F.col(0), F_c1 = F.col(1); // should equal Du*mat_to_vec(X)

  switch (energyType) {
    case QUADRATIC: {
      Mat3x2d proj = projectToManifold(x_new);
      Mat3x2d P = (F - proj);
      Vec6d P_vec, proj_vec;
            P_vec.block<3, 1>(0, 0) = P.col(0);
            P_vec.block<3, 1>(3, 0) = P.col(1);

      Mat6x9d dproj_dx = projectToManifoldBackward(x_new);
      stretchingForceBuffer =
              -k_stiff * area_rest * P_vec.transpose() * (dF_dx.transpose() - dproj_dx.transpose()).transpose();


      return stretchingForceBuffer;

      break;
    }

        default:
        case NON_QUADRATIC: {
            Mat2x2d G = (F.transpose() * F - I_two) / 2.0; // Green's strain
            double e_uu = G(0, 0), e_vv = G(1, 1), e_uv = G(0, 1);


            Vec3d dE_deps = Vec3d(k[0] * e_uu + k[1] * e_vv, k[1] * e_uu + k[2] * e_vv, 2 * k[3] * e_uv);
            Eigen::Matrix<double, 3, 6> deps_dF;
            deps_dF.setZero();
            deps_dF.block<1, 3>(0, 0) = F_c0;
            deps_dF.block<1, 3>(1, 3) = F_c1;
            deps_dF.block<1, 3>(2, 0) = 0.5 * F_c1;
            deps_dF.block<1, 3>(2, 3) = 0.5 * F_c0;

          Eigen::Matrix<double, 3, 9> deps_dx = deps_dF * dF_dx;

          Vec9d grad_e = dE_deps.transpose() * deps_dx;
          stretchingForceBuffer = -grad_e * area_rest;
          return stretchingForceBuffer; // TODO: add area back
        }


  }

//  return std::make_pair(-area * hess_e, -area * grad_e);
}

Vec9d Triangle::dfi_dk(const VecXd &x_new) {
  Mat3x2d F = getDeformationGradient(x_new);
  Vec3d F_c0 = F.col(0), F_c1 = F.col(1); // should equal Du*mat_to_vec(X)

  switch (energyType) {
    case QUADRATIC: {

      dfi_dk_buffer = stretchingForce(x_new) / (constrainWeightSqrt * constrainWeightSqrt);
      return dfi_dk_buffer;

      break;
    }

    default:
    case NON_QUADRATIC: {
      std::printf("WARNING NOT IMPLEMENTED\n");
      Mat2x2d G = (F.transpose() * F - I_two) / 2.0; // Green's strain
      double e_uu = G(0, 0), e_vv = G(1, 1), e_uv = G(0, 1);


      Vec3d dE_deps = Vec3d(k[0] * e_uu + k[1] * e_vv, k[1] * e_uu + k[2] * e_vv, 2 * k[3] * e_uv);
      Eigen::Matrix<double, 3, 6> deps_dF;
      deps_dF.setZero();
      deps_dF.block<1, 3>(0, 0) = F_c0;
      deps_dF.block<1, 3>(1, 3) = F_c1;
      deps_dF.block<1, 3>(2, 0) = 0.5 * F_c1;
      deps_dF.block<1, 3>(2, 3) = 0.5 * F_c0;

      Eigen::Matrix<double, 3, 9> deps_dx = deps_dF * dF_dx;

      Vec9d grad_e = dE_deps.transpose() * deps_dx;

      dfi_dk_buffer = -grad_e * area_rest; // TODO: add area back
      return dfi_dk_buffer;
    }


  }

//  return std::make_pair(-area * hess_e, -area * grad_e);
}


Mat9x9d outer(Vec9d a, Vec9d b) {
  return (a * b.transpose());
}
//Mat9x9d outer(Vec9d a, Vec9d b) {
//  Mat9x9d A;
//  for (int j = 0; j < 9; j++) A.col(j) = a * b[j];
//  return A;
//}

#ifdef USE_DEBUG
std::pair<std::vector<Vec9d>, Mat9x9d> Triangle::stretchingForceAndJacobianCheck() const {
  Mat3x2d F = getDeformationGradient();
  Mat2x2d I_two;
  Mat3x3d I_three;
  I_two.setIdentity();
  I_three.setIdentity();
  Mat2x2d G = (F.transpose() * F - I_two) / 2.0; // Green's strain
  Eigen::Vector4d k(123.989220f, 105.181770f, 365.966217f, 44.217571f);
  double e_uu = G(0, 0), e_vv = G(1, 1), e_uv = G(0, 1);

  Mat2x3d p;
  p.row(0) = Vec3d(-1, 1, 0);
  p.row(1) = Vec3d(-1, 0, 1);

  Mat2x3d D = inv_deltaUV.transpose() * p;
  Vec3d du = D.row(0), dv = D.row(1);
  Eigen::Matrix<double, 3,9> Du = kronecker<1,3,3,3>(du , I_three),
          Dv = kronecker<1,3,3,3>(dv, I_three);


  Eigen::Matrix<double, 3, 9> Du_v2 = dF_dx.block<3,9>(0,0);
  Eigen::Matrix<double, 3, 9> Dv_v2  = dF_dx.block<3,9>(3,0);

  double diffdu = (Du - Du_v2).norm();
  double diffdv = (Dv - Dv_v2).norm();

  AssertDebug( diffdu < 1e-10);
  AssertDebug( diffdv < 1e-10);

  const Vec3d &xu = F.col(0), &xv = F.col(1); // should equal Du*mat_to_vec(X)
  Vec9d fuu_2 = Du.transpose() * xu,
          fvv_2 = Dv.transpose() * xv,
          fuv_2 = (Du.transpose() * xv + Dv.transpose() * xu) / 2.;

  Vec9d grad_e2 =   (k[0] * e_uu + k[1] * e_vv) * fuu_2
                  + (k[2] * e_vv + k[1] * e_uu) * fvv_2 //  force
                  + 2 * k[3] * e_uv * fuv_2;

//  Vec9d grad_e2 = (k[0] * e_uu + k[1] * e_vv) * fuu_2
//                  + (k[1] * e_uu+ k[2] * e_vv) * fvv_2 //  force
//                  + 2 * k[3] * e_uv * fuv_2;


  Mat9x9d hess_e2 = k[0]*(outer(fuu_2, fuu_2) + std::fmax(e_uu, 0.) * Du.transpose() * Du) // Jacobian of force
                    + k[2]*(outer(fvv_2, fvv_2) + std::fmax(e_vv, 0.) * Dv.transpose() * Dv)
                    + k[1]*(outer(fuu_2, fvv_2) + std::fmax(e_uu, 0.) * Dv.transpose() * Dv
                            + outer(fvv_2, fuu_2) + std::fmax(e_vv, 0.) * Du.transpose() * Du)
                    + 2.*k[3]*(outer(fuv_2, fuv_2));

  std::vector<Vec9d> all9dVec;
  all9dVec.push_back(grad_e2);
  all9dVec.push_back(fuu_2);
  all9dVec.push_back(fvv_2);
  all9dVec.push_back(fuv_2);
  return std::make_pair(all9dVec, hess_e2); // TODO: add area back
}
#endif

// this function should be called whenever particle positions are changed since F depends on current pos


Mat9x9d Triangle::stretchingHessian(const VecXd &x_new) const {

    switch (energyType) {
        case QUADRATIC: {
            Mat6x9d dproj_dx = projectToManifoldBackward(x_new);
            return k_stiff * area_rest * ((dF_dx - dproj_dx).transpose() * dF_dx  );
            break;
        }
        case NON_QUADRATIC: {
            Mat3x2d F = getDeformationGradient(x_new);
            Vec6d F_vec;
            F_vec.block<3, 1>(0, 0) = F.col(0);
            F_vec.block<3, 1>(3, 0) = F.col(1);

            Mat2x2d G = (F.transpose() * F - I_two) / 2.0; // Green's strain

            double e_uu = G(0, 0), e_vv = G(1, 1), e_uv = G(0, 1);

            Eigen::Matrix<double, 3, 9> Du = dF_dx.block<3, 9>(0, 0);
            Eigen::Matrix<double, 3, 9> Dv = dF_dx.block<3, 9>(3, 0);

            Vec3d F_c0 = F.col(0), F_c1 = F.col(1); // should equal Du*mat_to_vec(X)



            Vec3d dE_deps = Vec3d(k[0] * e_uu + k[1] * e_vv, k[1] * e_uu + k[2] * e_vv, 2 * k[3] * e_uv);
            Eigen::Matrix<double, 3, 6> deps_dF;
            deps_dF.setZero();
            deps_dF.block<1, 3>(0, 0) = F_c0;
            deps_dF.block<1, 3>(1, 3) = F_c1;
            deps_dF.block<1, 3>(2, 0) = 0.5 * F_c1;
            deps_dF.block<1, 3>(2, 3) = 0.5 * F_c0;

            Eigen::Matrix<double, 3, 9> deps_dx = deps_dF * dF_dx;

            Vec9d fuu = deps_dx.row(0);
            Vec9d fvv = deps_dx.row(1);
            Vec9d fuv = deps_dx.row(2);


            Mat9x9d hess_e = k[0] * (outer(fuu, fuu) + std::fmax(e_uu, 0.) * Du.transpose() * Du) // Jacobian of force
                             + k[2] * (outer(fvv, fvv) + std::fmax(e_vv, 0.) * Dv.transpose() * Dv)
                             + k[1] * ((outer(fuu, fvv)) + std::fmax(e_uu, 0.) * Dv.transpose() * Dv
                                       + (outer(fvv, fuu)) + std::fmax(e_vv, 0.) * Du.transpose() * Du)
                             + 2. * k[3] * (outer(fuv, fuv));


            return hess_e * area_rest; // TODO: add area back
            break;
        }

    }

}

double clamp(double x, double min, double max) {
    if (x < min)
        return min;
    if (x > max)
        return max;
    return x;
}


void Triangle::addConstraint(std::vector<Triplet> &tri, int &c_idx, bool withWeight) {
    if (withWeight) {
        this->c_idx = c_idx;
    } else {
        this->c_weightless_idx = c_idx;
    }

    double weightUsed = withWeight ? constrainWeightSqrt : std::sqrt(area_rest * 1);

    for (int i = 0; i < 2; ++i) {
        for (int dim = 0; dim < 3; dim++) {
            tri.emplace_back(c_idx + dim + 3 * i, p0()->idx * 3 + dim,
                             -weightUsed * (inv_deltaUV(0, i) + inv_deltaUV(1, i)));
            tri.emplace_back(c_idx + dim + 3 * i, p1()->idx * 3 + dim, weightUsed * inv_deltaUV(0, i));
            tri.emplace_back(c_idx + dim + 3 * i, p2()->idx * 3 + dim, weightUsed * inv_deltaUV(1, i));
        }

    }

    c_idx += constraintNum;

}

VecXd Triangle::project(const VecXd &x_vec) const {
    Mat3x2d newF = projectToManifold(x_vec);
    Vec6d ret;
    ret.block<3, 1>(0, 0) = newF.col(0);
    ret.block<3, 1>(3, 0) = newF.col(1);
    return ret * constrainWeightSqrt;

}

VecXd Triangle::dp_dk(const VecXd &x_vec) const { // this is wrong, does not include area
    Mat3x2d newF = projectToManifold(x_vec);

    Vec6d ret;
    ret.block<3, 1>(0, 0) = newF.col(0);
    ret.block<3, 1>(3, 0) = newF.col(1);
    return ret;
}


Mat3x2d Triangle::projectToManifold(const VecXd &x_vec) const {

    Mat3x2d F = getDeformationGradient(x_vec);
    Mat3x2d newdeltaUV;
    newdeltaUV.setZero();
//  Mat3x2d p;
//  p.col(0) = p1_vec3(x_vec)  - p0_vec3(x_vec);
//  p.col(1) = p2_vec3(x_vec)  - p0_vec3(x_vec);

    Mat3x2d p = F;

    newdeltaUV.col(0) = p.col(0).normalized();
    newdeltaUV.col(1) = (p.col(1) - p.col(1).dot(newdeltaUV.col(0)) * newdeltaUV.col(0)).normalized();


    Mat2x2d F_2D = newdeltaUV.transpose() * F;
    Eigen::JacobiSVD<Mat2x2d> svd(F_2D, Eigen::ComputeFullU | Eigen::ComputeFullV);


    Mat2x2d F_2D_project = svd.matrixU() * svd.matrixV().transpose();
    Mat3x2d newF = newdeltaUV * F_2D_project;
    return newF;
}


Mat6x9d Triangle::projectToManifoldBackward(const VecXd &x_vec) const {
    Mat3x3d I_three = Mat3x3d::Identity();
    Mat2x2d I_two = Mat2x2d::Identity();
    Mat3x2d newdeltaUV;
    Vec3d a = p1_vec3(x_vec) - p0_vec3(x_vec);
    Vec3d aNorm = a.normalized();
    Vec3d b = p2_vec3(x_vec) - p0_vec3(x_vec);
    Mat3x2d p;
    p.col(0) = a;
    p.col(1) = b;

    // backward: p,b,aNorm --> x
    Mat2x3d dp_dxvec;
    dp_dxvec << -1, 1, 0,
            -1, 0, 1;
    Mat6x9d dp_dx = kronecker<2, 3, 3, 3>(dp_dxvec, I_three); //checked


    Mat3x9d db_dx = dp_dx.block<3, 9>(3, 0); // checked
    Mat3x9d da_dx = dp_dx.block<3, 9>(0, 0); // checked

    Mat3x3d daNorm_da = (I_three - aNorm * aNorm.transpose()) / a.norm();
    Mat3x9d daNorm_dx = daNorm_da * da_dx;


    // forward: aNorm, b --> f --> newDeltaUV
    Vec3d f = (b - b.dot(aNorm) * aNorm);
    Vec3d fNorm = f.normalized();
    newdeltaUV.col(0) = aNorm;
    // f --> col1
    newdeltaUV.col(1) = fNorm;

    // backward: newDeltaUV --> f --> aNorm, b
    Mat3x3d dcol1_df = (I_three - fNorm * fNorm.transpose()) / f.norm();
    Mat3x3d df_db = I_three - aNorm * aNorm.transpose();
    Mat3x3d df_daNorm = -(aNorm * b.transpose() + aNorm.transpose() * b * I_three);
    Mat3x9d df_dx = df_db * db_dx + df_daNorm * daNorm_dx;


    Mat6x9d dnewdeltaUV_dx;
    dnewdeltaUV_dx.setZero();
    dnewdeltaUV_dx.block<3, 9>(0, 0) = daNorm_dx;
    dnewdeltaUV_dx.block<3, 9>(3, 0) = dcol1_df * df_dx;


    //          2x3                  3x2
    Mat2x2d F1 = (newdeltaUV.transpose() * p);

    // backward
    Mat4x6d dF1_dnewdeltaUV, dF1_dp; // 4 x 6
    dF1_dnewdeltaUV.setZero();
    dF1_dnewdeltaUV.block<1, 3>(0, 0) = p.col(0);
    dF1_dnewdeltaUV.block<1, 3>(2, 0) = p.col(1);
    dF1_dnewdeltaUV.block<1, 3>(1, 3) = p.col(0);
    dF1_dnewdeltaUV.block<1, 3>(3, 3) = p.col(1);

    dF1_dp.setZero();
    dF1_dp.block<1, 3>(0, 0) = newdeltaUV.col(0);
    dF1_dp.block<1, 3>(1, 0) = newdeltaUV.col(1);
    dF1_dp.block<2, 3>(2, 3) = dF1_dp.block<2, 3>(0, 0);

    Mat4x9d dF1_dx; // 4 x 9
    dF1_dx = dF1_dnewdeltaUV * dnewdeltaUV_dx + dF1_dp * dp_dx; //checked
    Mat2x2d F = F1 * inv_deltaUV;

    Mat4x4d dF_dF1 = kronecker<2, 2, 2, 2>(inv_deltaUV.transpose(), I_two);;
    Mat4x9d dF_dx = dF_dF1 * dF1_dx;


    //forward  F --> R
    Eigen::JacobiSVD<Mat2x2d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat2x2d R = svd.matrixU() * svd.matrixV().transpose();

    // backward R--> F
    Mat2x2d S = svd.matrixV() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
    Vec4d lhs(-R(0, 1), -R(1, 1), R(0, 0), R(1, 0));
    Vec4d x = lhs / S.trace();
    Mat4x4d dR_dF = lhs * x.transpose();

    Mat4x9d dR_dx = dR_dF * dF_dx; //checked




    Mat6x6d dF2_dnewdeltaUV;
    Mat6x4d dF2_dR;
    dF2_dnewdeltaUV.setZero();
    dF2_dR.setZero();
    dF2_dnewdeltaUV = kronecker<2, 2, 3, 3>(R.transpose(), I_three);
    dF2_dR.block<3, 2>(0, 0) = newdeltaUV;
    dF2_dR.block<3, 2>(3, 2) = newdeltaUV;

    // Mat2x2d R = svd.matrixU() * svd.matrixV().transpose();
    //                 3x2       2x2
    //  Mat3x2d F2 = newdeltaUV * R
    Mat6x9d dF2_dx = dF2_dnewdeltaUV * dnewdeltaUV_dx + dF2_dR * dR_dx;
    return dF2_dx;
}

void Triangle::projectBackwardPrecompute(const VecXd &x_vec) {
    newF = projectToManifoldBackward(x_vec);
    newF *= constrainWeightSqrt;
}

void Triangle::projectBackward(const VecXd &x_vec, TripleVector &triplets) {
  insertIntoTriplets(triplets, newF, 6, 3, 0, 0, c_idx, p0()->idx * 3);
  insertIntoTriplets(triplets, newF, 6, 3, 0, 3, c_idx, p1()->idx * 3);
  insertIntoTriplets(triplets, newF, 6, 3, 0, 6, c_idx, p2()->idx * 3);
//  dproj_dxnew.block<6, 3>(c_idx, p0()->idx * 3) += newF.block<6, 3>(0, 0);
//  dproj_dxnew.block<6, 3>(c_idx, p1()->idx * 3) += newF.block<6, 3>(0, 3);
//  dproj_dxnew.block<6, 3>(c_idx, p2()->idx * 3) += newF.block<6, 3>(0, 6);

}

std::pair<Mat6x9d, Mat3x2d> Triangle::forwardBackwardCheck(const VecXd &x_vec) const {
  // forward: x-->p,aNorm, b,
  Mat3x3d I_three = Mat3x3d::Identity();
  Mat2x2d I_two = Mat2x2d::Identity();
  Mat3x2d newdeltaUV;
  Vec3d a = p1_vec3(x_vec) - p0_vec3(x_vec);
  Vec3d aNorm = a.normalized();
  Vec3d b = p2_vec3(x_vec) - p0_vec3(x_vec);
  Mat3x2d p;
  p.col(0) = a;
  p.col(1) = b;

  // backward: p,b,aNorm --> x
  Mat2x3d dp_dxvec;
  dp_dxvec << -1, 1, 0,
          -1, 0, 1;
  Mat6x9d dp_dx = kronecker<2, 3, 3, 3>(dp_dxvec, I_three); //checked


  Mat3x9d db_dx = dp_dx.block<3, 9>(3, 0); // checked
  Mat3x9d da_dx = dp_dx.block<3, 9>(0, 0); // checked

  Mat3x3d daNorm_da = (I_three - aNorm * aNorm.transpose()) / a.norm();
  Mat3x9d daNorm_dx = daNorm_da * da_dx;


  // forward: aNorm, b --> f --> newDeltaUV
  Vec3d f = (b - b.dot(aNorm) * aNorm);
  Vec3d fNorm = f.normalized();
  newdeltaUV.col(0) = aNorm;
  // f --> col1
  newdeltaUV.col(1) = fNorm;

  // backward: newDeltaUV --> f --> aNorm, b
  Mat3x3d dcol1_df = (I_three - fNorm * fNorm.transpose()) / f.norm();
  Mat3x3d df_db = I_three - aNorm * aNorm.transpose();
  Mat3x3d df_daNorm = -(aNorm * b.transpose() + aNorm.transpose() * b * I_three);
  Mat3x9d df_dx = df_db * db_dx + df_daNorm * daNorm_dx;


  Mat6x9d dnewdeltaUV_dx;
  dnewdeltaUV_dx.setZero();
  dnewdeltaUV_dx.block<3, 9>(0, 0) = daNorm_dx;
  dnewdeltaUV_dx.block<3, 9>(3, 0) = dcol1_df * df_dx;


  //          2x3                  3x2
  Mat2x2d F1 = (newdeltaUV.transpose() * p);


  // backward
  Mat4x6d dF1_dnewdeltaUV, dF1_dp; // 4 x 6
  dF1_dnewdeltaUV.setZero();
  dF1_dnewdeltaUV.block<1, 3>(0, 0) = p.col(0);
  dF1_dnewdeltaUV.block<1, 3>(2, 0) = p.col(1);
  dF1_dnewdeltaUV.block<1, 3>(1, 3) = p.col(0);
  dF1_dnewdeltaUV.block<1, 3>(3, 3) = p.col(1);

  dF1_dp.setZero();
  dF1_dp.block<1, 3>(0, 0) = newdeltaUV.col(0);
  dF1_dp.block<1, 3>(1, 0) = newdeltaUV.col(1);
  dF1_dp.block<2, 3>(2, 3) = dF1_dp.block<2, 3>(0, 0);

  Mat4x9d dF1_dx; // 4 x 9
  dF1_dx = dF1_dnewdeltaUV * dnewdeltaUV_dx + dF1_dp * dp_dx; //checked
  Mat2x2d F = F1 * inv_deltaUV;

  Mat4x4d dF_dF1 = kronecker<2, 2, 2, 2>(inv_deltaUV.transpose(), I_two);;
  Mat4x9d dF_dx = dF_dF1 * dF1_dx;
//
//

  {
    Mat3x2d newdeltaUV2;
    Mat3x2d p;
    p.col(0) = p1_vec3(x_vec) - p0_vec3(x_vec);
    p.col(1) = p2_vec3(x_vec) - p0_vec3(x_vec);
    newdeltaUV2.col(0) = p.col(0).normalized();
    newdeltaUV2.col(1) = (p.col(1) - p.col(1).dot(newdeltaUV.col(0)) * newdeltaUV.col(0)).normalized();


    Mat2x2d F2 = newdeltaUV2.transpose() * getDeformationGradient(x_vec);
    std::printf("F2 F diff: %.10f\n", (F2 - F).norm());
    std::printf("newdeltauv diff: %.10f\n", (newdeltaUV2 - newdeltaUV).norm());


  }

  //forward  F --> R
  Eigen::JacobiSVD<Mat2x2d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat2x2d R = svd.matrixU() * svd.matrixV().transpose();

  // backward R--> F
  Mat2x2d S = svd.matrixV() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
  Vec4d lhs(-R(0, 1), -R(1, 1), R(0, 0), R(1, 0));
  Vec4d x = lhs / S.trace();
  Mat4x4d dR_dF = lhs * x.transpose();

  Mat4x9d dR_dx = dR_dF * dF_dx; //checked



  Mat3x2d F_prime = newdeltaUV * R;

  Mat6x6d dF_prime_dnewdeltaUV;
  Mat6x4d dF_prime_dR;
  dF_prime_dnewdeltaUV.setZero();
  dF_prime_dR.setZero();
  dF_prime_dnewdeltaUV = kronecker<2, 2, 3, 3>(R.transpose(), I_three);
  dF_prime_dR.block<3, 2>(0, 0) = newdeltaUV;
  dF_prime_dR.block<3, 2>(3, 2) = newdeltaUV;

  Mat6x9d dF_prime_dx = dF_prime_dnewdeltaUV * dnewdeltaUV_dx + dF_prime_dR * dR_dx;
//      return dF_prime_dx;

  std::printf("%.10f %.10f %.10f %.10f\n", df_db.norm(), db_dx.norm(), df_daNorm.norm(), daNorm_dx.norm());
  return std::make_pair(dF_prime_dx, F_prime);
}

Triangle::Triangle(int p0_idx, int p1_idx, int p2_idx, std::vector<Particle> &pArr, bool checkArea) : Constraint(
        Constraint::ConstraintType::CONSTRAINT_TRIANGLE, 6), p0_idx(p0_idx), p1_idx(p1_idx), p2_idx(p2_idx),
                                                                                                      pArr(pArr),
                                                                                                      color(0, 0, 0),
                                                                                                      overrideColor(
                                                                                                              false),
                                                                                                      energyType(
                                                                                                              QUADRATIC) {

  assert(p0_idx == p0()->idx);
  assert(p1_idx == p1()->idx);
  assert(p2_idx == p2()->idx);

  idxArr.emplace_back(p0_idx);
  idxArr.emplace_back(p1_idx);
  idxArr.emplace_back(p2_idx);

  for (int i = 0; i < 3; i++) {
    int id1 = idxArr[i];
    int id2 = idxArr[(i + 1) % 3];
    int id3 = idxArr[(i + 2) % 3];

    double angle = getAngleABCInDegreeBetween(pArr[id1].pos_rest, pArr[id2].pos_rest,
                                              pArr[id3].pos_rest) * 180.0 / 3.14159265;
    if (angle < 20) {
      std::printf("WARNING: MESH TOPOLOGY: angle between point %d-%d-%d is %.2f degree\n", id1,
                  id2, id3, angle);
    }
  }

  I_two.setIdentity();
  I_three.setIdentity();
  // material space data precomputation
  deltaUV.setZero();
  inv_deltaUV.setZero();

  Mat3x2d edgeVec;
  edgeVec.col(0) = p1()->pos_rest - p0()->pos_rest;
  edgeVec.col(1) = p2()->pos_rest - p0()->pos_rest;
  P.col(0) = edgeVec.col(0).normalized();
  P.col(1) = (edgeVec.col(1) - edgeVec.col(1).dot(P.col(0)) * P.col(0)).normalized();

  deltaUV = P.transpose() * edgeVec;
  inv_deltaUV = deltaUV.inverse(); // == rest
  area_rest = std::abs(deltaUV.determinant() * 0.5);
  if (checkArea) {
    if (area_rest < 0.001) {
      std::printf("WARNING: MESH TOPOLOGY: Triangle (%d, %d, %d) has area %.4f\n", p0_idx, p1_idx, p2_idx, area_rest);
    }
  }


  Mat2x3d p;
  p.row(0) = Vec3d(-1, 1, 0);
  p.row(1) = Vec3d(-1, 0, 1);
  dF_dx = kronecker<2, 3, 3, 3>(inv_deltaUV.transpose() * p, I_three);
  setConstraintWeight();
  normal = Triangle::getNormal(p0()->pos, p1()->pos, p2()->pos);
}



Triangle &Triangle::operator=(const Triangle &other) {
  pArr = other.pArr;
  newF = other.newF;
  idxArr = other.idxArr;
  p0_idx = other.p0_idx;
  p1_idx = other.p1_idx;
  p2_idx = other.p2_idx;
  color = other.color;
  normal = other.normal;
  overrideColor = other.overrideColor;
  stretchingForceBuffer = other.stretchingForceBuffer;
  dfi_dk_buffer = other.dfi_dk_buffer;
  energyType =other.energyType;
  P = other.P;
  I_two = other.I_two;
  rest = other.rest;
  I_three = other.I_three;
  deltaUV = other.deltaUV;
  inv_deltaUV = other.inv_deltaUV;
  hessian_buffer = other.hessian_buffer;
  area_rest = other.area_rest;
  constrainWeightSqrt = other.constrainWeightSqrt;
  dF_dx = other.dF_dx;
  return *this;
}





