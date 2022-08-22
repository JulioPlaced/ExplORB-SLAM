/**
* Created by jjgomez on 22/04/22.
*/

#ifndef ORB_SLAM2_ROS_HESSIANCOMPUTATION_H
#define ORB_SLAM2_ROS_HESSIANCOMPUTATION_H

#include "Map.h"
#include "PinHole.h"
#include "System.h"

#include <Eigen/Sparse>
#include <unordered_set>

namespace ORB_SLAM2{

    template<typename Derived>
    inline bool is_nan(const Eigen::MatrixBase<Derived>& x){
        //return ((x.array() != x.array())).all();
        return !x.allFinite();
    }

    const int DoF_KF = 6;
    const int DoF_MP = 3;
    const int DoF_obs = 2;

    struct KeyFrameComparator {
        bool operator()(KeyFrame* pKF1, KeyFrame* pKF2) const {
            return pKF1->mnId < pKF2->mnId;
        }
    };

    struct MapPointComparator {
        bool operator()(MapPoint* pMP1, MapPoint* pMP2) const {
            return pMP1->mnId < pMP2->mnId;
        }
    };

    typedef Eigen::Matrix<float,3,4> SE3;

    SE3 cvPoseToEigen(cv::Mat& T);

    Eigen::Vector3f cvPointToEigen(cv::Mat& x);

    Eigen::Matrix3f rotationMatrix(SE3& T);

    Eigen::Vector3f composePosePoint(SE3& Tcw, Eigen::Vector3f& x);

    typedef std::unordered_set<ORB_SLAM2::KeyFrame*> KFset;
    typedef std::unordered_set<ORB_SLAM2::MapPoint*> MPset;
    typedef std::unordered_map<ORB_SLAM2::KeyFrame*,MPset> ObsMap;

    typedef Eigen::Matrix<double,DoF_obs,DoF_MP> MPJac;
    typedef Eigen::Matrix<double,DoF_obs,DoF_KF> KFJac;
    typedef Eigen::Triplet<double> Triplet;
    typedef std::vector<Triplet> Triplets;

    KFset getOrderedKFset(std::set<KeyFrame*> s);

    MPset getOrderedMPset(std::set<MapPoint*> s);

    MPJac mapPointJacobian(SE3& Tcw, Eigen::Vector3f& xw, PinHole& calibration);
    KFJac keyFrameJacobian(SE3& Tcw, Eigen::Vector3f& xw, PinHole& calibration);

    Eigen::SparseMatrix<double> HppInverse(Eigen::SparseMatrix<double>& Hpp);

    void insertMapPointBlock(int baseColBlockIdx, int baseRowBlockIdx,
                             Triplets& entries, MPJac block);

    void insertKeyFrameBlock(int baseColBlockIdx, int baseRowBlockIdx,
                             Triplets& entries, KFJac block);

    void insertWBlock(int baseColBlockIdx, int baseRowBlockIdx,
                      Triplets& entries, double w);

    int getKeyFramesAndMapPoints(Map* pMap, KFset& sKeyFrames, MPset& sMapPoints,
                                 ObsMap& obs, const int TH);

    Eigen::SparseMatrix<double> computeHessian(const KFset& sKeyFrames, const MPset& sMapPoints,
                                               ObsMap& obs, const int nObs, std::map<long unsigned int,size_t>& lookUpTable);

    void decomposeHinBlocks(const int Ncameras, Eigen::SparseMatrix<double>& H, Eigen::SparseMatrix<double>& Hcc,
                            Eigen::SparseMatrix<double>& Hpp, Eigen::SparseMatrix<double>& Hcp);

    Eigen::SparseMatrix<double> computeReducedCameraSystem(const int Nkf, const int Nmp,
                                                           Eigen::SparseMatrix<double>& H);

    void writeToSystem(Eigen::SparseMatrix<double>& H, KFset sKeyFrames, std::map<long unsigned int,size_t>& lookUpTable, MPset mPoints, const int TH,
                     System* pSystem);

    void computeFactorGraph(Map* pMap, const int TH, System* pSystem);
}

#endif //ORB_SLAM2_ROS_HESSIANCOMPUTATION_H
