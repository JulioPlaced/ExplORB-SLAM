/**
* Created by jjgomez on 22/04/22.
*/

#include "include/HessianComputation.h"
#include "include/Converter.h"

#include <Eigen/LU>
#include <set>
#include <Eigen/Geometry>

using namespace std;

typedef long unsigned int ID;

/** Intersection and union function for unordered containers which support a fast lookup function find()
 *  Return values are moved by move-semantics, for c++11/c++14 this is efficient, otherwise it results in a copy
 */

namespace unorderedHelpers {

    template<typename UnorderedIn1, typename UnorderedIn2,
            typename UnorderedOut = UnorderedIn1>
    UnorderedOut makeIntersection(const  UnorderedIn1 &in1, const  UnorderedIn2 &in2)
    {
        if (in2.size() < in1.size()) {
            return makeIntersection<UnorderedIn2,UnorderedIn1,UnorderedOut>(in2, in1);
        }

        UnorderedOut out;
        auto e = in2.end();
        for(auto & v : in1)
        {
            if (in2.find(v) != e){
                out.insert(v);
            }
        }
        return out;
    }

    template<typename UnorderedIn1, typename UnorderedIn2,
            typename UnorderedOut = UnorderedIn1>
    UnorderedOut makeUnion(const UnorderedIn1 &in1, const UnorderedIn2 &in2)
    {
        UnorderedOut out;
        out.insert(in1.begin(), in1.end());
        out.insert(in2.begin(), in2.end());
        return out;
    }
}

namespace ORB_SLAM2{
    KFset getOrderedKFset(std::set<KeyFrame*> s){
        KFset orderedSet(s.begin(),s.end());
        return orderedSet;
    }

    MPset getOrderedMPset(std::set<MapPoint*> s){
        MPset orderedSet(s.begin(),s.end());
        return orderedSet;
    }

    SE3 cvPoseToEigen(cv::Mat& T){
        SE3 eigenT = SE3::Zero();

        eigenT(0,0) = T.at<float>(0,0); eigenT(0,1) = T.at<float>(0,1); eigenT(0,2) = T.at<float>(0,2); eigenT(0,3) = T.at<float>(0,3);
        eigenT(1,0) = T.at<float>(1,0); eigenT(1,1) = T.at<float>(1,1); eigenT(1,2) = T.at<float>(1,2); eigenT(1,3) = T.at<float>(1,3);
        eigenT(2,0) = T.at<float>(2,0); eigenT(2,1) = T.at<float>(2,1); eigenT(2,2) = T.at<float>(2,2); eigenT(2,3) = T.at<float>(2,3);

        return eigenT;
    }

    Eigen::Vector3f cvPointToEigen(cv::Mat& x){
        Eigen::Vector3f eigenX;

        eigenX(0) = x.at<float>(0);
        eigenX(1) = x.at<float>(1);
        eigenX(2) = x.at<float>(2);

        return eigenX;
    }

    Eigen::Matrix3f rotationMatrix(SE3& T){
        return T.block<3,3>(0,0);
    }

    Eigen::Vector3f composePosePoint(SE3& Tcw, Eigen::Vector3f& x){
        return rotationMatrix(Tcw) * x + Tcw.block<3,1>(0,3);
    }

    MPJac mapPointJacobian(SE3& Tcw, Eigen::Vector3f& xw, PinHole& calibration){
        Eigen::Vector3d xc = composePosePoint(Tcw, xw).cast<double>();
        Eigen::Matrix<double,2,3> projectJac = calibration.projectJac(xc);
        Eigen::Matrix<double,2,3> Jac = -projectJac * rotationMatrix(Tcw).cast<double>();

        return Jac;
    }

    KFJac keyFrameJacobian(SE3& Tcw, Eigen::Vector3f& xw, PinHole& calibration){
        Eigen::Vector3d xc = composePosePoint(Tcw, xw).cast<double>();
        Eigen::Matrix<double,2,3> projectJac = calibration.projectJac(xc);

        double x = xc[0];
        double y = xc[1];
        double z = xc[2];

        Eigen::Matrix<double,3,6> SE3deriv;
        SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
                -z , 0.f, x, 0.f, 1.f, 0.f,
                y ,  -x , 0.f, 0.f, 0.f, 1.f;

        Eigen::Matrix<double,2,6> Jac = -projectJac * SE3deriv;

        return Jac;
    }

    int getKeyFramesAndMapPoints(Map* pMap, KFset& sKeyFrames, MPset& sMapPoints, ObsMap& obs, const int TH){
        //Essential graph threshold
        //const int TH = 100;

        int nObs = 0;

        vector<KeyFrame*> vKeyFrames = pMap->GetAllKeyFrames();
        //sKeyFrames = KFset(vKeyFrames.begin(),vKeyFrames.end());

        //Get MapPoints
        for(KeyFrame* pKFcurr : vKeyFrames){
            ID currID = pKFcurr->mnId;

            //Get set of MapPoints for the current KeyFrame
            MPset sCurrMPs = getOrderedMPset(pKFcurr->GetMapPoints());
            MPset sCurrMPsUsed;

            //Get covisible KeyFrames
             vector<KeyFrame*> vConnectedKFs = pKFcurr->GetVectorCovisibleKeyFrames();
            //vector<KeyFrame*> vConnectedKFs = pKFcurr->GetCovisiblesByWeight(TH);

            int connected = 0;

            for(KeyFrame* pKFother : vConnectedKFs){
                ID otherID = pKFother->mnId;
                if(currID == otherID)
                    continue;

                /*int NcommonObs = pKFcurr->GetWeight(pKFother);
                if(NcommonObs < TH)
                    continue;*/

                //Get set of MapPoints for the other KeyFrame
                MPset sOtherMPs = getOrderedMPset(pKFother->GetMapPoints());

                //Get common points between them
                MPset sIntersection;
                sIntersection = unorderedHelpers::makeIntersection(sCurrMPs,sOtherMPs);

                if(sIntersection.size() < TH)
                  continue;

                  connected++;

                //Add to the global set of MapPoints
                sMapPoints = unorderedHelpers::makeUnion(sMapPoints,sIntersection);

                sCurrMPsUsed = unorderedHelpers::makeUnion(sCurrMPsUsed,sIntersection);

            }

            if(connected == 0)
              continue;

            sKeyFrames.insert(pKFcurr);
            // cout << "KF " << currID << " has " << connected << " connected KFs" << endl;
            nObs += sCurrMPsUsed.size();
            obs[pKFcurr] = sCurrMPsUsed;
        }

        return nObs;
    }

    void insertMapPointBlock(int baseColBlockIdx, int baseRowBlockIdx,
                             Triplets& entries, MPJac block){
        for(size_t r = 0; r < DoF_obs; r++){
            for(size_t c = 0; c < DoF_MP; c++){
                entries.push_back(Triplet(baseRowBlockIdx + r, baseColBlockIdx + c, block(r,c)));
            }
        }
    }

    void insertKeyFrameBlock(int baseColBlockIdx, int baseRowBlockIdx,
                             Triplets& entries, KFJac block){
        for(size_t r = 0; r < DoF_obs; r++){
            for(size_t c = 0; c < DoF_KF; c++){
                entries.push_back(Triplet(baseRowBlockIdx + r, baseColBlockIdx + c, block(r,c)));
            }
        }
    }

    void insertWBlock(int baseColBlockIdx, int baseRowBlockIdx,
                      Triplets& entries, double w){
        entries.push_back(Triplet(baseColBlockIdx,baseColBlockIdx,w));
        entries.push_back(Triplet(baseColBlockIdx + 1,baseColBlockIdx + 1,w));
    }

    Eigen::SparseMatrix<double> computeHessian(const KFset& sKeyFrames, const MPset& sMapPoints,
                                               ObsMap& obs, const int nObs, std::map<long unsigned int,size_t>& lookUpTable){

        const int nRows = 2 * nObs;
        const int nCols = DoF_KF * sKeyFrames.size() + DoF_MP * sMapPoints.size();

        //Build translation table from KeyFrame and MapPoint IDs to Jacobian indices
        map<ID,size_t> mTranslationKF;
        map<ID,size_t> mTranslationMP;

        int currKfBlockIdx = 0;
        int currMpBlockIdx = DoF_KF * sKeyFrames.size();

        int currObsBlockIdx = 0;

        //GeometricCamera* calibration = (*sKeyFrames.begin())->mpCamera;
        vector<float> vCalib = {(*sKeyFrames.begin())->fx, (*sKeyFrames.begin())->fy, (*sKeyFrames.begin())->cx, (*sKeyFrames.begin())->cy};
        PinHole calibration(vCalib);

        Triplets JacEntries;
        Triplets WEntries;

        lookUpTable.clear();

        for(KeyFrame* pKF : sKeyFrames){
            lookUpTable[pKF->mnId] = currKfBlockIdx / DoF_KF;

            //Add to the KeyFrame translation table
            int kfBlockIdx = currKfBlockIdx;
            mTranslationKF[pKF->mnId] = currKfBlockIdx;
            currKfBlockIdx += DoF_KF;

            //Get KeyFrame's MapPoints
            //MPset sMPsOfKF = getOrderedMPset(pKF->GetMapPoints());
            MPset sMPsOfKF = obs[pKF];

            //Sophus::SE3f Tcw = pKF->GetPose();
            cv::Mat cvTcw = pKF->GetPose();
            SE3 Tcw = cvPoseToEigen(cvTcw);

            //Consider only valid MapPoints
            for(MapPoint* pMP : sMPsOfKF){
                if(!sMapPoints.count(pMP))  //Not valid
                    continue;

                //Get MapPoint position at the Jacobian
                int mpBlockIdx;
                if(!mTranslationMP.count(pMP->mnId)){   //MapPoint has not been used yet
                    mTranslationMP[pMP->mnId] = currMpBlockIdx;
                    mpBlockIdx = currMpBlockIdx;
                    currMpBlockIdx += DoF_MP;
                }
                else{   //MapPoint has already been used
                    mpBlockIdx = mTranslationMP[pMP->mnId];
                }

                //Get MapPoint 3D world position
                //Eigen::Vector3f xw = pMP->GetWorldPos();
                cv::Mat xwCv = pMP->GetWorldPos();
                Eigen::Vector3f xw = cvPointToEigen(xwCv);

                //Compute Jacobian with respect the map point
                MPJac xwJac = mapPointJacobian(Tcw,xw,calibration);

                //Insert it to the global Jacobian
                insertMapPointBlock(mpBlockIdx,currObsBlockIdx,JacEntries,xwJac);

                //Compute Jacobian with respect the camera pose
                KFJac TcwJac = keyFrameJacobian(Tcw,xw,calibration);

                // if(is_nan(TcwJac)){
                //   std::cout << Tcw << '\n';
                //   std::cout << xw << '\n';
                // }
                // if(is_nan(xwJac)){
                //   std::cout << Tcw << '\n';
                //   std::cout << xw << '\n';
                // }

                //Insert it to the global Jacobian
                insertKeyFrameBlock(kfBlockIdx,currObsBlockIdx,JacEntries,TcwJac);

                //Compute W values
                int idxInKF = pMP->GetIndexInKeyFrame(pKF);
                int octave = pKF->mvKeysUn[idxInKF].octave;
                double w = pKF->mvInvLevelSigma2[octave];

                //Insert block to the global W matrix
                insertWBlock(currObsBlockIdx,currObsBlockIdx,WEntries,w);

                currObsBlockIdx += 2;
            }
        }

        //Build Sparse Jacobian
        Eigen::SparseMatrix<double> J(nRows,nCols);
        J.setFromTriplets(JacEntries.begin(),JacEntries.end());

        Eigen::SparseMatrix<double> W(nRows, nRows);
        W.setFromTriplets(WEntries.begin(), WEntries.end());

        //Compute Hessian
        Eigen::SparseMatrix<double> H = J.transpose()*W*J;

        return H;
    }

    Eigen::SparseMatrix<double> HppInverse(Eigen::SparseMatrix<double>& Hpp){
        const int Np = Hpp.cols() / DoF_MP;

        Triplets entries;

        for(size_t i = 0; i < (size_t)Np; i++){
            const size_t start = DoF_MP * i;
            Eigen::Matrix3d blockInverse = Hpp.block(start,start,DoF_MP,DoF_MP).toDense().inverse();

            for(size_t r = 0; r < DoF_MP; r++){
                for(size_t c = 0; c < DoF_MP; c++){
                    entries.push_back(Triplet(start+r,start+c,blockInverse(r,c)));
                }
            }
        }

        Eigen::SparseMatrix<double> HppInverse(Hpp.rows(),Hpp.cols());
        HppInverse.setFromTriplets(entries.begin(),entries.end());

        return HppInverse;
    }

    void decomposeHinBlocks(const int Ncameras, Eigen::SparseMatrix<double>& H, Eigen::SparseMatrix<double>& Hcc,
                            Eigen::SparseMatrix<double>& Hpp, Eigen::SparseMatrix<double>& Hcp){
        const int Nh = H.cols();
        const int Hcc_size = Ncameras * DoF_KF;
        const int Hpp_size = Nh - Hcc_size;
        Hcc = H.block(0,0,Hcc_size,Hcc_size);
        Hpp = H.block(Hcc_size,Hcc_size,Hpp_size,Hpp_size);
        Hcp = H.block(Hcc_size,0,Hpp_size,Hcc_size);
    }

    Eigen::SparseMatrix<double> computeReducedCameraSystem(const int Nkf, const int Nmp,
                                                           Eigen::SparseMatrix<double>& H){
        Eigen::SparseMatrix<double> Hcc, Hpp, Hcp;
        decomposeHinBlocks(Nkf,H,Hcc,Hpp,Hcp);

        //Hred = Hcc - Hcp * Hpp⁻1 * Hcp^t
        Eigen::SparseMatrix<double> Hreduced = Hcc - Hcp.transpose() * HppInverse(Hpp) * Hcp;

        return Hreduced;
    }

    void writeToSystem(Eigen::SparseMatrix<double>& H, KFset sKeyFrames, std::map<long unsigned int,size_t>& lookUpTable, MPset mPointSet, const int TH, System* pSystem){

        list<float> lVertices, lEdges, lMapPoints;

        for(KeyFrame* pKF : sKeyFrames){
            //Save KeyFrames IDs and poses
            long unsigned int ID = pKF->mnId;
            cv::Mat cvPose = pKF->GetPose();
            SE3 Tcw = cvPoseToEigen(cvPose);
            Eigen::Quaternionf q(rotationMatrix(Tcw));
            Eigen::Vector3f t = Tcw.block<3,1>(0,3);

            lVertices.push_back(ID); lVertices.push_back(q.w()); lVertices.push_back(q.x()); lVertices.push_back(q.y());
            lVertices.push_back(q.z()); lVertices.push_back(t.x()); lVertices.push_back(t.y()); lVertices.push_back(t.z());

            //Save H taking into account the Essential Graph connections
            // vector<KeyFrame*> vConnectedKFs = pKF->GetCovisiblesByWeight(TH);
            vector<KeyFrame*> vConnectedKFs = pKF->GetVectorCovisibleKeyFrames();


            for(KeyFrame* pKFother : vConnectedKFs){
                long unsigned int otherID = pKFother->mnId;
                if(lookUpTable.count(otherID)){
                    //Entry must be written to file
                    size_t startRow = lookUpTable[ID] * DoF_KF;
                    size_t startCol = lookUpTable[otherID] * DoF_KF;
                    Eigen::Matrix<double,6,6> Hcams = H.block(startRow,startCol,6,6).toDense();

                    lEdges.push_back(ID); lEdges.push_back(otherID);
                    lEdges.push_back(Hcams(0,0)); lEdges.push_back(Hcams(0,1)); lEdges.push_back(Hcams(0,2)); lEdges.push_back(Hcams(0,3)); lEdges.push_back(Hcams(0,4)); lEdges.push_back(Hcams(0,5));
                    lEdges.push_back(Hcams(1,0)); lEdges.push_back(Hcams(1,1)); lEdges.push_back(Hcams(1,2)); lEdges.push_back(Hcams(1,3)); lEdges.push_back(Hcams(1,4)); lEdges.push_back(Hcams(1,5));
                    lEdges.push_back(Hcams(2,0)); lEdges.push_back(Hcams(2,1)); lEdges.push_back(Hcams(2,2)); lEdges.push_back(Hcams(2,3)); lEdges.push_back(Hcams(2,4)); lEdges.push_back(Hcams(2,5));
                    lEdges.push_back(Hcams(3,0)); lEdges.push_back(Hcams(3,1)); lEdges.push_back(Hcams(3,2)); lEdges.push_back(Hcams(3,3)); lEdges.push_back(Hcams(3,4)); lEdges.push_back(Hcams(3,5));
                    lEdges.push_back(Hcams(4,0)); lEdges.push_back(Hcams(4,1)); lEdges.push_back(Hcams(4,2)); lEdges.push_back(Hcams(4,3)); lEdges.push_back(Hcams(4,4)); lEdges.push_back(Hcams(4,5));
                    lEdges.push_back(Hcams(5,0)); lEdges.push_back(Hcams(5,1)); lEdges.push_back(Hcams(5,2)); lEdges.push_back(Hcams(5,3)); lEdges.push_back(Hcams(5,4)); lEdges.push_back(Hcams(5,5));

                }
            }
        }

        for(MapPoint* pMP : mPointSet){
            lMapPoints.push_back(pMP->mnId);

            cv::Mat xwcv = pMP->GetWorldPos();
            Eigen::Vector3d xw = Converter::toVector3d(xwcv);

            lMapPoints.push_back(xw.x()); lMapPoints.push_back(xw.y()); lMapPoints.push_back(xw.z());

            map<KeyFrame*,size_t> mpObs = pMP->GetObservations();
            for(auto kfAndIdx : mpObs){
               lMapPoints.push_back(kfAndIdx.first->mnId);
            }
            lMapPoints.push_back(-1);
        }

        pSystem->saveVertex(lVertices);
        pSystem->saveEdges(lEdges);
        pSystem->saveMapPoints(lMapPoints);
    }

    void computeFactorGraph(Map* pMap,const int TH, System* pSystem){
        //Get KeyFrames and MapPoints
        KFset sKeyFrames;
        MPset sMapPoints;
        ObsMap obs;
        int nObs = getKeyFramesAndMapPoints(pMap,sKeyFrames,sMapPoints,obs,TH);
        if(nObs < 1)
            return;

        map<long unsigned int,size_t> lookUpTable;
        Eigen::SparseMatrix<double> H = computeHessian(sKeyFrames,sMapPoints,obs,nObs,lookUpTable);
        Eigen::SparseMatrix<double> Hreduced = computeReducedCameraSystem(sKeyFrames.size(),sMapPoints.size(), H);

        writeToSystem(Hreduced, sKeyFrames, lookUpTable, sMapPoints, TH,pSystem);

    }

}
