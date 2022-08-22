//
// Created by jjgomez on 25/04/22.
//

#ifndef ORB_SLAM2_PINHOLE_H
#define ORB_SLAM2_PINHOLE_H

#include <Eigen/Core>
#include <vector>
#include <opencv2/opencv.hpp>

class PinHole {
public:
    PinHole() {}

    /*
     * Constructor with calibration parameters
     */
    PinHole(const std::vector<float> &_vParameters) : vParameters_(_vParameters) {}

    /*
     * Projects a given 3D point into the image
     */
    void project(const Eigen::Vector3f& p3D, Eigen::Vector2f& p2D);

    /*
     * Unprojects a given image 2D point into it is bearing ray
     */
    void unproject(const Eigen::Vector2f& p2D, Eigen::Vector3f& p3D);

    /*
     * Analytic Jacobian of the projection function (stored by rows)
     */
    void projectJac(const Eigen::Vector3f& p3D, Eigen::Matrix<float,2,3>& Jac);

    //-----------------------------------------------------------------
    //Useful overloadings to allow the use of different data structures
    //-----------------------------------------------------------------
    cv::Point2f project(Eigen::Vector3f & X){
        Eigen::Vector2f uv;

        this->project(X.cast<float>(),uv);

        cv::Point2f toReturn(uv(0),uv(1));
        return toReturn;
    }

    Eigen::Vector2d project(Eigen::Vector3d & X){
        Eigen::Vector2f uv;

        this->project(X.cast<float>(),uv);

        return uv.cast<double>();
    }

    Eigen::Matrix<float,1,3> unproject(const float u, const float v){
        Eigen::Vector2f uv(u,v);
        Eigen::Vector3f ray;

        this->unproject(uv,ray);

        return ray;
    }

    Eigen::Matrix<float,1,3> unproject(Eigen::Vector2f& p2D){
        Eigen::Vector3f ray;

        this->unproject(p2D,ray);

        return ray;
    }

    Eigen::Matrix<float,1,3> unproject(cv::Point2f puv){
        Eigen::Vector2f uv(puv.x,puv.y);
        Eigen::Vector3f ray;

        this->unproject(uv,ray);

        return ray;
    }

    Eigen::Matrix<float,2,3> projectJac(Eigen::Vector3f &p3D){
        Eigen::Matrix<float,2,3> jac;

        this->projectJac(p3D,jac);

        return jac;
    }

    Eigen::Matrix<double,2,3> projectJac(Eigen::Vector3d &p3D){
        Eigen::Matrix<float,2,3> jac;

        this->projectJac(p3D.cast<float>(),jac);

        return jac.cast<double>();
    }
private:
    std::vector<float> vParameters_;    //Vector of calibration parametets

};


#endif //ORB_SLAM2_PINHOLE_H
