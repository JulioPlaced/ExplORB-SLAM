//
// Created by jjgomez on 25/04/22.
//

#include "PinHole.h"

#define fx vParameters_[0]
#define fy vParameters_[1]
#define cx vParameters_[2]
#define cy vParameters_[3]

void PinHole::project(const Eigen::Vector3f& p3D, Eigen::Vector2f& p2D){
       p2D(0) = fx * p3D(0) / p3D(2) + cx;
    p2D(1) = fy * p3D(1) / p3D(2) + cy;
}

void PinHole::unproject(const Eigen::Vector2f& p2D, Eigen::Vector3f& p3D) {
    p3D(0) = (p2D(0) - cx) / fx;
    p3D(1) = (p2D(1) - cy) / fy;
    p3D(2) = 1.f;
}

void PinHole::projectJac(const Eigen::Vector3f& p3D, Eigen::Matrix<float,2,3>& Jac) {
    Jac(0,0) = fx / p3D(2);
    Jac(0,1) = 0.f;
    Jac(0,2) = -fx * p3D(0) / (p3D(2) * p3D(2));

    Jac(1,0) = 0.f;
    Jac(1,1) = fy / p3D(2);
    Jac(1,2) = -fy * p3D(1) / (p3D(2) * p3D(2));
}
