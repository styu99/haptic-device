#ifndef CAMERA_H
#define CAMERA_H

#include "chai3d.h"

class CameraManager{
private:
    chai3d::cCamera* camera;
    double rho;


public:
    CameraManager(chai3d::cCamera* cam);

    double getRho();
    void setRho(double val);
};



#endif
