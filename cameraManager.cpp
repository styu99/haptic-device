#include "cameraManager.h"



CameraManager::CameraManager(chai3d::cCamera* cam){
    camera = cam;

    // radius of the camera
    rho = 0.35;

    // creates the radius, origin reference, along with the zenith and azimuth
    // direction vectors
    chai3d::cVector3d origin(0.0, 0.0, 0.0);
    chai3d::cVector3d zenith(0.0, 0.0, 1.0);
    chai3d::cVector3d azimuth(1.0, 0.0, 0.0);

    // sets the camera's references of the origin, zenith, and azimuth
    camera->setSphericalReferences(origin, zenith, azimuth);

    // sets the camera's position to have a radius of .1, located at 0 radians
    // (vertically and horizontally)
    camera->setSphericalRad(rho, 0, 0);

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    // TODO: change?
    camera->setStereoMode(chai3d::C_STEREO_DISABLED);

    // set stereo eye separation and focal length (applies only if stereo is
    // enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(1.8);

    // set vertical mirrored display mode
    camera->setMirrorVertical(false);
}

double CameraManager::getRho(){
    return rho;
}

void CameraManager::setRho(double val){
    rho = val;
}
