#ifndef BOUNDARIES_H
#define BOUNDARIES_H

#include "chai3d.h"

const double BOUNDARY_LIMIT = .5;

// boundary conditions
const cVector3d northPlanePos = cVector3d(0, BOUNDARY_LIMIT, 0);
const cVector3d northPlaneP1 = cVector3d(1, BOUNDARY_LIMIT, 0);
const cVector3d northPlaneP2 = cVector3d(1, BOUNDARY_LIMIT, 1);
const cVector3d northPlaneNorm =
    cComputeSurfaceNormal(northPlanePos, northPlaneP1, northPlaneP2);
const cVector3d southPlanePos = cVector3d(0, -BOUNDARY_LIMIT, 0);
const cVector3d southPlaneP1 = cVector3d(1, -BOUNDARY_LIMIT, 0);
const cVector3d southPlaneP2 = cVector3d(1, -BOUNDARY_LIMIT, 1);
const cVector3d southPlaneNorm =
    cComputeSurfaceNormal(southPlanePos, southPlaneP1, southPlaneP2);
const cVector3d eastPlanePos = cVector3d(BOUNDARY_LIMIT, 0, 0);
const cVector3d eastPlaneP1 = cVector3d(BOUNDARY_LIMIT, 1, 0);
const cVector3d eastPlaneP2 = cVector3d(BOUNDARY_LIMIT, 1, 1);
const cVector3d eastPlaneNorm =
    cComputeSurfaceNormal(eastPlanePos, eastPlaneP1, eastPlaneP2);
const cVector3d westPlanePos = cVector3d(-BOUNDARY_LIMIT, 0, 0);
const cVector3d westPlaneP1 = cVector3d(-BOUNDARY_LIMIT, 1, 0);
const cVector3d westPlaneP2 = cVector3d(-BOUNDARY_LIMIT, 1, 1);
const cVector3d westPlaneNorm =
    cComputeSurfaceNormal(westPlanePos, westPlaneP1, westPlaneP2);
const cVector3d forwardPlanePos = cVector3d(0, 0, BOUNDARY_LIMIT);
const cVector3d forwardPlaneP1 = cVector3d(0, 1, BOUNDARY_LIMIT);
const cVector3d forwardPlaneP2 = cVector3d(1, 1, BOUNDARY_LIMIT);
const cVector3d forwardPlaneNorm =
    cComputeSurfaceNormal(forwardPlanePos, forwardPlaneP1, forwardPlaneP2);
const cVector3d backPlanePos = cVector3d(0, 0, -BOUNDARY_LIMIT);
const cVector3d backPlaneP1 = cVector3d(0, 1, -BOUNDARY_LIMIT);
const cVector3d backPlaneP2 = cVector3d(1, 1, -BOUNDARY_LIMIT);
const cVector3d backPlaneNorm =
    cComputeSurfaceNormal(backPlanePos, backPlaneP1, backPlaneP2);


bool checkBounds(chai3d::cVector3d location);


#endif
