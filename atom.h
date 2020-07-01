#ifndef ATOM_H
#define ATOM_H

#include "chai3d.h"

using namespace std;
using namespace chai3d;

// Radius of each sphere
const double SPHERE_RADIUS = 0.008;
const double SPHERE_MASS = 0.02;

class Atom : public cShapeSphere {
private:
    bool anchor;
    bool current;
    bool repeating;
    cVector3d velocity;
    cVector3d force;
    cShapeLine *velVector;
    double sphere_mass;
    cColorf baseColor;

public:
    Atom(double radius, double sphere_mass, cColorf color = cColorf());
    bool isAnchor();
    void setAnchor(bool newAnchor);
    bool isRepeating();
    void setRepeating(bool newRepeat);
    bool isCurrent();
    void setCurrent(bool newCurrent);
    cVector3d getVelocity();
    void setVelocity(cVector3d newVel);
    cVector3d getForce();
    void setForce(cVector3d newForce);
    cShapeLine *getVelVector();
    void setVelVector(cShapeLine *newVelVector);
    void updateVelVector();
    void setInitialPosition(double spawn_dist = .1);
    double getMass();
    void setColor(cColorf color);
};

#endif  // ATOM_H
