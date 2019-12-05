#ifndef LABEL_MANAGER_H
#define LABEL_MANAGER_H

#include "chai3d.h"

class LabelManager{
private:

    // a label to show the potential energy
    chai3d::cLabel* energyNum;

    // a label showing the number of anchored atoms
    chai3d::cLabel* numAnchored;

    chai3d::cLabel* isFrozen;

public:
    LabelManager(chai3d::cCamera *camera, int width);
    void updateEnergyNum(double potentialEnergy);
    void updateNumAnchored(int anchored, int total, int width);
    void updateFrozen(int width, bool freezeAtoms);

};

#endif
