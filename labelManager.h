#ifndef LABEL_MANAGER_H
#define LABEL_MANAGER_H

#include "chai3d.h"

class LabelManager{
private:

    chai3d::cLabel* energyNum;

public:
    LabelManager(chai3d::cCamera *camera);
    void updateEnergyNum(double potentialEnergy);


};

#endif
