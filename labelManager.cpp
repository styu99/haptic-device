#include "labelManager.h"
#include "utility.h"

LabelManager::LabelManager(chai3d::cCamera* camera){

    // potential energy label
    addLabel(energyNum, camera);
    energyNum->setLocalPos(0, 15, 0);
    std::cout << "Initialized" << std::endl;

}

void LabelManager::updateEnergyNum(double potentialEnergy){
    energyNum->setText("Potential Energy: " + chai3d::cStr((potentialEnergy / 2), 5));

}
