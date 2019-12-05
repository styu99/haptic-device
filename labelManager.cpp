#include "labelManager.h"
#include "utility.h"
#include <string>


LabelManager::LabelManager(chai3d::cCamera* camera, int width){

    // potential energy label
    addLabel(energyNum, camera);
    energyNum->setLocalPos(0, 15, 0);

    addLabel(numAnchored, camera);

    // add and initialize frozen state label
    addLabel(isFrozen, camera);
    updateFrozen(width, false);
}

void LabelManager::updateEnergyNum(double potentialEnergy){
    energyNum->setText("Potential Energy: " + chai3d::cStr((potentialEnergy / 2), 5));

}

void LabelManager::updateNumAnchored(int anchored, int total, int width){
    numAnchored->setText(std::to_string(anchored) + " anchored / " +
                          std::to_string(total) + " total");
    auto num_anchored_width = (width - numAnchored->getWidth()) - 5;
    numAnchored->setLocalPos(num_anchored_width, 0);
}

void LabelManager::updateFrozen(int width, bool freezeAtoms){
    // update frozen state label
    std::string trueFalse = freezeAtoms ? "true" : "false";
    isFrozen->setText("Freeze simulation: " + trueFalse);
    auto isFrozenWidth = (width - isFrozen->getWidth()) - 5;
    isFrozen->setLocalPos(isFrozenWidth, 15);

}
