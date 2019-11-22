#include "chai3d.h"
#include "atomManager.h"

using namespace std;
using namespace chai3d;

AtomManager::AtomManager(){
    numAtoms = 0;
    currentAtom = NULL;
    numAnchored = 0;
}

void AtomManager::setCurrentAtom(Atom* newCur){
    currentAtom = newCur;
}

Atom* AtomManager::getCurrentAtom(){
    return currentAtom;
}

void generateAtoms(int totalAtoms){
    /*
    numAtoms = totalAtoms;

    // Randomly generate the specified number of atoms
    for (int i = 0; i < numAtoms; i++) {
      // create a sphere and define its radius
      Atom *new_atom = new Atom(SPHERE_RADIUS, SPHERE_MASS);

      // store pointer to sphere primitive
      atomsList.push_back(new_atom);

      // add sphere primitive to world
      world->addChild(new_atom);

      // add line to world
      world->addChild(new_atom->getVelVector());

      // Atom spawning
      bool inside_atom = true;
      if (i != 0) {
        bool collision_detected;
        auto iter{0};
        while (inside_atom) {
          // Place atom at a random position
          if (iter > 1000) {
            // If there are too many failed attempts at placing the atom
            // increase the radius in which it can spawn
            new_atom->setInitialPosition(.115);
          } else {
            new_atom->setInitialPosition();
          }

          // Check that it doesn't collide with any others
          collision_detected = false;
          for (auto i{0}; i < spheres.size(); i++) {
            auto dist_between =
                cDistance(new_atom->getLocalPos(), spheres[i]->getLocalPos());
            dist_between = dist_between / .02;
            if (dist_between == 0) {
              continue;
            } else if (dist_between < 1.5) {
              // The number dist between is being compared to
              // is the threshold for collision
              collision_detected = true;
              iter++;
              break;
            }
          }

          if (!collision_detected) {
            inside_atom = false;
          }
        }
      }

      // set graphic properties of sphere
      new_atom->setTexture(texture);
      new_atom->m_texture->setSphericalMappingEnabled(true);
      new_atom->setUseTexture(true);

      // set the first sphere to the current
      if (i == 0)  {
        new_atom->setCurrent(true);
      }
  }*/
}
