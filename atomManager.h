#ifndef ATOM_MANAGER_H
#define ATOM__MANAGER_H

#include "atom.h"

class AtomManager{

private:
    int numAnchored;
    int numAtoms;
    Atom* currentAtom;
    vector<Atom *> atomsList;

public:
    AtomManager();
    void setCurrentAtom(Atom* newCur);
    Atom* getCurrentAtom();
    void generateAtoms(int totalAtoms);
};


#endif
