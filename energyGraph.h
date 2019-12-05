#ifndef ENERGY_GRAPH_H
#define ENERGY_GRAPH_H

#include "chai3d.h"

using namespace chai3d;

class EnergyGraph {
private:
    cCamera* camera;

    double global_minimum;

    cLabel *scope_upper;
    cLabel *scope_lower;

    cScope *scope;

    double upper_bound;
    double lower_bound;

    bool global_min_known;

    double getGlobalMinima(int cluster_size);

public:
    EnergyGraph(cCamera* cam, bool lj, int cluster_size);
    void updateGraph(cPrecisionClock *clock, double potentialEnergy);
};

#endif
