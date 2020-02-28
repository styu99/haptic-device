#include "energySurface.h"
#include "math.h"
#include <assert.h>

EnergySurface::EnergySurface(std::string pName){
    name = pName;
}

std::string EnergySurface::getName(){
    return name;
}

enum Potential LJPotential::getType(){
    return LENNARD_JONES;
}

double LJPotential::getEnergy(double distance){
    return 4 * EPSILON * (pow(SIGMA / distance, 12) - pow(SIGMA / distance, 6));   
}

double LJPotential::getForce(double distance){
    return -4 * FORCE_DAMPING * EPSILON *
         ((-12 * pow(SIGMA / distance, 13)) - (-6 * pow(SIGMA / distance, 7)));
}

enum Potential MorsePotential::getType(){
    return MORSE;
}

//based off off ASE Morse calculator parameters and format; see ASE documentation
double MorsePotential::getEnergy(double distance){
    double expf = exp(RHO0 * (1.0 - distance / R0));
    return EPSILON * expf * (expf - 2);
}

//negative derivative of energy with respect to distance
double MorsePotential::getForce(double distance){
    double temp = -2 * RHO0 * EPSILON * exp(RHO0 - (2 * RHO0 * distance) / R0) * (exp((RHO0 * distance)/R0) - exp(RHO0));
    return temp / R0 * FORCE_DAMPING;
}