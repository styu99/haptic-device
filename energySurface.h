#ifndef ENERGY_SURFACE_H
#define ENERGY_SURFACE_H

#include <string>

enum Potential { LENNARD_JONES, MORSE, MACHINE_LEARNING };

const double EPSILON = 1.0;
const double FORCE_DAMPING = 0.75;
const double K_VALUE = 1.0;


class EnergySurface{
protected:
    std::string name;

public:
    EnergySurface(std::string pName);
    virtual enum Potential getType() = 0;
    virtual double getEnergy(double distance) = 0;
    virtual double getForce(double distance) = 0;
    std::string getName();
};


class LJPotential : public EnergySurface{
private:
    const double SIGMA = 1.0;

public:   
    LJPotential(std::string pName) : EnergySurface(pName){}
    enum Potential getType();
    double getEnergy(double distance);
    double getForce(double distance);
};

class MorsePotential : public EnergySurface{
private:
    const double RHO0 = 6.0;
    const double R0 = 1.0;

public:
    MorsePotential(std::string pName) : EnergySurface(pName){}
    enum Potential getType();
    double getEnergy(double distance);
    double getForce(double distance);
};

#endif
