#ifndef GRAPES_H
#define GRAPES_H

#include <vector>
#include "random_generator.h" // Ensure this includes the definition for RandomGenerator
#include "transformation.h"



class Berry {
public:
    RandomGenerator randomG;
    double radius; // default berry radius

    Berry();
    std::vector<float> createGaussianFactor(int numPoints, double a, double b, double c, double d);
    std::vector<Node> createTaperedSphere(float radius, int slices, int stacks);
};


class Trunk //Stem
{
public:
    double length = 5.5;
    double init_length = length;        //rachis length of a unit layer
    double length_shrink = 0.9;        //stem length shrinks ratio
    double radius = 0.3;                //stem radius
    double init_radius = radius;
    double radius_shrink = 0.5;         //stem radius shrinks ratio
};


class Grapes
{
public:
    Grapes();
    Trunk trunk;
    Berry berry;
};

#endif // GRAPES_H

