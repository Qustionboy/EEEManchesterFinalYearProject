#ifndef L_SYSTEM_H
#define L_SYSTEM_H

#include "grammar.h"           
#include "grapes.h"            
#include "transformation.h"  
#include "random_generator.h"
#include <vector>
#include <cmath>               
#include <iostream>
#include <fstream>
#include <string>




class State { // current state
public:
    Node pos;
    Node dir; 

};

class TrunkPosition {
public:
    Node pos1;
    Node pos2;
    double radius = 0.0;
};

class BerryPosition {
public:
    Node pos1;
    Node pos2;
    Node centres;
    Node direction;
    double radius;

    void getCentres() {
        centres.x = (pos1.x + pos2.x) / 2;
        centres.y = (pos1.y + pos2.y) / 2;
        centres.z = (pos1.z + pos2.z) / 2;
    }
};

class LSystem {
public:
    int stackpointer;
    double dx, dy, dz;
    double length;                      // rachis length of a unit layer
    double init_length;
    double lengthFactor;                // stem length shrinks ratio
    double radius;                      // stem radius
    double init_radius;
    double radiusFactor;                // stem radius shrinks ratio
    State curState;                     // current position and travelling direction
    double berriesRadius;               
    int BezierN = 10;
    double cylinder_step = 30;
    int lats = 50;
    int longs = 50;
    bool Draw_Berry = true;


    Grammar grammar;                    // Grammar object to generate and store rule
    RandomGenerator rand;               // Initiate Random Genterator
    Grapes g;

    std::vector<TrunkPosition> trunks;  // Stem vertices
    std::vector<BerryPosition> berries; // Berry vertices

    LSystem();
    void generateFractal();             // Generate vertices

    void NewPOS(double r, double New_r, int idx);
    void Bezier(int BezierN, std::vector<TrunkPosition>& trunks, Node& X, Node& Y, Node& Z, double radius); // Generate Bezier curves for the stem


};

#endif // L_SYSTEM_H
