#ifndef RANDOMGENERATOR_H
#define RANDOMGENERATOR_H

#include <random>

class RandomGenerator {
public:
    RandomGenerator();                                  // Constructor
    int getRandomInt(int min, int max);                 // Get a random integer between min and max
    double getRandomDouble(double min, double max);     // Get a random double between min and max
    double getRandomNormal(double mean, double stddev); // Get a random double by noramal distribution

private:
    std::mt19937 eng;                               // Mersenne Twister engine
};

#endif // RANDOMGENERATOR_H


