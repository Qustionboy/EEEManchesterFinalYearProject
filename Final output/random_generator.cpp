#include "random_generator.h"

RandomGenerator::RandomGenerator() {
    // Initialize our Mersenne Twister with a random seed based on the clock (system time)
    std::random_device rd;
    eng = std::mt19937(rd());
}

int RandomGenerator::getRandomInt(int min, int max) {
    std::uniform_int_distribution<int> distr(min, max);
    return distr(eng);
}

double RandomGenerator::getRandomDouble(double min, double max) {
    std::uniform_real_distribution<double> distr(min, max);
    return distr(eng);
}

double RandomGenerator::getRandomNormal(double mean, double stddev) {
    std::normal_distribution<double> distr(mean, stddev);
    return distr(eng);
}