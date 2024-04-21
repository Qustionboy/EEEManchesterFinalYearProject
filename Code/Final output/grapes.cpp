
#define _USE_MATH_DEFINES
#include <cmath>
#include "grapes.h"
#include "random_generator.h"



Berry::Berry() {
	radius = randomG.getRandomDouble(1, 1.5);
}

std::vector<float> Berry::createGaussianFactor(int numPoints, double a, double b, double c, double d) {
    std::vector<float> GaussianFactor(numPoints);
    float step = 2.0 / (numPoints - 1);

    for (int i = 0; i < numPoints; ++i) {
        float z = -2 + i * step;                   
        GaussianFactor[i] = a * exp(-b * pow(z - c, 2)) + d;
    }

    return GaussianFactor;
}

std::vector<Node> Berry::createTaperedSphere(float radius, int slices, int stacks) {
    std::vector<Node> temp_berry; 
    double amp = randomG.getRandomDouble(0.2, 0.3);
    double b = randomG.getRandomDouble(0.4, 0.6);
    double height = randomG.getRandomDouble(1.1, 1.4);
    double randomFactor = randomG.getRandomDouble(-0.5, 0.2);
    double randomPosition = randomG.getRandomDouble(-2.0, 0.0);
    double randomFactor1 = randomG.getRandomDouble(-0.5, 0.2);
    double randomPosition1 = randomG.getRandomDouble(-2.0, 0.0);
    auto zFactor = createGaussianFactor(stacks + 1, amp, b, 1, +0.1);
    auto randomness1 = createGaussianFactor(slices + 1, randomFactor, amp, randomPosition, 0.1);
    auto randomness2 = createGaussianFactor(slices + 1, randomFactor1, amp, randomPosition1, 0.1);

    for (int i = 0; i <= stacks; ++i) {

        float stackAngle = M_PI * i / stacks;
        float sinStack = sin(stackAngle);
        float cosStack = cos(stackAngle);
        double radius_p = radius + zFactor[i];
        for (int j = 0; j <= slices; ++j) {

            float sliceAngle = 2 * M_PI * j / slices;
            float sinSlice = sin(sliceAngle);
            float cosSlice = cos(sliceAngle);
            

            Node v((radius_p + randomness1[j]) * sinStack * cosSlice,
                height * radius * cosStack,
                (radius_p + randomness2[j]) * sinStack * sinSlice);
            temp_berry.push_back(v);
        }
    }

    return temp_berry; 
}


Grapes::Grapes(){

}


