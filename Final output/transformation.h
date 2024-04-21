#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <array>

class Node // Homogeneous coordinate point
{
public:
    Node() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
    Node(double xx, double yy, double zz) : x(xx), y(yy), z(zz), w(1.0f) {}

public:
    double x, y, z, w;
};

class Transformation {
public:
    Transformation() { Identity(); }

    void set(const Node& point);

    void Identity();

    Node Translate(double dx, double dy, double dz);

    Node Scale(double tx, double ty, double tz);

    Node Rotate(char axis, double angle);

    void MultiMatrix();

public:
    Node POld;
    std::array<std::array<float, 4>, 4> T; // Transformation matrix
};

#endif // TRANSFORMATION_H

