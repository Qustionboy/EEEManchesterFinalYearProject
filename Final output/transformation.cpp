#define _USE_MATH_DEFINES
#include "transformation.h"
#include <cmath>

using namespace std;

void Transformation::set(const Node& point){
    POld = point;
}
void Transformation::Identity(){
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            T[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

Node Transformation::Translate(double tx, double ty, double tz){
    Identity();
    T[3][0] = tx;
    T[3][1] = ty;
    T[3][2] = tz;
    MultiMatrix();
    return POld;
}

Node Transformation::Scale(double sx, double sy, double sz) {
    Identity();
    T[0][0] = sx;
    T[1][1] = sy;
    T[2][2] = sz;
    MultiMatrix();
    return POld;
}

Node Transformation::Rotate(char shaft, double beta){
    Identity();
    double rad = (beta * M_PI) / 180;
    switch (shaft)
    {
    case 'X'://rotate around X
        T[0][0] = 1;
        T[1][1] = cos(rad); T[1][2] = sin(rad);
        T[2][1] = -sin(rad); T[2][2] = cos(rad);
        break;
    case 'Y': //rotate around Y
        T[0][0] = cos(rad); T[0][2] = -sin(rad);
        T[1][1] = 1;
        T[2][0] = sin(rad); T[2][2] = cos(rad);
        break;
    case 'Z':  // around Z
        T[0][0] = cos(rad); T[0][1] = sin(rad);
        T[1][0] = -sin(rad); T[1][1] = cos(rad);
        T[2][2] = 1;
    }
    MultiMatrix();
    return POld;
}

void Transformation::MultiMatrix(){
    Node PNew = POld;

    POld.x = PNew.x * T[0][0] + PNew.y * T[1][0] + PNew.z * T[2][0] + PNew.w * T[3][0];
    POld.y = PNew.x * T[0][1] + PNew.y * T[1][1] + PNew.z * T[2][1] + PNew.w * T[3][1];
    POld.z = PNew.x * T[0][2] + PNew.y * T[1][2] + PNew.z * T[2][2] + PNew.w * T[3][2];
    POld.w = PNew.x * T[0][3] + PNew.y * T[1][3] + PNew.z * T[2][3] + PNew.w * T[3][3];
}