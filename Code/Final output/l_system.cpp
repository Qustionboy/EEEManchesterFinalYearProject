#define _USE_MATH_DEFINES
#include "l_system.h"
#include <cmath>
#include"random_generator.h"

using namespace std;

Transformation trans;


// Constructor
LSystem::LSystem() {
    stackpointer = 0;
    length = g.trunk.length;
    init_length = g.trunk.init_length;
    lengthFactor = g.trunk.length_shrink;
    radius = rand.getRandomDouble(0.3,0.5);
    init_radius = g.trunk.init_radius;
    radiusFactor = g.trunk.radius_shrink;
    
    curState = {};
    grammar.iteration();
    
}



void LSystem::Bezier(int BezierN, std::vector<TrunkPosition>& trunks, Node& X, Node& Y, Node& Z, double radius) {
    float f1, f2, f3;
    float deltaT = 1.0 / BezierN;
    float T;
    Node pre = X;
    Node cur;
    TrunkPosition tmp;
    for (int i = 0; i <= BezierN; i++) {
        tmp.pos1 = pre;
        T = i * deltaT;
        f1 = (1 - T) * (1 - T);
        f2 = 2 * T * (1 - T);
        f3 = T * T;
        float x = f1 * X.x + f2 * Y.x + f3 * Z.x;
        float y = f1 * X.y + f2 * Y.y + f3 * Z.y;
        float z = f1 * X.z + f2 * Y.z + f3 * Z.z;
        cur.x = x;
        cur.y = y;
        cur.z = z;
        tmp.pos2 = cur;
        tmp.radius = radius;
        pre = cur;
        trunks.push_back(tmp);
    }
}

void LSystem::generateFractal() {
    trunks.clear();
    curState.pos = Node(0, 0, 0);
    curState.dir = Node(0, 1, 0);
    State stack[3000];                      // state stacks (for brunching)
    for (int i = 0; i < 3000; i++){
        stack[i].pos.x = 0.0;
        stack[i].pos.y = 0.0;
        stack[i].pos.z = 0.0;
        stack[i].dir.x = 0.0;
        stack[i].dir.y = 0.0;
        stack[i].dir.z = 0.0;
    }
    size_t i = 0;
    unsigned int len = grammar.getRule().length();
    double num_A = 0;
    double num_B = 0;
    std::cout << "Grammer length " << len << std::endl;
    while (i < len) {
        TrunkPosition tmp_trunk;
        BerryPosition tmp_berry;
        Node Z;

        char key = grammar.getRule()[i];
        berriesRadius = rand.getRandomDouble(1,1.5);

        switch (key)
        {


        case 'F':   //Rachis
            tmp_trunk.pos1 = curState.pos;
            curState.pos.x += 0.8 * length * curState.dir.x;
            curState.pos.y += 0.8 * length * curState.dir.y;
            curState.pos.z += 0.8 * length * curState.dir.z;
            tmp_trunk.pos2 = curState.pos;

            Z.x = 0.5 * (tmp_trunk.pos1.x + tmp_trunk.pos2.x) + length / init_length * (rand.getRandomInt(0, 32767) % 10 + 2) / 10;
            Z.y = 0.5 * (tmp_trunk.pos1.y + tmp_trunk.pos2.y) - length / init_length * (rand.getRandomInt(0, 32767) % 10 + 5) / 10;
            Z.z = 0.5 * (tmp_trunk.pos1.z + tmp_trunk.pos2.z) + length / init_length * (rand.getRandomInt(0, 32767) % 10 + 4) / 10;

            Z.w = curState.pos.w;
            Bezier(BezierN, trunks, tmp_trunk.pos1, Z, tmp_trunk.pos2, radius);
            //num_F++;
            break;

        case 'W':   //pedicel
            tmp_trunk.pos1 = curState.pos;
            curState.pos.x += 0.2 * length * curState.dir.x;
            curState.pos.y += 0.2 * length * curState.dir.y;
            curState.pos.z += 0.2 * length * curState.dir.z;
            tmp_trunk.pos2 = curState.pos;

            Z.x = 0.5 * (tmp_trunk.pos1.x + tmp_trunk.pos2.x) + 0.4 * length / init_length * (rand.getRandomInt(0, 32767) % 10 + 2) / 10;
            Z.y = 0.5 * (tmp_trunk.pos1.y + tmp_trunk.pos2.y) - 0.4 * length / init_length * (rand.getRandomInt(0, 32767) % 10 + 5) / 10;
            Z.z = 0.5 * (tmp_trunk.pos1.z + tmp_trunk.pos2.z) + 0.4 * length / init_length * (rand.getRandomInt(0, 32767) % 10 + 4) / 10;
            Z.w = curState.pos.w;
            Bezier(BezierN, trunks, tmp_trunk.pos1, Z, tmp_trunk.pos2, radius);
            //num_W++;
            break;

        case 'U':  //peduncle
            tmp_trunk.pos1 = curState.pos;
            curState.pos.x += 0.35 * length * curState.dir.x;
            curState.pos.y += 0.35 * length * curState.dir.y;
            curState.pos.z += 0.35 * length * curState.dir.z;
            tmp_trunk.pos2 = curState.pos;

            Z.x = 0.5 * (tmp_trunk.pos1.x + tmp_trunk.pos2.x) + 0.5 * length / init_length * (rand.getRandomInt(0, 32767) % 10 + 2) / 10;
            Z.y = 0.5 * (tmp_trunk.pos1.y + tmp_trunk.pos2.y) - 0.5 * length / init_length * (rand.getRandomInt(0, 32767) % 10 + 5) / 10;
            Z.z = 0.5 * (tmp_trunk.pos1.z + tmp_trunk.pos2.z) + 0.5 * length / init_length * (rand.getRandomInt(0, 32767) % 10 + 4) / 10;
            Z.w = curState.pos.w;
            Bezier(BezierN, trunks, tmp_trunk.pos1, Z, tmp_trunk.pos2, radius);
            //num_U++;
            break;


        case 'D':   //Bud
            tmp_trunk.pos1 = curState.pos;
            curState.pos.x += berriesRadius * curState.dir.x;
            curState.pos.y += berriesRadius * curState.dir.y;
            curState.pos.z += berriesRadius * curState.dir.z;
            tmp_trunk.pos2 = curState.pos;
            Z.x = 0.5 * (tmp_trunk.pos1.x + tmp_trunk.pos2.x) + (rand.getRandomInt(0, 32767) % 20 - 5) / 10;
            Z.y = 0.5 * (tmp_trunk.pos1.y + tmp_trunk.pos2.y) - (rand.getRandomInt(0, 32767) % 20 - 2) / 10;
            Z.z = 0.5 * (tmp_trunk.pos1.z + tmp_trunk.pos2.z) + (rand.getRandomInt(0, 32767) % 20 - 4) / 10;
            Z.w = curState.pos.w;
            Bezier(BezierN, trunks, tmp_trunk.pos1, Z, tmp_trunk.pos2, radius);
            //num_D++;
            break;

        case 'X':   //Berry
            tmp_berry.pos1 = curState.pos;
            curState.pos.x += 2 * berriesRadius * curState.dir.x;
            curState.pos.y += 2 * berriesRadius * curState.dir.y;
            curState.pos.z += 2 * berriesRadius * curState.dir.z;
            tmp_berry.pos2 = curState.pos;
            tmp_berry.radius = berriesRadius;
            tmp_berry.direction = curState.dir;
            tmp_berry.getCentres();
            berries.push_back(tmp_berry);
            break;

            

        case 'A':    //Shrinks length
            length = length * lengthFactor;
            radius = radius * std::pow(radiusFactor, (1 / (1 + 0.5 * num_A)));
            num_A++;
            break;
        case 'B':    //Scale length
            length = length / lengthFactor;
            radius = radius / std::pow(radiusFactor, (1 / (1 + 0.5 * num_B)));
            num_B++;
            break;

      

        case '[':    //Branch out
            stack[stackpointer] = curState;
            stackpointer++;;
            break;
        case ']':    //return to the bifurcation point
            curState = stack[stackpointer - 1];
            stackpointer--;
            //num_F = tem_F;
            break;
        case '^':    //rotate around x-axis
            dx = rand.getRandomDouble(55, 65);
            trans.set(curState.dir);
            curState.dir = trans.Rotate('X', dx + (rand.getRandomInt(0, 32767) % 20 - 10));
            break;
        case '&':
            dx = rand.getRandomDouble(55, 65);
            trans.set(curState.dir);
            curState.dir = trans.Rotate('X', -dx + (rand.getRandomInt(0, 32767) % 20 - 10));
            break;
        case '$': //rotate around y-axis
            dy = rand.getRandomDouble(5, 15);
            trans.set(curState.dir);
            curState.dir = trans.Rotate('Y', dy);
            break;
        case '%':
            dy = rand.getRandomDouble(5, 15);
            trans.set(curState.dir);
            curState.dir = trans.Rotate('Y', -dy);
            break;
        case '*'://rotate around z-axis
            dz = rand.getRandomDouble(55, 65);
            trans.set(curState.dir);
            curState.dir = trans.Rotate('Z', dz + (rand.getRandomInt(0, 32767) % 20 - 10));
            break;
        case '/':
            dz = rand.getRandomDouble(55, 65);
            trans.set(curState.dir);
            curState.dir = trans.Rotate('Z', -dz + (rand.getRandomInt(0, 32767) % 20 - 10));
            break;
        default:
            break;
        }
        i++;

    }

    num_A = 0;
    num_B = 0;
}

void LSystem::NewPOS(double r, double New_r, int idx) {
    double k = New_r / (r - New_r);
    double m = r / (r - New_r);
    berries[idx].radius = New_r;
    berries[idx].pos2.x = ((berries[idx].pos1.x + k * berries[idx].pos2.x) / m);
    berries[idx].pos2.y = ((berries[idx].pos1.y + k * berries[idx].pos2.y) / m);
    berries[idx].pos2.z = ((berries[idx].pos1.z + k * berries[idx].pos2.z) / m);
    berries[idx].getCentres();
}