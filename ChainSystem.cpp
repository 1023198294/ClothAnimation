
#include "ChainSystem.h"
#include <iostream>

using namespace std;

SimpleChain::SimpleChain():PendulumSystem(4) {

    //wireframe_ON = true;

    particles.push_back(Vector4f_(-1.0f, 0.0f, 0.0f, 1.0f));
    particles.push_back(Vector4f_(-2.0f, 0.0f, 0.5f, 0.0f));
    particles.push_back(Vector4f_(-0.0f, -1.0f, 0.0f, 0.0f));
    particles.push_back(Vector4f_(-1.0f, 1.0f, -1.5f, 0.0f));

    //float restLength = 1.0f;
    //float springConstant = 2.0f;

    springs.push_back(Vector4f_(0.0f, 1.0f, 0.25f, 5.0f));
    springs.push_back(Vector4f_(1.0f, 2.0f, 0.25f, 5.0f));
    springs.push_back(Vector4f_(2.0f, 3.0f, 0.25f, 5.0f));

    initPendulum(particles, springs, faces);

}
