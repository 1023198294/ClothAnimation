
#include "simpleSystem.h"

using namespace std;

SimpleSystem::SimpleSystem() {
    // add a vertex.
    m_vVecState.push_back(Vector3f_(0, 0.5, 0));
    m_numParticles = 1;
}

// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f_> SimpleSystem::evalF(vector<Vector3f_> state) {
    vector<Vector3f_> f;
    f.push_back(Vector3f_(-state[0].y(), state[0].x(), 0));
    return f;
}

// render the system (ie draw the particles)
void SimpleSystem::draw() {
    Vector3f_ pos = getState()[0];//YOUR PARTICLE POSITION
    glPushMatrix();
    glTranslatef(pos[0], pos[1], pos[2]);
    glutSolidSphere(0.075f, 10.0f, 10.0f);
    glPopMatrix();
}
