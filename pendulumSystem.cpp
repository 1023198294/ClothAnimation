
#include "pendulumSystem.h"
#include "particleSystem.h"
#include <iostream>
#include <cmath>

int degree = 0;        //Global variable for sinusoidalMovement
float xTrans = 0.0f;  //Global variable to make ball interacive
float yTrans = 0.0f;

using namespace std;

PendulumSystem::PendulumSystem(int numParticles) : ParticleSystem(numParticles) {
    m_numParticles = numParticles;

    //wireframe_ON = false;
    sinusoidMove_ON = false;
    //faces_ON = true;
    particles_ON = true;
    structSprings_ON = false;
    //Force Constants//
    //gravC = -9.8

    mg = -2.0f;
    vDrag = 1.3f;

}

void PendulumSystem::initPendulum(vector<Vector4f_> myParticles,
                                  vector<Vector4f_> mySprings,
                                  vector<Vector3f_> myFaces) {
    particles = myParticles;
    springs = mySprings;
    faces = myFaces;

    // fill in code for initializing the state based on the of number particles
    for (int i = 0; i < m_numParticles; i++) {

        // for this system, we care about the position and the velocity
        m_vVecState.push_back(particles[i].xyz());
        m_vVecState.push_back(Vector3f_(0.0, 0.0, 0.0));
    }

}


// TODO: implement evalF
// for a given state, evaluate f(X,t)
vector<Vector3f_> PendulumSystem::evalF(vector<Vector3f_> state) {
    vector<Vector3f_> f;

    for (unsigned int s = 0; s < state.size(); s++) {

        Vector3f_ derivF_v = state[2 * s + 1];    // v = state (odd number)
        f.push_back(derivF_v);

        Vector3f_ f_Gravity = Vector3f_(0, mg, 0);    //Gravity
        Vector3f_ f_Drag = -1 * vDrag * state[2 * s + 1];    //Drag
        Vector3f_ f_Net = f_Gravity + f_Drag;    //Net Force
        f.push_back(f_Net);
    }

    //Implement Springs
    for (unsigned int i = 0; i < springs.size(); i++) {
        float restL = springs[i][2];
        float springC = springs[i][3];

        int p0_Index = (int) springs[i][0];
        int p1_Index = (int) springs[i][1];

        Vector3f_ p0 = state[2 * p0_Index];
        Vector3f_ p1 = state[2 * p1_Index];

        Vector3f_ p0_p1 = p0 - p1;

        Vector3f_ f_Spring_p0 = -1 * springC * (p0_p1.abs() - restL) * (p0_p1 / (p0_p1.abs()));
        Vector3f_ f_Spring_p1 = -1 * f_Spring_p0;

        f[2 * p0_Index + 1] += f_Spring_p0;
        f[2 * p1_Index + 1] += f_Spring_p1;
    }

    //Wind forces

    if (wind) {
        for (int i = 0; i < m_numParticles; i++) {
            float fraction = float(rand()) / RAND_MAX;
            f[2 * i + 1] += fraction * Vector3f_(0.0f, 0.0f, 2.0f);
        }
    }


    //Make necessary particles fixed
    for (unsigned int k = 0; k < particles.size(); k++) {
        if (particles[k].w() == 1.0f) {
            f[2 * k + 1] = Vector3f_(0.0f, 0.0f, 0.0f);
        }
    }

    //Sinusoidal Movement
    if (move) {
        degree++;
        float moveVelocity = 1.5 * cos(convertRad(degree) * .5);
        for (unsigned int m = 0; m < particles.size(); m++) {
            if (particles[m].w() == 1.0f) {
                f[2 * m] = Vector3f_(0.0f, 0.0f, moveVelocity);
            }
        }
    }

    return f;
}


// render the system (ie draw the particles)
void PendulumSystem::draw() {
    //Wireframe
    if (this->display_spring) {
        if (particles_ON) {
            for (int i = 0; i < m_numParticles; i++) {
                Vector3f_ pos = getState()[2 * i];//  position of particle i.
                glPushMatrix();
                glTranslatef(pos[0], pos[1], pos[2]);
                glutSolidSphere(0.075f, 10.0f, 10.0f);
                glPopMatrix();
            }
        }

        for (unsigned int a = 0; a < springs.size(); a++) {
            if (!structSprings_ON) {
                Vector3f_ p0 = getState()[2 * (int) springs[a][0]];
                Vector3f_ p1 = getState()[2 * (int) springs[a][1]];
                glLineWidth(2.0f);
                glBegin(GL_LINES);
                glVertex3f(p0.x(), p0.y(), p0.z());
                glVertex3f(p1.x(), p1.y(), p1.z());
                glEnd();
            } else {
                float restL = 0.20f;
                if (springs[a][2] == restL) {
                    Vector3f_ p0 = getState()[2 * (int) springs[a][0]];
                    Vector3f_ p1 = getState()[2 * (int) springs[a][1]];
                    glLineWidth(2.0f);
                    glBegin(GL_LINES);
                    glVertex3f(p0.x(), p0.y(), p0.z());
                    glVertex3f(p1.x(), p1.y(), p1.z());
                    glEnd();
                }
            }
        }
    }

    //Faces
    if (!this->display_spring) {
        drawClothes();
    }
    //Ball for Cloth
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    glColor3f(1.0f, 1.0f, 0.0f);
    Vector3f_ locBall = Vector3f_(1.0f + xTrans, -3.5f + yTrans, 0.0f);
    float radBall = 1.0f;
    float epsilon = 0.125f;
    glPushMatrix();
    glTranslatef(locBall.x(), locBall.y(), locBall.z());
    glutSolidSphere(radBall, 10.0f, 10.0f);
    glPopMatrix();
    glDisable(GL_COLOR_MATERIAL);
    for (int n = 0; n < m_numParticles; n++) {
        if ((m_vVecState[2 * n] - locBall).abs() <= (radBall + epsilon)) {
            m_vVecState[2 * n] = (locBall +
                                  (radBall + epsilon) * (m_vVecState[2 * n] - locBall).normalized());
        }
    }


}

void PendulumSystem::drawClothes() {
    for (unsigned int f = 0; f < faces.size(); f++) {
        Vector3f_ face = faces[f];
        Vector3f_ v1 = getState()[2 * (int) face[0]];
        Vector3f_ v2 = getState()[2 * (int) face[1]];
        Vector3f_ v3 = getState()[2 * (int) face[2]];
        Vector3f_ n = Vector3f_::cross(v2 - v1, v3 - v1).normalized();
        glBegin(GL_TRIANGLES);
        glNormal3f(n[0], n[1], n[2]);
        glVertex3f(v1[0], v1[1], v1[2]);
        glVertex3f(v2[0], v2[1], v2[2]);
        glVertex3f(v3[0], v3[1], v3[2]);
        glEnd();
    }
}
