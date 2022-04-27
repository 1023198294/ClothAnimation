#include "ClothSystem.h"
#include <cmath>
#include <iostream>

using namespace std;

ClothSystem::ClothSystem(int C_SIZE) : PendulumSystem(C_SIZE * C_SIZE) {
    this->C_SIZE = C_SIZE;
    particles_ON = false;
    structSprings_ON = true;


    float restL = 0.20f;
    float distance = 0.20f;
    float springC = 200.0f;
    for (int row = 0; row < C_SIZE; row++) {
        vector<vector<int> > temp;
        for (int col = 0; col < C_SIZE; col++) {
            if ((row == 0 && col == 0) || (row == 0 && col == C_SIZE - 1)) {    //endpoints are fixed
                particles.push_back(Vector4f(col * distance, 0.0f, row * distance, 1.0f));
            } else {
                particles.push_back(Vector4f(col * distance, 0.0f, row * distance, 0.0f));
            }
            //Structural Springs
            //======================================================
            //Link(row,col) to Link(row+1,col)
            if (row < C_SIZE - 1) {
                float p0 = (float) ((row) * C_SIZE + (col));
                float p1 = (float) ((row + 1) * C_SIZE + (col));
                springs.push_back(Vector4f(p0, p1, restL, springC));
            }
            //Link(row,col) to Link(row,col+1)
            if (col < C_SIZE - 1) {
                float p0 = (float) ((row) * C_SIZE + (col));
                float p1 = (float) ((row) * C_SIZE + (col + 1));
                springs.push_back(Vector4f(p0, p1, restL, springC));
            }

            //Shear Springs
            //======================================================
            //Link(row,col) to Link(row+1,col+1)
            if (row < C_SIZE - 1 && col < C_SIZE - 1) {
                float p0 = (float) ((row) * C_SIZE + (col));
                float p1 = (float) ((row + 1) * C_SIZE + (col + 1));
                springs.push_back(Vector4f(p0, p1, restL * sqrt(2), springC));
            }
            //Link(row,col) to Link(row+1,col-1)
            if (row < C_SIZE - 1 && col > 0) {
                float p0 = (float) ((row) * C_SIZE + (col));
                float p1 = (float) ((row + 1) * C_SIZE + (col - 1));
                springs.push_back(Vector4f(p0, p1, restL * sqrt(2), springC));
            }

            //Flex Springs
            //======================================================
            //Link(row,col) to Link(row+2,col)
            if (row < C_SIZE - 2) {
                float p0 = (float) ((row) * C_SIZE + (col));
                float p1 = (float) ((row + 2) * C_SIZE + (col));
                springs.push_back(Vector4f(p0, p1, restL * 2, springC));
            }
            //Link(row,col) to Link(row,col+2)
            if (col < C_SIZE - 2) {
                float p0 = (float) ((row) * C_SIZE + (col));
                float p1 = (float) ((row) * C_SIZE + (col + 2));
                springs.push_back(Vector4f(p0, p1, restL * 2, springC));
            }

            //Faces

            //======================================================
            if (row < C_SIZE - 1 && col < C_SIZE - 1) {
                //Front - normals pointing forward
                int face1_idx = faces.size();
                Vector3f face1 = Vector3f((float) ((row + 1) * C_SIZE + (col + 1)),
                                          (float) ((row) * C_SIZE + (col + 1)),
                                          (float) ((row + 1) * C_SIZE + (col)));
                faces.push_back(face1);
                int face2_idx = face1_idx + 1;
                Vector3f face2 = Vector3f((float) ((row) * C_SIZE + (col + 1)),
                                          (float) ((row) * C_SIZE + (col)),
                                          (float) ((row + 1) * C_SIZE + (col)));
                faces.push_back(face2);
            }
        }
    }
    initPendulum(particles, springs, faces);
}

bool edge(int index, int C_SIZE) {
    int row = index / C_SIZE;
    int col = index % C_SIZE;
    if (row <= 0 || col <= 0 || row >= C_SIZE - 1 || col >= C_SIZE - 1) {
        return true;
    }
    return false;
}

void ClothSystem::draw() {
    //Wireframe
    if (this->display_spring) {
        if (particles_ON) {
            for (int i = 0; i < m_numParticles; i++) {
                Vector3f pos = getState()[2 * i];//  position of particle i.
                glPushMatrix();
                glTranslatef(pos[0], pos[1], pos[2]);
                glutSolidSphere(0.075f, 10.0f, 10.0f);
                glPopMatrix();
            }
        }

        for (unsigned int a = 0; a < springs.size(); a++) {
            if (!structSprings_ON) {
                Vector3f p0 = getState()[2 * (int) springs[a][0]];
                Vector3f p1 = getState()[2 * (int) springs[a][1]];
                glLineWidth(2.0f);
                glBegin(GL_LINES);
                glVertex3f(p0.x(), p0.y(), p0.z());
                glVertex3f(p1.x(), p1.y(), p1.z());
                glEnd();
            } else {
                float restL = 0.20f;
                if (springs[a][2] == restL) {
                    Vector3f p0 = getState()[2 * (int) springs[a][0]];
                    Vector3f p1 = getState()[2 * (int) springs[a][1]];
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
    Vector3f locBall = Vector3f(1.0f + xTrans, -3.5f + yTrans, 0.0f);
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

void ClothSystem::drawClothes() {
    vector<Vector3f> state = getState();
    vector<Vector3f> norms(C_SIZE * C_SIZE);
    for (int i = 0; i < C_SIZE - 1; i++) {
        for (int j = 0; j < C_SIZE - 1; j++) {
            int index = i * C_SIZE + j;
            Vector3f v1 = state[2 * index];
            Vector3f v2 = state[2 * (index + 1)];
            Vector3f v3 = state[2 * (index + 1 + C_SIZE)];
            Vector3f v4 = state[2 * (index + C_SIZE)];

            Vector3f norm1 = Vector3f::cross(v3 - v1, v2 - v3).normalized();
            Vector3f norm2 = Vector3f::cross(v4 - v1, v3 - v4).normalized();

            norms[i * C_SIZE + j] = norms[i * C_SIZE + j] + norm1 + norm2;
            norms[i * C_SIZE + j + 1] = norms[i * C_SIZE + j] + norm1;
            norms[(i + 1) * C_SIZE + j] = norms[i * C_SIZE + j] + norm2;
            norms[(i + 1) * C_SIZE + j + 1] = norms[i * C_SIZE + j] + norm1 + norm2;
        }
    }
    for (int i = 0; i < norms.size(); i++) {
        norms[i] = norms[i].normalized();
    }

    for (int i = 0; i < C_SIZE - 1; i++) {
        for (int j = 0; j < C_SIZE - 1; j++) {
            int index = i * C_SIZE + j;
            Vector3f v1 = state[2 * index];
            Vector3f v2 = state[2 * (index + 1)];
            Vector3f v3 = state[2 * (index + 1 + C_SIZE)];
            Vector3f v4 = state[2 * (index + C_SIZE)];
            Vector3f n1 = norms[index];
            Vector3f n2 = norms[index + 1];
            Vector3f n3 = norms[index + 1 + C_SIZE];
            Vector3f n4 = norms[index + C_SIZE];
            glBegin(GL_TRIANGLES);

            glNormal3f(n1[0], n1[1], n1[2]);
            glVertex3f(v1[0], v1[1], v1[2]);
            glNormal3f(n3[0], n3[1], n3[2]);
            glVertex3f(v3[0], v3[1], v3[2]);
            glNormal3f(n2[0], n2[1], n2[2]);
            glVertex3f(v2[0], v2[1], v2[2]);

            glNormal3f(n1[0], n1[1], n1[2]);
            glVertex3f(v1[0], v1[1], v1[2]);
            glNormal3f(n4[0], n4[1], n4[2]);
            glVertex3f(v4[0], v4[1], v4[2]);
            glNormal3f(n3[0], n3[1], n3[2]);
            glVertex3f(v3[0], v3[1], v3[2]);

            glNormal3f(-n2[0], -n2[1], -n2[2]);
            glVertex3f(v2[0], v2[1], v2[2]);
            glNormal3f(-n3[0], -n3[1], -n3[2]);
            glVertex3f(v3[0], v3[1], v3[2]);
            glNormal3f(-n1[0], -n1[1], -n1[2]);
            glVertex3f(v1[0], v1[1], v1[2]);

            glNormal3f(-n3[0], -n3[1], -n3[2]);
            glVertex3f(v3[0], v3[1], v3[2]);
            glNormal3f(-n4[0], -n4[1], -n4[2]);
            glVertex3f(v4[0], v4[1], v4[2]);
            glNormal3f(-n1[0], -n1[1], -n1[2]);
            glVertex3f(v1[0], v1[1], v1[2]);

            glEnd();
        }
    }
}



