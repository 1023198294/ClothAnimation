#include "particleSystem.h"

ParticleSystem::ParticleSystem(int nParticles) : wind(false), display_spring(true), move(false), velocity(-2.0f),
                                                 distance(4.0), m_numParticles(nParticles) {

}

void ParticleSystem::switch_draw_mode() {
    this->display_spring = !this->display_spring;
}

void ParticleSystem::switch_move() {
    this->move = !this->move;
}

void ParticleSystem::switch_wind() {
    this->wind = !this->wind;
}

void ParticleSystem::velocity_negative() {
    this->velocity = -this->velocity;
}

