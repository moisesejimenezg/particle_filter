#include "../src/particle_filter.h"
#include <iostream>

int main()
{
    ParticleFilter particle_filter{};
    double sigma_pos [3] = {0.3, 0.3, 0.01};
    particle_filter.init(0, 0, 0, sigma_pos, 5);
    particle_filter.printParticles();
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;
    particle_filter.particles[0].weight = 10;
    particle_filter.particles[1].weight = 10;
    particle_filter.resample();
    particle_filter.printParticles();
}
