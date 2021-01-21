#include "../src/particle_filter.h"
#include "../src/helper_functions.h"
#include "../src/map.h"
#include <iostream>

int main()
{
    ParticleFilter particle_filter{};
    //double sigma_pos [3] = {0.3, 0.3, 0.01};
    double sigma_pos [3] = {0., 0., 0.};
    //double v_pos [2] = {0.1, 0.1};
    double v_pos [2] = {0., 0.};
    const size_t particle_num{5u};
    particle_filter.init(0, 0, 0, sigma_pos, particle_num);
    if (particle_filter.particles.size() != particle_num)
    {
        std::cout << "Wrong number of particles" << std::endl;
        return -1;
    }
    std::cout << "Initialize" << std::endl;
    particle_filter.printParticles();
    std::cout << std::endl;
    particle_filter.prediction(1, v_pos, 10., 0.);
    std::cout << "Predict" << std::endl;
    particle_filter.printParticles();
    std::cout << std::endl;
    particle_filter.particles[0].weight = 10;
    particle_filter.particles[1].weight = 10;
    particle_filter.resample();
    //particle_filter.printParticles();
}
