/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

namespace
{
void moveParticle(Particle& p, const double dt, const double v, const double theta_dot)
{
    const auto theta_d{theta_dot * dt};
    const auto v_d{v / theta_dot};
    if (theta_dot > std::numeric_limits<double>::epsilon())
    {
        p.x += v_d * (std::sin(p.theta + theta_d) - std::sin(p.theta));
        p.y += v_d * (std::cos(p.theta) - std::sin(p.theta + theta_d));
    }
    else
    {
        p.x += v * dt * std::cos(p.theta);
        p.y += v * dt * std::sin(p.theta);
    }
    p.theta += theta_dot;
}
}  // namespace

void ParticleFilter::printParticles() const
{
    for (const auto particle : particles)
    {
        std::cout << particle << std::endl;
    }
}

void ParticleFilter::init(double x, double y, double theta, double std[], int num_particles)
{
    std::normal_distribution<double> distribution_x{0, std[0]};
    std::normal_distribution<double> distribution_y{0, std[1]};
    std::normal_distribution<double> distribution_theta{0, std[2]};
    num_particles = num_particles;
    for (auto i{0}; i < num_particles; ++i)
    {
        Particle particle{};
        particle.id = i;
        particle.x = x + distribution_x(gen);
        particle.y = y + distribution_y(gen);
        particle.theta = theta + distribution_theta(gen);
        particle.weight = 1.;
        particles.emplace_back(particle);
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    for (auto& particle : particles)
    {
        moveParticle(particle, velocity, yaw_rate, delta_t);
        std::normal_distribution<double> distribution_x{0, std_pos[0]};
        std::normal_distribution<double> distribution_y{0, std_pos[1]};
        std::normal_distribution<double> distribution_theta{0, std_pos[2]};
        particle.x += distribution_x(gen);
        particle.y += distribution_y(gen);
        particle.theta = distribution_theta(gen);
    }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations)
{
    /**
     * TODO: Find the predicted measurement that is closest to each
     *   observed measurement and assign the observed measurement to this
     *   particular landmark.
     * NOTE: this method will NOT be called by the grading code. But you will
     *   probably find it useful to implement this method and use it as a helper
     *   during the updateWeights phase.
     */
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs>& observations,
                                   const Map& map_landmarks)
{
    /**
     * TODO: Update the weights of each particle using a mult-variate Gaussian
     *   distribution. You can read more about this distribution here:
     *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
     * NOTE: The observations are given in the VEHICLE'S coordinate system.
     *   Your particles are located according to the MAP'S coordinate system.
     *   You will need to transform between the two systems. Keep in mind that
     *   this transformation requires both rotation AND translation (but no scaling).
     *   The following is a good resource for the theory:
     *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
     *   and the following is a good resource for the actual equation to implement
     *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
     */
}

void ParticleFilter::resample()
{
    /**
     * TODO: Resample particles with replacement with probability proportional
     *   to their weight.
     * NOTE: You may find std::discrete_distribution helpful here.
     *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
     */
}

void ParticleFilter::SetAssociations(Particle& particle, const vector<int>& associations,
                                     const vector<double>& sense_x, const vector<double>& sense_y)
{
    // particle: the particle to which assign each listed association,
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
    vector<double> v;

    if (coord == "X")
    {
        v = best.sense_x;
    }
    else
    {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}
