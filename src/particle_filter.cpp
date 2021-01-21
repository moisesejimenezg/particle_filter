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
    if (std::fabs(theta_dot) > std::numeric_limits<double>::epsilon())
    {
        p.x += v_d * (std::sin(p.theta + theta_d) - std::sin(p.theta));
        p.y += v_d * (std::cos(p.theta) - std::cos(p.theta + theta_d));
    }
    else
    {
        p.x += v * dt * std::cos(p.theta);
        p.y += v * dt * std::sin(p.theta);
    }
    p.theta += theta_d;
}

LandmarkObs transformToMapCoordinates(const LandmarkObs& observation, const Particle& particle)
{
    LandmarkObs new_observation{};
    new_observation.x = particle.x + std::cos(particle.theta) * observation.x -
                        std::sin(particle.theta) * observation.y;
    new_observation.y = particle.y + std::sin(particle.theta) * observation.x +
                        std::cos(particle.theta) * observation.y;
    return new_observation;
}

std::vector<double> extractWeights(const std::vector<Particle>& particles)
{
    std::vector<double> weights{};
    for (const auto& p : particles)
    {
        weights.emplace_back(p.weight);
    }
    return weights;
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
    num_particles = num_particles;
    for (auto i{0}; i < num_particles; ++i)
    {
        Particle particle{};
        particle.id = -1;
        particle.x = x;
        particle.y = y;
        particle.theta = theta;
        particle.weight = 1.;
        particles.emplace_back(particle);
    }
    addGaussianNoise(std);
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    for (auto& particle : particles)
    {
        moveParticle(particle, delta_t, velocity, yaw_rate);
    }
    addGaussianNoise(std_pos);
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations)
{
    auto min_dist{std::numeric_limits<double>::max()};
    for (auto& observation : observations)
    {
        for (const auto predicted_meas : predicted)
        {
            const auto new_dist{
                dist(predicted_meas.x, predicted_meas.y, observation.x, observation.y)};
            if (new_dist < min_dist)
            {
                observation.id = predicted_meas.id;
                min_dist = new_dist;
            }
        }
        min_dist = std::numeric_limits<double>::max();
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs>& observations,
                                   const Map& map_landmarks)
{
    for (auto& particle : particles)
    {
        std::vector<LandmarkObs> landmarks_in_range{};
        for (const auto landmark : map_landmarks.landmark_list)
        {
            if (dist(particle.x, particle.y, landmark.x_f, landmark.y_f) < sensor_range)
            {
                landmarks_in_range.emplace_back(
                    LandmarkObs{landmark.id_i, landmark.x_f, landmark.y_f});
            }
        }
        std::vector<LandmarkObs> transformed_observations{};
        for (auto& observation : observations)
        {
            transformed_observations.emplace_back(transformToMapCoordinates(observation, particle));
        }
        dataAssociation(landmarks_in_range, transformed_observations);
        auto weight = 1;
        particle.weight = 1;
        for (const auto& observation : transformed_observations)
        {
            for (const auto& landmark : landmarks_in_range)
            {
                if (landmark.id == observation.id)
                {
                    weight *= multiv_prob(std_landmark[0], std_landmark[1], observation.x,
                                          observation.y, landmark.x, landmark.y);
                    break;
                }
            }
            particle.weight *= weight == 0 ? std::numeric_limits<double>::epsilon() : weight;
        }
    }
}

void ParticleFilter::resample()
{
    const auto weights{extractWeights(particles)};
    std::discrete_distribution<int> distribution{weights.cbegin(), weights.cend()};
    decltype(particles) new_particles{};
    for (auto i{0u}; i < particles.size(); ++i)
    {
        new_particles.emplace_back(particles[distribution(gen)]);
    }
    particles = new_particles;
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

void ParticleFilter::addGaussianNoise(double std[])
{
    std::normal_distribution<double> distribution_x{0, std[0]};
    std::normal_distribution<double> distribution_y{0, std[1]};
    std::normal_distribution<double> distribution_theta{0, std[2]};
    for (auto& particle : particles)
    {
        particle.x += distribution_x(gen);
        particle.y += distribution_y(gen);
        particle.theta += distribution_theta(gen);
    }
}
