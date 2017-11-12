/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <cassert>
#include "particle_filter.h"

namespace {
    std::default_random_engine& get_engine()
    {
        static std::default_random_engine s_engine;
        return s_engine;
    }
}

void ParticleFilter::init(double x, double y, double theta, double std[])
{
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
    //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    num_particles = 10;

    auto& gen = get_engine();

    // define normal distributions for sensor noise
    std::normal_distribution<double> N_x_init(x, std[0]);
    std::normal_distribution<double> N_y_init(y, std[1]);
    std::normal_distribution<double> N_theta_init(theta, std[2]);

    int id = 0;
    std::generate_n(
        std::back_inserter(particles),
        num_particles,
        [&]() {
            return Particle{id++, N_x_init(gen), N_y_init(gen), N_theta_init(gen), 1.0};
        });

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/std::normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    std::normal_distribution<double> N_x(0, std_pos[0]);
    std::normal_distribution<double> N_y(0, std_pos[1]);
    std::normal_distribution<double> N_theta(0, std_pos[2]);

    auto& gen = get_engine();

    if (fabs(yaw_rate) < 0.00001) {
        for (auto& p : particles) {
            p.x += velocity * delta_t * cos(p.theta) + N_x(gen);
            p.y += velocity * delta_t * sin(p.theta) + N_y(gen);
        }
    } else {
        for (auto& p : particles) {
            p.x += velocity / yaw_rate * (sin(p.theta + yaw_rate*delta_t) - sin(p.theta)) + N_x(gen);
            p.y += velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate*delta_t)) + N_y(gen);
            p.theta += yaw_rate * delta_t + N_theta(gen);
        }
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations)
{
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
    //   implement this method and use it as a helper during the updateWeights phase.

    for (auto& o : observations) {
        auto min_dist = std::numeric_limits<double>::max();
        int min_dist_id;
        for (auto const& p : predicted) {
            double cur_dist = dist(o.x, o.y, p.x, p.y);
            if (cur_dist < min_dist) {
                min_dist = cur_dist;
                min_dist_id = p.id;
            }
        }
        o.id = min_dist_id;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
    const std::vector<LandmarkObs> &observations, const Map &map_landmarks)
{
    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_std::normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation 
    //   3.33
    //   http://planning.cs.uiuc.edu/node99.html

    for (auto& p : particles) {

        // predictions = all landmarks predicted to be within sensor range of the particle:
        std::vector<LandmarkObs> predictions;
        std::transform(
            map_landmarks.landmark_list.begin(),
            map_landmarks.landmark_list.end(),
            std::back_inserter(predictions),
            [&](Map::single_landmark_s const& lm) {
                return LandmarkObs{
                    lm.id_i,
                    lm.x_f,
                    lm.y_f
                };
            });
        predictions.erase( std::remove_if(predictions.begin(), predictions.end(), 
            [&](LandmarkObs const& lmo) {
                return !(
                    fabs(lmo.x - p.x) <= sensor_range &&
                    fabs(lmo.y - p.y) <= sensor_range
                    );
            }), 
            predictions.end());

        // observations_tr = transformed observations from vehicle coordinates to map coordinates:
        std::vector<LandmarkObs> observations_tr;
        std::transform(
            observations.begin(), 
            observations.end(), 
            std::back_inserter(observations_tr),
            [&](LandmarkObs const& lmo) {
                return LandmarkObs{
                    lmo.id,
                    cos(p.theta) * lmo.x - sin(p.theta) * lmo.y + p.x,
                    sin(p.theta) * lmo.x + cos(p.theta) * lmo.y + p.y
                };
            });

        // do it:
        dataAssociation(predictions, observations_tr);

        // Update weight for this particle:
        p.weight = 1.;
        for (auto& o : observations_tr) {
            // find compatible prediction:
            auto lm_pi = std::find_if(predictions.begin(), predictions.end(), 
                [&](LandmarkObs const& p) {
                    return p.id == o.id;
                });
            assert(lm_pi != predictions.end());
            auto const& lm_p = *lm_pi;
            // calculate weight for this observation with multivariate Gaussian
            double s_x = std_landmark[0];
            double s_y = std_landmark[1];
            double obs_w = (1 / (2 * M_PI*s_x*s_y)) * exp(-(pow(lm_p.x - o.x, 2) / (2 * pow(s_x, 2)) + (pow(lm_p.y - o.y, 2) / (2 * pow(s_y, 2)))));

            // total observations weight:
            p.weight *= obs_w;
        }
    }
}

void ParticleFilter::resample()
{
    // TODO: Resample particles with replacement with probability proportional to their weight. 
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    auto& gen = get_engine();

    std::vector<Particle> new_particles;

    // all current weights:
    std::vector<double> weights;
    for (auto const& p : particles) {
        weights.push_back(p.weight);
    }

    // generate random starting index for resampling wheel
    std::uniform_int_distribution<int> uniintdist(0, num_particles - 1);
    auto index = uniintdist(gen);

    // get max weight
    double max_weight = *std::max_element(weights.begin(), weights.end());

    // uniform random distribution [0.0, max_weight)
    std::uniform_real_distribution<double> unirealdist(0.0, max_weight);

    // spin the resample wheel!
    double beta = 0.0;
    for (int i = 0; i < num_particles; i++) {
        beta += unirealdist(gen) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
    }

    particles.swap(new_particles);
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

using namespace std;

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
    vector<double> v = best.sense_x;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<double>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
    vector<double> v = best.sense_y;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<double>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}
