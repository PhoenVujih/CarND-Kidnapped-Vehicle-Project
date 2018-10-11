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

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// Set particle number
	num_particles = 100;

	// Extract standard deviation of x,y,theta
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];

	// Create normal (Gaussian) distributions for x, y and theta
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	for (int i=0; i<num_particles; i++){
		Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1.0;
		particles.push_back(particle);
		weights.push_back(1.0);

	}
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Extract standard deviation of x,y,theta
	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	// Create normal (Gaussian) distributions for x, y and theta
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);

	for (int i=0; i<num_particles; i++) {
		double theta = particles[i].theta;
		if (yaw_rate < 0.00001){
			particles[i].x += velocity * delta_t * cos(theta) + dist_x(gen);
			particles[i].y += velocity * delta_t * sin(theta) + dist_y(gen);
		}
		else {
			particles[i].x += velocity * (sin(theta+yaw_rate*delta_t) - sin(theta)) / yaw_rate + dist_x(gen);
      particles[i].y += velocity * (cos(theta) - cos(theta+yaw_rate*delta_t)) / yaw_rate + dist_y(gen);
		}
		particles[i].theta += yaw_rate * delta_t + dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

	for(unsigned int i=0; i<observations.size(); i++){
		// Initialize the containers of minimum distance and ID
    double min_distance = 99999999;
    int closest_ID = -1;
		double observe_x = observations[i].x;
		double observe_y = observations[i].y;
    for(unsigned int j=0; j<predicted.size(); j++){
			double predict_x = predicted[j].x;
			double predict_y = predicted[j].y;
			double distance = dist(observe_x, observe_y, predict_x, predict_y);
      if(distance < min_distance){
        min_distance = distance;
        closest_ID = j;
      }
    }
    observations[i].id = closest_ID;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	double sum_weight = 0.0;

	for (int i=0; i<num_particles; i++) {
		// Extract parameters of particles
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;

		// Filter and convert the map landmarks
		vector<LandmarkObs> map_legal_landmarks;
		double sensor_range_2 = sensor_range * sensor_range;
		for (unsigned int j=0; j<map_landmarks.landmark_list.size(); j++) {
			float landmark_x = map_landmarks.landmark_list[j].x_f;
			float landmark_y = map_landmarks.landmark_list[j].y_f;
			int landmark_id = map_landmarks.landmark_list[j].id_i;
			double delta_x = x - landmark_x;
			double delta_y = y - landmark_y;
			if (delta_x*delta_x + delta_y*delta_y <= sensor_range_2) {
				map_legal_landmarks.push_back(LandmarkObs {landmark_id, landmark_x, landmark_y});
			}
		}

		// Tansform observation coordinates to map coordinates
		vector<LandmarkObs> map_obs_landmarks;
		for (unsigned int j=0; j<observations.size(); j++) {
			LandmarkObs transformed_p;
			transformed_p.id = j;
			transformed_p.x = x + (cos(theta) * observations[j].x) - (sin(theta) * observations[j].y);
      transformed_p.y = y + (sin(theta) * observations[j].x) + (cos(theta) * observations[j].y);
      map_obs_landmarks.push_back(transformed_p);
		}

		// Data association
		dataAssociation(map_legal_landmarks, map_obs_landmarks);

		// Weight calculation of the particles
		double sigma_x_2 = std_landmark[0] * std_landmark[0];
		double sigma_y_2 = std_landmark[1] * std_landmark[1];
		double coeff_k = 1.0 / (2.0 * M_PI * std_landmark[0] * std_landmark[1]);
		particles[i].weight = 1.0;

		for (unsigned int k=0; k<map_obs_landmarks.size(); k++) {
			double map_obs_x = map_obs_landmarks[k].x;
			double map_obs_y = map_obs_landmarks[k].y;
			double map_landmark_index = map_obs_landmarks[k].id;

			double map_landmark_x = map_legal_landmarks[map_landmark_index].x;
			double map_landmark_y = map_legal_landmarks[map_landmark_index].y;

			double dx = map_obs_x - map_landmark_x;
			double dy = map_obs_y - map_landmark_y;

			double landmark_weight = coeff_k * exp(-1.0 * (dx * dx / (2.0 * sigma_x_2) + dy * dy /(2.0 * sigma_y_2)));

			particles[i].weight *= landmark_weight;

		}
		sum_weight += particles[i].weight;
	}

	// Normalize the Weights
	for (unsigned int m=0; m<particles.size(); m++) {
//		particles[m].weight /= sum_weight;
		weights[m] = particles[m].weight;
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	discrete_distribution<> distribution(weights.begin(), weights.end());
	vector<Particle> resampled_particles;

	int index;
	for (int i=0; i<num_particles; i++) {
		index = distribution(gen);
		resampled_particles.push_back(particles[index]);
	}
	particles = resampled_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
