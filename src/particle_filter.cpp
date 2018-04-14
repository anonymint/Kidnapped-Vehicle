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
	num_particles = 100;

	//inital random and norm distribution
	std::default_random_engine gen;
	std::normal_distribution<double> norm_x(x, std[0]);
	std::normal_distribution<double> norm_y(y, std[1]);
	std::normal_distribution<double> norm_theta(theta, std[2]);

	for (int i=0; i < num_particles; i++) {
		Particle particle;
		particle.id = i;
		particle.x = norm_x(gen);
		particle.y = norm_y(gen);
		particle.theta = norm_theta(gen);
		particle.weight = 1;
		particles.push_back(particle);
		weights.push_back(1);
	}

	//set flag initialize to true
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::default_random_engine gen;

	for (int i=0; i < num_particles; i++) {
		double predict_new_x;
		double predict_new_y;
		double predict_new_theta;
		if (yaw_rate == 0) {
			predict_new_x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
			predict_new_y = particles[i].y + velocity*delta_t*sin(particles[i].theta);
			predict_new_theta = particles[i].theta;
		} else {
			predict_new_x = particles[i].x + velocity/yaw_rate*(sin(particles[i].theta+yaw_rate*delta_t) - sin(particles[i].theta));
			predict_new_y = particles[i].y + velocity/yaw_rate*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
			predict_new_theta = particles[i].theta + yaw_rate*delta_t;
		}

		normal_distribution<double> norm_x(predict_new_x, std_pos[0]);
		normal_distribution<double> norm_y(predict_new_y, std_pos[1]);
		normal_distribution<double> norm_theta(predict_new_theta, std_pos[2]);

		// set new value of x, y and theta back with noise!
		particles[i].x = norm_x(gen);
		particles[i].y = norm_y(gen);
		particles[i].theta = norm_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

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

	for (int i = 0; i < particles.size(); i++) {
		std::vector<LandmarkObs> observations_transf;
		Particle p = particles[i];
		for (LandmarkObs obs : observations) {
			LandmarkObs trans_obj;
			trans_obj.id = obs.id;
			trans_obj.x = p.x + cos(p.theta)*obs.x - sin(p.theta)*obs.y;
			trans_obj.y = p.y + sin(p.theta)*obs.x + cos(p.theta)*obs.y;
			observations_transf.push_back(trans_obj);
		}

		p.weight = 1.0;

		std::vector<int> associations;
		std::vector<double> sense_x;
		std::vector<double> sense_y;
		for(LandmarkObs t_obs: observations_transf) {
			int association = 0;
			double closest_dist = sensor_range*sensor_range;
			for (int j=0; j < map_landmarks.landmark_list.size(); j++) {
				Map::single_landmark_s s_landmark = map_landmarks.landmark_list[j];	

				//check map within range
				double dist_from_particle = dist(p.x, p.y, s_landmark.x_f, s_landmark.y_f);
				if (dist_from_particle <= sensor_range) {
					double dist_obs_map =  dist(t_obs.x, t_obs.y, s_landmark.x_f, s_landmark.y_f);
					if (dist_obs_map < closest_dist) {
						closest_dist = dist_obs_map;
						association = j;
					}
				}				
			}

			if (association != 0) {
				double meas_x = t_obs.x;
				double meas_y = t_obs.y;
				double mu_x = map_landmarks.landmark_list[association].x_f;
				double mu_y = map_landmarks.landmark_list[association].y_f;
				double sig_x = std_landmark[0];
				double sig_y = std_landmark[1];
				long double gauss_norm = 1/(2*M_PI*sig_x*sig_y);
				long double exponent = (pow(meas_x-mu_x,2)/2*pow(sig_x,2)) + (pow(meas_y-mu_y,2)/2*pow(sig_y,2));
				long double multiplier = gauss_norm*exp(-(exponent));
				if (multiplier > 0) {
					p.weight *= multiplier;
					weights[i] = p.weight;
				}
			}

			associations.push_back(association);
			sense_x.push_back(t_obs.x);
			sense_y.push_back(t_obs.y);
		}	

		SetAssociations(p, associations, sense_x, sense_y);			
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::vector<Particle> resampling_particles;
	std::default_random_engine gen;
	for(int i=0; i< num_particles; i++) {
		std::discrete_distribution<int> index(weights.begin(), weights.end());
		resampling_particles[i] = particles[index(gen)];
	}

	// replace current particles
	particles = resampling_particles;
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
