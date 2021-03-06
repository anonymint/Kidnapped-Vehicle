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

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

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
				int map_id = map_landmarks.landmark_list[association].id_i;
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

				associations.push_back(map_id);
				sense_x.push_back(mu_x);
				sense_y.push_back(mu_y);
			} else {
				p.weight *= numeric_limits<double>::min();
				weights[i] = p.weight;
			}
		}

		// reassign p back to particles
		particles[i] = p;
		SetAssociations(particles[i], associations, sense_x, sense_y);
			
	}
}

void ParticleFilter::resample() {

	std::vector<Particle> resampling_particles;
	std::default_random_engine gen;
	for(int i=0; i< num_particles; i++) {
		std::discrete_distribution<int> index(weights.begin(), weights.end());		
		resampling_particles.push_back(particles[index(gen)]);
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

	return particle;
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
