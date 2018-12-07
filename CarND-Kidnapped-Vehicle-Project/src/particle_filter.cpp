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

	// set number of particles
	num_particles = 100;

	// resize weights vector based on num_particles
	weights.resize(num_particles);

	// resize vector of particles
	particles.resize(num_particles);

	// creates normal (Gaussian) distributions for x, y, theta
	normal_distribution<double>	dist_x(x,std[0]);
	normal_distribution<double>	dist_y(y,std[1]);
	normal_distribution<double>	dist_theta(theta,std[2]);

	// random engine for later generation of particles
	default_random_engine gen;

	// init particles
	for (int i = 0; i < num_particles; i++){

		particles[i].id = i;
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		particles[i].weight = 1.0;

	}

	// complite initialization
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// creates normal (Gassian) distributions for adding noise
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	// random engine for later generation of particles
	default_random_engine gen;

	// predict state
	for (int i = 0; i < num_particles; i++){

		// if yaw rate is too small (i.e., yaw rate is zero)
		if (fabs(yaw_rate) < 0.000001){
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		}
		else{
			particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}

		// add noise
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);

	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	for (unsigned int i = 0; i < observations.size(); i++){

		// initialize minimun distance to be very large
		double mindistance = numeric_limits<double>::max();

		// initialize map id 
		int map_id = -1;

		// for each prediction
		for (unsigned j = 0; j < predicted.size(); j++){

			double dX = observations[i].x - predicted[j].x;
			double dY = observations[i].y - predicted[j].y;
			double distance = dX*dX + dY*dY;

			// find the predicted landmark nearest to the current observed landmark
			if (distance < mindistance){
				mindistance = distance;
				map_id = predicted[j].id;
			}
		}

		// update the observation id
		observations[i].id = map_id;
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

	// update weights for each particles
	for (int i = 0; i < num_particles; i++){
		
		// get the state of the particle
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;

		// particle range
		double p_range = sensor_range * sensor_range;

		// create a vector to hold the map landmark information that exist within the particle range
		vector<LandmarkObs> inRangeLandmarks;

		// for each landmark
		for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++){

			// get information of the landmark
			float lm_x = map_landmarks.landmark_list[j].x_f;
			float lm_y = map_landmarks.landmark_list[j].y_f;
			int lm_id = map_landmarks.landmark_list[j].id_i;
			
			// position difference between the particle and the landmark
			double dX = p_x - lm_x;
			double dY = p_y - lm_y;

			// check whether the landmark is within the range
			if (dX*dX + dY*dY <= p_range){
				inRangeLandmarks.push_back(LandmarkObs{lm_id,lm_x,lm_y});
			}
		}

		// transform the observartion coordinates 
		vector<LandmarkObs> mappedObservations;
		for (unsigned int k = 0; k < observations.size(); k++){
			double t_x = cos(p_theta)*observations[k].x - sin(p_theta)*observations[k].y + p_x;
			double t_y = sin(p_theta)*observations[k].x + cos(p_theta)*observations[k].y + p_y;
			mappedObservations.push_back(LandmarkObs{observations[k].id,t_x,t_y});
		}

		// data association for the particle
		dataAssociation(inRangeLandmarks, mappedObservations);

		// reset weight
		particles[i].weight = 1.0;

		// calculate weight
		for (unsigned int l = 0; l < mappedObservations.size(); l++){
			
			// get the transformed observation information
			double to_x = mappedObservations[l].x;
			double to_y = mappedObservations[l].y;
			int to_id = mappedObservations[l].id;

			// place holder for the prediction coordinates
			double pr_x, pr_y;

			
			// get the coordinates of the prediction associated with the current observation
			unsigned int o = 0;
			bool found = false;
			while (!found && o < inRangeLandmarks.size()){
				if (inRangeLandmarks[o].id == to_id){
					found = true;
					pr_x = inRangeLandmarks[o].x;
					pr_y = inRangeLandmarks[o].y;
				}
				o++;
			}
			
			double dX = to_x - pr_x;
			double dY = to_y - pr_y;

			// calculate the Multivariate Gaussian 
			double weight = 1/(2*M_PI*std_landmark[0]*std_landmark[1]) *
							exp(-((dX*dX)/(2*(std_landmark[0]*std_landmark[0])) + 
							(dY*dY)/(2*(std_landmark[1]*std_landmark[1]))));
			
			// calculate the particle's final weight
			if (weight < 1e-20){
				particles[i].weight *= 1e-20;	
			}
			else{
				particles[i].weight *= weight;
			}

			// update weight vector
			weights[i] = particles[i].weight;

		}

	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<Particle> resampled_particles;
	default_random_engine gen;

	// generate random starting index for resampling wheel
	uniform_int_distribution<int> uniintdist(0, num_particles-1);
	auto index = uniintdist(gen);

	// get max weight
	double max_weight = *max_element(weights.begin(),weights.end());

	// uniform random distribution
	uniform_real_distribution<double> unirealdist(0.0, max_weight);

	double beta = 0.0;

	// spin the resampling wheel
	for (int i = 0; i < num_particles; i++){
		beta += unirealdist(gen)*2.0;
		while (beta > weights[index])
		{
			beta -= weights[index];
			index = (index + 1)%num_particles;
		}
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
