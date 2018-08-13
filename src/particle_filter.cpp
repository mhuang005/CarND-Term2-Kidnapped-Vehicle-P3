/*
 * particle_filter.cpp
 *
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
#include <limits>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	
	if (is_initialized) return;

	// Number of particles used for the filter
	num_particles = 10;

	
    // Generate Gaussian (sensor) noise for x, y and theta
    default_random_engine gen;
    normal_distribution<double> dist_x(0, std[0]);
    normal_distribution<double> dist_y(0, std[1]);
    normal_distribution<double> dist_theta(0, std[2]);

    for (int i = 0; i < num_particles; i++) {

      // create a particle and store it in the particle array	
      Particle p{ i,  // id
      	          x + dist_x(gen),
      	          y + dist_y(gen),
      	          theta + dist_theta(gen),
                  1.0  // weight
      }; 
      particles.push_back(p);
      
      weights.push_back(1.0);
    }
    
    // Done the initialization
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

    // Generate Gaussian (sensor) noise for x, y and theta
	default_random_engine gen;
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);
    
    for (auto& p: particles) {
      
      // Predicte x, y and theta based on motion model
      if (fabs(yaw_rate) > 1e-5) { // Avoid division by 0
        p.x += velocity/yaw_rate * (sin(p.theta + yaw_rate*delta_t) - sin(p.theta));
        p.y += velocity/yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate*delta_t));
		p.theta += yaw_rate * delta_t;
      } 
      else {
        p.x += velocity * delta_t * cos(p.theta);
        p.y += velocity * delta_t * sin(p.theta);
      }

      // Add sensor noise
      p.x += dist_x(gen);
      p.y += dist_y(gen);
      p.theta += dist_theta(gen);

    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	
  // For each observation/landmark, search the nearest predicted measurement
  // and assign its id to the observation's id
  for (auto& obs: observations) {
  	double min_dist = numeric_limits<double>::infinity();
    for (auto& pred: predicted) {
      double dist = (obs.x-pred.x)*(obs.x-pred.x) + (obs.y-pred.y)*(obs.y-pred.y);
      if (min_dist > dist) {
        obs.id = pred.id;
        min_dist = dist;
      }	 
    }
  }	
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.
    
    int idx = 0; // Track the particle's index in the vector
    double dist = sensor_range * sensor_range;
    double std_x = std_landmark[0];
    double std_y = std_landmark[1];
    double var_x = std_x * std_x;
    double var_y = std_y * std_y;
    double normalizer = 0.5 / (M_PI * std_x * std_y);

	for (auto& p: particles) {
      // Transform observations from (local) car/particle coordinates to (global) map coordinates
	  vector<LandmarkObs> obs_meas;
	  for (const auto& obs: observations) {
        double x_map = p.x + obs.x * cos(p.theta) - obs.y * sin(p.theta); 
        double y_map = p.y + obs.x * sin(p.theta) + obs.y * cos(p.theta);
        obs_meas.push_back(LandmarkObs{-1, x_map, y_map});
	  } 	
      
      // Choose the landmarks in a range
      vector<LandmarkObs> pred_meas;
      for (const auto& landmark: map_landmarks.landmark_list) {
       
        double diff_x = p.x - landmark.x_f;
        double diff_y = p.y - landmark.y_f;
        if (diff_x*diff_x + diff_y*diff_y < dist) {
          pred_meas.push_back(LandmarkObs{landmark.id_i, landmark.x_f, landmark.y_f});	
        }  
      }

      dataAssociation(pred_meas, obs_meas);

      // Calculate the particle's weight
      p.weight = 1.0; 
      for (const auto& obs: obs_meas) {
        for (const auto& pred: pred_meas) {
          if (obs.id == pred.id) {
		    double diff_x = pred.x - obs.x;
			double diff_y = pred.y - obs.y;
            double prob = normalizer * exp(-0.5*diff_x*diff_x/var_x - 0.5*diff_y*diff_y/var_y);
            if (prob == 0) prob = 1e-6;
            p.weight *= prob;
            break;  
          }	
        }
      }

      weights[idx] = p.weight;
      idx++;  
	}
}


void ParticleFilter::resample() {

	// Implement the resampling wheel algorithm
    default_random_engine gen;
    uniform_int_distribution<int> dist_i(0, num_particles-1);

    int idx = dist_i(gen);
    double beta = 0;
    double max_w = *max_element(weights.begin(), weights.end());

    uniform_real_distribution<double> dist_r(0, max_w); 
	vector<Particle> resampled_particles;

	for (int i = 0; i < num_particles; i++) {
      beta += 2.0 * dist_r(gen);
      while (beta > weights[idx]) {
        beta -= weights[idx];
        idx = (idx + 1) % num_particles;	
      }
      resampled_particles.push_back(particles[idx]);
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

    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

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
