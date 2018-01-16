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
#include "helper_functions.h"
#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	// Number of particles to draw
		
	num_particles = 10; 
	
	default_random_engine gen;	
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	
	particles.resize(num_particles);
	for(int i=0; i<num_particles; i++){
			particles[i].id = i;
			particles[i].x = dist_x(gen);
			particles[i].y = dist_y(gen);
			particles[i].theta = dist_theta(gen);
			particles[i].weight = 1.0;
	}
	
	weights.resize(num_particles);
	for (int i=0; i<num_particles; i++){
		weights[i]=1.0;
	}
	// Flag, if filter is initialized
	is_initialized = true;
	return;
	

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	double v = velocity;
	double yawd = yaw_rate;
	default_random_engine gen;
	for(int i=0; i<num_particles; i++){
		double p_x = particles[i].x	;
		double p_y = particles[i].y;
		double yaw = particles[i].theta;
		  		  
	   //predicted state values
	   double px_p, py_p;
	   //avoid division by zero
	   if (fabs(yawd) > 0.001) {
		   px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
		   py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
	   }
	   else {
		   px_p = p_x + v*delta_t*cos(yaw);
		   py_p = p_y + v*delta_t*sin(yaw);
	   }
		
	   double yaw_p = yaw + yawd*delta_t;
	
	   	
	   	normal_distribution<double> dist_x(px_p, std_pos[0]);
	   	normal_distribution<double> dist_y(py_p, std_pos[1]);
	   	normal_distribution<double> dist_theta(yaw_p, std_pos[2]);
		   
	   	
	   	particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
		bool isdebug=false;
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
		
	for(int i=0;i<particles.size();i++){
//		cout<<particles[i].weight;
//		cout<<endl<<i<<endl;
		float prob_obs = 1.0;
				
		float px = particles[i].x;
		float py = particles[i].y;
		float theta = particles[i].theta;
//		cout<<prob_obs;
		if(isdebug==true) cout<<"Considering particle: "<<particles[i].id<<" Its configutaion is: "<<px<<" , "<<py<<" , "<<theta<<endl;  

		for(int j = 0; j<observations.size(); j++){
			//Transform observation to map frame to find the closed landmark
			
			float ox_veh = observations[j].x;
			float oy_veh = observations[j].y;
			
			if(isdebug==true) cout<<"\t        lets see observation: "<<j<<" which is:"<<ox_veh<<","<<oy_veh<<endl;

			float ox_map = ox_veh*cos(theta) - oy_veh*sin(theta) + px;
			float oy_map = ox_veh*sin(theta) + oy_veh*cos(theta) + py;
			
			//Find the nearest landmark to this observation in the map
			float minDist = 1000.0;
			float nearest_lx_map;
			float nearest_ly_map;
			for(int m=0; m<map_landmarks.landmark_list.size();m++){
				float lx_map = map_landmarks.landmark_list[m].x_f;
				float ly_map = map_landmarks.landmark_list[m].y_f;

				float distance = dist(ox_map,oy_map, lx_map,ly_map);
				if(distance<minDist){
					minDist = distance;
					nearest_lx_map = lx_map;
					nearest_ly_map = ly_map;				
				}
			}
			if(isdebug==true) cout<<"				closest landmark is: "<<nearest_lx_map<<","<<nearest_ly_map<<" at a distance of: " <<minDist;
			float sx = std_landmark[0];
			float sy = std_landmark[1];
			float mux = nearest_lx_map - ox_map;
			float muy = nearest_ly_map - oy_map;
			

			float numx = pow(mux, 2.0);
			float denx = 2.0*pow(sx,2.0);
//			cout<<"numx is: "<< numx<< "  denx is:" << denx<<"    ";			
		
//					
			float numy = pow(muy, 2.0);
			float deny = 2.0*pow(sy,2.0);
//			
			if(dist(nearest_lx_map,nearest_ly_map,px,py)<sensor_range)
				prob_obs *= (1/(2.0*3.14*sx*sy))*exp(-( (numx/denx) + (numy/deny) ));




//			prob_obs*= ( 1 / ( sx * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (mux-ox_veh)/sx, 2.0 ) );
//			prob_obs*= ( 1 / ( sy * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (muy-oy_veh)/sy, 2.0 ) );


//			double prob_obs_temp = ( 1 / ( sx * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (mux-ox_veh)/sx, 2.0 ) );
//				if(isdebug==true) cout<<"                 \t"<<prob_obs_temp<<"\t";
//			if(prob_obs_temp>0.000001){
//				prob_obs*=prob_obs_temp;
//			}

//			prob_obs_temp = ( 1 / ( sy * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (muy-oy_veh)/sy, 2.0 ) );
//			if(isdebug==true) cout<<"                 \t"<<prob_obs_temp<<"\t"<<endl;
//			if(prob_obs_temp>0.000001){
//				prob_obs*=prob_obs_temp;
//			}
					
			
	
		}
		particles[i].weight = prob_obs;
		weights[i] = prob_obs;
		if(isdebug==true) cout<<"Particle ID:"<<particles[i].id<<" Weight:"<<particles[i].weight<<endl;
		if(isdebug==true) cout<<endl;
	}	


	
			
	
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
//	 vector<Particle> particles_temp;
//	 particles_temp.resize(num_particles);
	 
	 default_random_engine gen;
	 uniform_real_distribution<double> distribution(0.0,1.0);
	 double number = distribution(gen);
	 
	 int index = number*num_particles;
//	 cout<<"index is:"<<index<<"\t";
	 float beta = 0.0;
	 float maxWeight = *max_element(weights.begin(), weights.end());
//	 cout<<maxWeight;
	 
	 for(int i=0; i<particles.size(); i++){
		 double ran = distribution(gen);
//		 cout<<ran;
		 beta += ran*2.0*maxWeight;
		 while(beta>weights[index]){
			 beta-=weights[index];
			 index = (index + 1)%num_particles;
		 }
//		 particles_temp.push_back(particles[index]);
		 particles[i].id = particles[index].id;
		 particles[i].x = particles[index].x;
		 particles[i].y = particles[index].y;
		 particles[i].theta = particles[index].theta;
		 particles[i].weight = particles[index].weight;
	 }
//	 particles = particles_temp;
	 
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
