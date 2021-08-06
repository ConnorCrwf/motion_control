///////////////////////////////////////////////////////////////////////////////
//      Title     : encoder_mapping.cpp
//      Project   :	scrat_movement
//      Author    : Alex Navarro
//      Created   : 09/18/2020
//      Copyright : Copyright© The University of Texas at Austin, 2014-2021. All rights reserved.
//                
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////


#include "../include/wombotGen3_base/encoder_mapping.h"
// #include "encoder_mapping.h"
#include <iostream>
#include <assert.h>

using std::vector;
using std::list;
using std::cout;
using std::endl;

EncoderMap::EncoderMap(unsigned int wheel_count)
{
	N_ = wheel_count;

	abs_angles_rad_.resize(N_);
	abs_angles_deg_.resize(N_);
	last_abs_angle_.resize(N_);

	max_abs_encoder_.resize(N_);
	min_abs_encoder_.resize(N_);

	vel_delay_ = 100;

	start_time_ = std::chrono::steady_clock::now();	// Initialize clock time
	appendToHistory(vector<float>(N_));				// Initialize drive positions at 0
}

void EncoderMap::setAbsEncoderMax(vector<short int> max_vec) {max_abs_encoder_ = max_vec;}
void EncoderMap::setAbsEncoderMin(vector<short int> min_vec) {min_abs_encoder_ = min_vec;}
void EncoderMap::setAbsEncoderZero(vector<short int> zero_vec) {abs_encoder_zero_ = zero_vec;}
void EncoderMap::setVelocityDelay(double t) {vel_delay_ = t;}


/* Since the encoder min/max values seem to be dynamic, this function reads when the 
 * encoder moves from max to min or vice-versa and records these values as the new 
 * maximum and minimum values for scaling */
void EncoderMap::calibrateMinMax(vector<short int> new_reading)
{
	if ( abs_encoder_vals_.empty() ) abs_encoder_vals_ = new_reading;

	for(int i = 0; i < N_; i++)
	{
		bool low_to_high = ( abs_encoder_vals_[i] < (0.2 * max_abs_encoder_[i]) ) && ( new_reading[i] > (0.5 * max_abs_encoder_[i]) );
		bool high_to_low = ( abs_encoder_vals_[i] > (0.5 * max_abs_encoder_[i]) ) && ( new_reading[i] < (0.2 * max_abs_encoder_[i]) );

		if (high_to_low){
			max_abs_encoder_[i] = abs_encoder_vals_[i];
			min_abs_encoder_[i] = new_reading[i];

			if (abs_encoder_zero_[i] > max_abs_encoder_[i]) 
				abs_encoder_zero_[i] = min_abs_encoder_[i] + abs_encoder_zero_[i] - max_abs_encoder_[i];
			else if (abs_encoder_zero_[i] < min_abs_encoder_[i])
				abs_encoder_zero_[i] = max_abs_encoder_[i] - min_abs_encoder_[i] + abs_encoder_zero_[i];
		
		}else if (low_to_high){
			max_abs_encoder_[i] = new_reading[i];
			min_abs_encoder_[i] = abs_encoder_vals_[i];

			if (abs_encoder_zero_[i] > max_abs_encoder_[i]) 
				abs_encoder_zero_[i] = min_abs_encoder_[i] + abs_encoder_zero_[i] - max_abs_encoder_[i];	
			else if (abs_encoder_zero_[i] < min_abs_encoder_[i])
				abs_encoder_zero_[i] = max_abs_encoder_[i] - min_abs_encoder_[i] + abs_encoder_zero_[i];	
		}	
	}
}


// TODO: Ask Alex why I need this
// Map a 10-bit input to an absolute angle in radians
vector<double> EncoderMap::mapAbsoluteEncoderRad(vector<short int> data){
	abs_encoder_vals_ = data;
	vector<double> mapped_angles(N_);

	for (int i = 0; i < N_; i++){
		if (data[i] < abs_encoder_zero_[i]) {
			data[i] = max_abs_encoder_[i] + data[i] - min_abs_encoder_[i];
		}

		mapped_angles[i] = 2*M_PI * (data[i] - abs_encoder_zero_[i]) / (max_abs_encoder_[i] - min_abs_encoder_[i]);
	}
	return mapped_angles;
}

// Map a 10-bit input into an absolute angle in degrees
vector<double> EncoderMap::mapAbsoluteEncoderDeg(vector<short int> data)
{
	vector<double> angles = this->mapAbsoluteEncoderRad(data);
	for (double &angle : angles)
		angle *= 180.0/M_PI;

	return angles;
}

// Record a new drive encoder reading and its timestamp
// Trim any readings are from before (now - vel_delay_)
void EncoderMap::appendToHistory(vector<float> data)
{
	double now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time_).count();

	// Append Value and Time Stamp
	drive_history_.push_back(data);
	history_time_stamps_.push_back(now);

	// Count How Many Readings Are Too Old
	short int erase_count = 0;
	for (const double t_stamp : history_time_stamps_)
	{
		if (now - t_stamp > vel_delay_)	erase_count++;
		else break; 
	}

	// Get Rid of All Readings That Are Too Old
	auto it_drive = drive_history_.begin();
	auto it_time  = history_time_stamps_.begin();

	std::advance(it_drive, erase_count);
	std::advance(it_time,  erase_count);

	drive_history_.erase(drive_history_.begin(), it_drive);
	history_time_stamps_.erase(history_time_stamps_.begin(), it_time);
}

// Map a drive encoder reading to a speed in radians per second
vector<double> EncoderMap::mapRelativeEncoderRPS(vector<float> data){
	rel_encoder_vals_ = data;
	vector<double> mapped_velocities(N_);

	appendToHistory(data);

	for (int i = 0; i < N_; i++)
	{
		double d_theta = drive_history_.back()[i] - drive_history_.begin()->at(i);
		double dt 	   = history_time_stamps_.back() - *history_time_stamps_.begin();
		if (dt == 0) continue;

		mapped_velocities[i] = 1000 * d_theta / dt;	// 1000 for converting ms to seconds
	}

	return mapped_velocities;
}
