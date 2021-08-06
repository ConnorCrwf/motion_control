///////////////////////////////////////////////////////////////////////////////
//      Title     : encoder_mapping.h
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


#ifndef ENCODER_MAP_CC
#define ENCODER_MAP_CC

#include <vector>
#include <list>
#include <chrono>
#include <math.h>

#define NUM_MOTORS 2

using std::vector;
using std::list;

class EncoderMap{
public:
	EncoderMap(unsigned int wheel_count);

	void setAbsEncoderMin(vector<short int>);
	void setAbsEncoderMax(vector<short int>);
	void setAbsEncoderZero(vector<short int>);
	void setVelocityDelay(double t);

	void calibrateMinMax(vector<short int>);	

	vector<double> mapAbsoluteEncoderRad(vector<short int>);
	vector<double> mapAbsoluteEncoderDeg(vector<short int>);
	vector<double> mapRelativeEncoderRPS(vector<float>);

private:
	unsigned int N_;

	vector<short int> abs_encoder_vals_;
	vector<float> rel_encoder_vals_;

	vector<double> abs_angles_deg_;
	vector<double> abs_angles_rad_;
	vector<short int> last_abs_angle_;

	vector<short int> max_abs_encoder_;
	vector<short int> min_abs_encoder_;
	vector<short int> abs_encoder_zero_;

	double vel_delay_;	// Delay between drive encoder readings in milliseconds

	list< vector<float> > drive_history_;
	list<double> history_time_stamps_;

	std::chrono::steady_clock::time_point start_time_;

	void appendToHistory(vector<float>);
};

#endif