//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/ExpResetMcl2.h"
#include "emcl/Matplot.h"
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <chrono>
#include <ctime>

namespace emcl2 {

ExpResetMcl2::ExpResetMcl2(const Pose &p, int num, const Scan &scan,
				const std::shared_ptr<OdomModel> &odom_model,
				const std::shared_ptr<LikelihoodFieldMap> &map,
				double alpha_th, 
				double expansion_radius_position, double expansion_radius_orientation,
				double extraction_rate, double range_threshold, bool sensor_reset)
	: alpha_threshold_(alpha_th), 
	  expansion_radius_position_(expansion_radius_position),
	  expansion_radius_orientation_(expansion_radius_orientation),
	  extraction_rate_(extraction_rate),
	  range_threshold_(range_threshold),
	  sensor_reset_(sensor_reset),
	  Mcl::Mcl(p, num, scan, odom_model, map)
{
}

ExpResetMcl2::~ExpResetMcl2()
{
}

void ExpResetMcl2::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv, std::vector<double> &most_observations)
{
	if(processed_seq_ == scan_.seq_)
		return;

	Scan scan;
	int seq = -1;
	while(seq != scan_.seq_){//trying to copy the latest scan before next 
		seq = scan_.seq_;
		scan = scan_;
	}

	scan.lidar_pose_x_ = lidar_x;
	scan.lidar_pose_y_ = lidar_y;
	scan.lidar_pose_yaw_ = lidar_t;

	int i = 0;
	if(!inv){
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_min_ + (i++)*scan.angle_increment_)
			);
	}else{
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_max_ - (i++)*scan.angle_increment_)
			);
	}

	double valid_pct = 0.0;
	int valid_beams = scan.countValidBeams(&valid_pct);
	if(valid_beams == 0)
		return;

	beam_matching_score_sum_ = 0;
	valid_beam_sum_= 0;
	std::vector<std::vector<int>> particle_scan_angles;
	particle_scan_angles.clear();

	int cnt = 0; 
	std::vector<int> scan_angle(360,1);
	partial_sum(scan_angle.begin(), scan_angle.end(), scan_angle.begin());
	std::vector<int> scan_angle_cnt(360,0);
	std::vector<uint8_t> multiple_observation;

    std::chrono::system_clock::time_point start_2, end_2;
    start_2 = std::chrono::high_resolution_clock::now();
	// std::clock_t start = std::clock();
	
	if (particles_.size() != 1000){
		std::cout << "Not particle num" << "\n"; 
		exit(1);
	}

	for(auto &p : particles_){
		double beam_matching_score;
		p.s_ += scan;

		beam_matching_score = p.likelihood(map_.get(), p.s_, valid_beam_sum_, scan_angle_cnt);
		p.w_ *= beam_matching_score;
		beam_matching_score_sum_ += beam_matching_score;
	}

	// std::clock_t end = std::clock();
	end_2 = std::chrono::high_resolution_clock::now();
	double time_2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_2 - start_2).count();
	// double time_2 = static_cast<double>(end - start) / CLOCKS_PER_SEC;

	static std::vector<double> data_2;
	static int cnt_2 = 0;
	cnt_2++ ;
	if (time_2 > 0 && cnt_2 > 100){
		data_2.push_back(time_2);
		double ave_2 = 0.0, var_2 = 0.0;
		for(const auto &x : data_2){
			ave_2 += x;
			var_2 += x * x;
		}
		ave_2 /= data_2.size();
		var_2 = var_2 / data_2.size() - ave_2 * ave_2;
		// std::cout << "重みの計算のみ　平均：" << ave_2 << ", 標準偏差：" << std::sqrt(var_2) << std::endl;
	}

	std::vector<int>::iterator iter = std::max_element(scan_angle_cnt.begin(), scan_angle_cnt.end());
	size_t index = std::distance(scan_angle_cnt.begin(), iter);
	most_observations.push_back(double(index));
	
    std::chrono::system_clock::time_point start_3, end_3;
    start_3 = std::chrono::system_clock::now();

	normalizeBelief();
	alpha_ = beam_matching_score_sum_/valid_beam_sum_;

	if(normalizeBelief() > 0.000001)
		resampling();
	else
		resetWeight();

	
	end_3 = std::chrono::system_clock::now();
	double time_3 = std::chrono::duration_cast<std::chrono::milliseconds>(end_3 - start_3).count();

	static std::vector<double> data_3;
	static int cnt_3 = 0;
	cnt_3++ ;
	if (time_3 > 0 && cnt_3 > 100){
		data_3.push_back(time_3);
		double ave_3 = 0.0, var_3 = 0.0;
		for(const auto &x : data_3){
			ave_3 += x;
			var_3 += x * x;
		}
		ave_3 /= data_3.size();
		var_3 = var_3 / data_3.size() - ave_3 * ave_3;
		// std::cout << "重みの正規化＋リサンプリング＋重みのリセット　平均：" << ave_3 << ", 標準偏差：" << std::sqrt(var_3) << std::endl;
	}

	processed_seq_ = scan_.seq_;


}

bool ExpResetMcl2::getMultipleObservation(std::vector<int> &scan_angle_cnt, std::vector<uint8_t> &multiple_observation){
	int cnt=0;
	for (auto scan_angle : scan_angle_cnt)
	{
		if (scan_angle > 1000)
			multiple_observation.push_back(cnt);
		++cnt;  
	}
	if (multiple_observation.size() > 1){
		return true;
	}
	else
		return false;
}


double ExpResetMcl2::nonPenetrationRate(int skip, LikelihoodFieldMap *map, Scan &scan)
{
	static uint16_t shift = 0;
	int counter = 0;
	int penetrating = 0;
	for(int i=shift%skip; i<particles_.size(); i+=skip){
		counter++;
		if(particles_[i].wallConflict(map, scan, range_threshold_, sensor_reset_))
			penetrating++;
	}
	shift++;

	// std::cout << penetrating << " " << counter << std::endl;
	return (double)(counter - penetrating) / counter;
}

void ExpResetMcl2::expansionReset(void)
{
	for(auto &p : particles_){
		double length = 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_position_;
		double direction = 2*((double)rand()/RAND_MAX - 0.5)*M_PI;

		p.p_.x_ += length*cos(direction);
		p.p_.y_ += length*sin(direction);
		p.p_.t_ += 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_orientation_;
		p.w_ = 1.0/particles_.size();
	}
}

std::vector<std::vector<int>> ExpResetMcl2::getParticleScanAngles(void)
{
	return particle_scan_angles_;
}

}
