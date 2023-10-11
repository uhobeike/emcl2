//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/Particle.h"
#include "emcl/Mcl.h"
#include "emcl/Matplot.h"
#include <cmath>
#include <ros/ros.h>
#include <chrono>

#include<fstream>

namespace emcl2 {



Particle::Particle(double x, double y, double t, double w) : p_(x, y, t)
{
	w_ = w;
}

double Particle::likelihood(LikelihoodFieldMap *map, Scan &scan)
{
	uint16_t t = p_.get16bitRepresentation();
	double lidar_x = p_.x_ + scan.lidar_pose_x_*Mcl::cos_[t] 
				- scan.lidar_pose_y_*Mcl::sin_[t];
	double lidar_y = p_.y_ + scan.lidar_pose_x_*Mcl::sin_[t] 
				+ scan.lidar_pose_y_*Mcl::cos_[t];
	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

	double ans = 0.0;
	for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){
		if(not scan.valid(scan.ranges_[i]))
			continue;
		if(scan.scan_mask_angle_middle_)
			if ((scan.scan_mask_angle_begin_ <= i && i <= scan.ranges_.size()) 
					&& (0 >= i && i <= scan.scan_mask_angle_end_))
				continue;
		else
			if (scan.scan_mask_angle_begin_ <= i && i <= scan.scan_mask_angle_end_)
				continue;
		
		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
		double lx = lidar_x + scan.ranges_[i] * Mcl::cos_[a];
		double ly = lidar_y + scan.ranges_[i] * Mcl::sin_[a];

		ans += map->likelihood(lx, ly);
	}

	return ans;
}

double Particle::likelihood(LikelihoodFieldMap *map, Scan &scan, int &valid_beam_sum)
{
	uint16_t t = p_.get16bitRepresentation();
	double lidar_x = p_.x_ + scan.lidar_pose_x_*Mcl::cos_[t] 
				- scan.lidar_pose_y_*Mcl::sin_[t];
	double lidar_y = p_.y_ + scan.lidar_pose_x_*Mcl::sin_[t] 
				+ scan.lidar_pose_y_*Mcl::cos_[t];
	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

	double test_cnt = 0;

	double ans = 0.0;
	for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){
		if(not scan.valid(scan.ranges_[i]))
			continue;
		if(scan.scan_mask_angle_middle_)
			if ((scan.scan_mask_angle_begin_ <= i && i <= scan.ranges_.size()) 
					|| (0 <= i && i <= scan.scan_mask_angle_end_)){
				continue;
			}
		if(not scan.scan_mask_angle_middle_){
			if ((scan.scan_mask_angle_begin_ <= i) && (i <= scan.scan_mask_angle_end_)){
				continue;
			}
		}
		
		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
		double lx = lidar_x + scan.ranges_[i] * Mcl::cos_[a];
		double ly = lidar_y + scan.ranges_[i] * Mcl::sin_[a];

		ans += map->likelihood(lx, ly);
		++valid_beam_sum;
		++test_cnt;
	}

	std::cout << test_cnt << ", ";
	return ans;
}

double Particle::likelihood(LikelihoodFieldMap *map, Scan &scan, int &valid_beam_sum, std::vector<int> &scan_angle)
{
	uint16_t t = p_.get16bitRepresentation();
	double lidar_x = p_.x_ + scan.lidar_pose_x_*Mcl::cos_[t] 
				- scan.lidar_pose_y_*Mcl::sin_[t];
	double lidar_y = p_.y_ + scan.lidar_pose_x_*Mcl::sin_[t] 
				+ scan.lidar_pose_y_*Mcl::cos_[t];
	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);
	
	double ans = 0.0;
	static int p_cnt = 0; //パーテイクル分、実行されているかのカウントアップ
	int exec_cnt = 0; //実際にforが観測範囲分、実行されているかのカウントアップ
	static double time_sum = 0; //全パーテイクルでの処理時間
	int scan_hani = 10; //観測範囲

    std::chrono::system_clock::time_point start, end;

	//計測開始
    start = std::chrono::high_resolution_clock::now();

	for(int i=0;i<scan_hani;i+=1){	
		ans += map->likelihood(0, 0); //適当に0を固定で入れておく
		++exec_cnt;//実行のカウントアップ
	}

	end = std::chrono::high_resolution_clock::now();
	//計測終了

	double time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

	//実行回数と観測範囲があっているか、合っていなかったら異常終了
	if (exec_cnt != scan_hani){
		std::cout << "debug exec_cnt: " << exec_cnt << ", " 
										<< scan.scan_mask_angle_middle_ << ", "
										<< scan.scan_mask_angle_begin_ << ", "
										<< scan.scan_mask_angle_end_ << "\n";
		exit(1);
	}

	//全てのパーテイクル分処理が終わった時にcsvに処理時間を書き込む
	time_sum += time;
	p_cnt++;
	if (p_cnt >= 50000){
		time_sum/=1000000;
		std::ofstream ofs_csv_file("/tmp/time_50000_10_1.csv", std::ios::app);
		ofs_csv_file << time_sum << ',';
		ofs_csv_file << std::endl;
		time_sum = 0;
		p_cnt = 0;
	}
	return ans;
}

double Particle::likelihood2(LikelihoodFieldMap *map, Scan &scan, int &valid_beam_sum, std::vector<uint8_t> &scan_angle)
{
	uint16_t t = p_.get16bitRepresentation();
	double lidar_x = p_.x_ + scan.lidar_pose_x_*Mcl::cos_[t] 
				- scan.lidar_pose_y_*Mcl::sin_[t];
	double lidar_y = p_.y_ + scan.lidar_pose_x_*Mcl::sin_[t] 
				+ scan.lidar_pose_y_*Mcl::cos_[t];
	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

	double ans = 0.0;
	for(auto i : scan_angle){
		if(not scan.valid(scan.ranges_[i]))
			continue;
		if(scan.scan_mask_angle_middle_)
			if ((scan.scan_mask_angle_begin_ <= i && i <= scan.ranges_.size()) 
					|| (0 <= i && i <= scan.scan_mask_angle_end_)){
				continue;
			}
		if(not scan.scan_mask_angle_middle_){
			if ((scan.scan_mask_angle_begin_ <= i) && (i <= scan.scan_mask_angle_end_)){
				continue;
			}
		}
		
		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
		double lx = lidar_x + scan.ranges_[i] * Mcl::cos_[a];
		double ly = lidar_y + scan.ranges_[i] * Mcl::sin_[a];

		ans += map->likelihood(lx, ly);
		++valid_beam_sum;
	}
	return ans;
}

bool Particle::wallConflict(LikelihoodFieldMap *map, Scan &scan, double threshold, bool replace)
{
	uint16_t t = p_.get16bitRepresentation();
	double lidar_x = p_.x_ + scan.lidar_pose_x_*Mcl::cos_[t]
				- scan.lidar_pose_y_*Mcl::sin_[t];
	double lidar_y = p_.y_ + scan.lidar_pose_x_*Mcl::sin_[t]
				+ scan.lidar_pose_y_*Mcl::cos_[t];
	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

	std::vector<int> order;
	if(rand()%2){
		for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_)
			order.push_back(i);
	}else{
		for(int i=scan.ranges_.size()-1;i>=0;i-=scan.scan_increment_)
			order.push_back(i);
	}

	int hit_counter = 0;
	for(int i : order){
		if(not scan.valid(scan.ranges_[i]))
			continue;
	
		double range = scan.ranges_[i];
		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
	
		double hit_lx, hit_ly;
		double hit_lx1, hit_ly1, r1;
		uint16_t a1;
		if(isPenetrating(lidar_x, lidar_y, range, a, map, hit_lx, hit_ly)){
			if(hit_counter == 0){
				hit_lx1 = hit_lx;
				hit_ly1 = hit_ly;
				r1 = range;
				a1 = a;
			}

			hit_counter++;
		}else
			hit_counter = 0;

		if(hit_counter*scan.angle_increment_ >= threshold){
			if(replace)
				sensorReset(lidar_x, lidar_y,
						r1, a1, hit_lx1, hit_ly1,
						range, a, hit_lx, hit_ly);
			return true;
		}
	}
	return false;
}

bool Particle::isPenetrating(double ox, double oy, double range, uint16_t direction,
		LikelihoodFieldMap *map, double &hit_lx, double &hit_ly)
{
	bool hit = false;
	for(double d=map->resolution_;d<range;d+=map->resolution_){
		double lx = ox + d * Mcl::cos_[direction];
		double ly = oy + d * Mcl::sin_[direction];

		if((not hit) and map->likelihood(lx, ly) > 0.99){
			hit = true;
			hit_lx = lx;
			hit_ly = ly;
		}
		else if(hit and map->likelihood(lx, ly) == 0.0){ // openspace after hit
			return true; // penetration
		}
	}
	return false;
}

void Particle::sensorReset(double ox, double oy,
		double range1, uint16_t direction1, double hit_lx1, double hit_ly1,
		double range2, uint16_t direction2, double hit_lx2, double hit_ly2)
{
	double p1_x = ox + range1 * Mcl::cos_[direction1];
	double p1_y = oy + range1 * Mcl::sin_[direction1];
	double p2_x = ox + range2 * Mcl::cos_[direction2];
	double p2_y = oy + range2 * Mcl::sin_[direction2];

	double cx = (hit_lx1 + hit_lx2)/2;
	double cy = (hit_ly1 + hit_ly2)/2;

	p_.x_ -= (p1_x + p2_x)/2 - cx;
	p_.y_ -= (p1_y + p2_y)/2 - cy;

	double theta_delta = atan2(p2_y - p1_y, p2_x - p1_x) - atan2(hit_ly2 - hit_ly1, hit_lx2 - hit_lx1);
	/*
	double d = std::sqrt((p_.x_ - cx)*(p_.x_ - cx) + (p_.y_ - cy)*(p_.y_ - cy));

	double theta = atan2(p_.y_ - cy, p_.x_ - cx) - theta_delta;
	p_.x_ = cx + d * std::cos(theta);
	p_.y_ = cy + d * std::cos(theta);
*/
	p_.t_ -= theta_delta;
}

Particle Particle::operator =(const Particle &p)
{
	p_ = p.p_;
	w_ = p.w_;
	return *this;
}

}
