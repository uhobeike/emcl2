//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 
#include "emcl/Mcl.h"
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <random>
#include <fstream>
#include <chrono>

namespace emcl2 {

Mcl::Mcl(const Pose &p, int num, const Scan &scan,
		const std::shared_ptr<OdomModel> &odom_model,
		const std::shared_ptr<LikelihoodFieldMap> &map)
	: last_odom_(NULL), prev_odom_(NULL)
{
	odom_model_ = move(odom_model);
	map_ = move(map);
	scan_ = scan;

	if(num <= 0)
		ROS_ERROR("NO PARTICLE");

	Particle particle(p.x_, p.y_, p.t_, 1.0/num);
	for(int i=0; i<num; i++)
		particles_.push_back(particle);

	processed_seq_ = -1;
	alpha_ = 1.0;

	beam_matching_score_sum_= 0;
	valid_beam_sum_ = 0;

	for(int i=0;i<(1<<16);i++){
		cos_[i] = cos(M_PI*i/(1<<15));
		sin_[i] = sin(M_PI*i/(1<<15));
	}
}

Mcl::~Mcl()
{
	delete last_odom_;
	delete prev_odom_;
}

void Mcl::resampling(void)
{
	std::chrono::system_clock::time_point start_2, end_2;
    start_2 = std::chrono::system_clock::now();
	
	std::vector<double> accum;
	accum.push_back(particles_[0].w_);
	for(int i=1;i<particles_.size();i++){
		accum.push_back(accum.back() + particles_[i].w_);
	}

	std::vector<Particle> old(particles_);

	double start = (double)rand()/(RAND_MAX * particles_.size());
	double step = 1.0/particles_.size();

	std::vector<int> chosen;

	int tick = 0;
	for(int i=0; i<particles_.size(); i++){
		while(accum[tick] <= start + i*step){
			tick++;
			if(tick == particles_.size()){
				ROS_ERROR("RESAMPLING FAILED");
				exit(1);
			}	
		}	
		chosen.push_back(tick);
	}

	for(int i=0; i<particles_.size(); i++){
		particles_[i] = old[chosen[i]];
		particles_[i].s_ = old[chosen[i]].s_;
	}

	constexpr int MIN = 1;
	constexpr int MAX = 3340;
	constexpr int RAND_NUMS_TO_GENERATE = 100;
	
	std::random_device rd;
	std::mt19937 eng(rd());
	std::uniform_int_distribution<int> distr(MIN, MAX);
	std::vector<int> result;

	for (int i=0;i<RAND_NUMS_TO_GENERATE;i++){
		result.push_back(distr(eng));
	}

	int random_scan_cnt = 0;
	for(auto &p : particles_){
		random_scan_cnt++;
		if (result.end() != std::find(result.begin(), result.end(), random_scan_cnt)){
			p.s_ = randomScanRange(p.s_);
		}
	}

	end_2 = std::chrono::system_clock::now();
	double time_2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_2 - start_2).count();
}

void Mcl::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv)
{
	if(processed_seq_ == scan_.seq_)
		return;

    std::chrono::system_clock::time_point start_2, end_2;
    start_2 = std::chrono::system_clock::now();
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
	if (!inv) {
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_min_ + (i++)*scan.angle_increment_)
			);
	} else {
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_max_ - (i++)*scan.angle_increment_)
			);
	}

	double valid_pct = 0.0;
	int valid_beams = scan.countValidBeams(&valid_pct);
	if(valid_beams == 0)
		return;

	for(auto &p : particles_)
		p.w_ *= p.likelihood(map_.get(), scan);

	/*
	alpha_ = normalizeBelief()/valid_beams;
	if(alpha_ < alpha_threshold_ and valid_pct > open_space_threshold_){
		ROS_INFO("RESET");
		expansionReset();
		for(auto &p : particles_)
			p.w_ *= p.likelihood(map_.get(), scan);
	}
	*/

	if(normalizeBelief() > 0.000001)
		resampling();
	else
		resetWeight();

	processed_seq_ = scan_.seq_;

	end_2 = std::chrono::system_clock::now();
	double time_2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_2 - start_2).count();

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
		// std::cout << "Mcl::sensorUpdate　平均：" << ave_2 << ", 標準偏差：" << std::sqrt(var_2) << std::endl;
	}
}

void Mcl::motionUpdate(double x, double y, double t)
{
	if(last_odom_ == NULL){
		last_odom_ = new Pose(x, y, t);
		prev_odom_ = new Pose(x, y, t);
		return;
	}else
		last_odom_->set(x, y, t);

	Pose d = *last_odom_ - *prev_odom_;
	if(d.nearlyZero())
		return;

	int scan_hani = 360;

    std::chrono::system_clock::time_point start, end;
    start = std::chrono::high_resolution_clock::now();

	double fw_length = sqrt(d.x_*d.x_ + d.y_*d.y_);
	double fw_direction = atan2(d.y_, d.x_) - prev_odom_->t_;

	odom_model_->setDev(fw_length, d.t_);

	for(auto &p : particles_)
		p.p_.move(fw_length, fw_direction, d.t_,
			odom_model_->drawFwNoise(), odom_model_->drawRotNoise());

	prev_odom_->set(*last_odom_);

	end = std::chrono::high_resolution_clock::now();
	double time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

	time/=1000000;
	std::string str = "/tmp/time_idou_" + std::to_string(scan_hani) + ".csv";
	std::ofstream ofs_csv_file(str, std::ios::app);
	ofs_csv_file << time << ',';
	ofs_csv_file << std::endl;
}

void Mcl::meanPose(double &x_mean, double &y_mean, double &t_mean,
				double &x_dev, double &y_dev, double &t_dev,
				double &xy_cov, double &yt_cov, double &tx_cov)
{
	double x, y, t, t2;
	x = y = t = t2 = 0.0;
	for(const auto &p : particles_){
		x += p.p_.x_;
		y += p.p_.y_;
		t += p.p_.t_;
		t2 += normalizeAngle(p.p_.t_ + M_PI);
	}

	x_mean = x / particles_.size();
	y_mean = y / particles_.size();
	t_mean = t / particles_.size();
	double t2_mean = t2 / particles_.size();

	double xx, yy, tt, tt2;
	xx = yy = tt = tt2 = 0.0;
	for(const auto &p : particles_){
		xx += pow(p.p_.x_ - x_mean, 2);
		yy += pow(p.p_.y_ - y_mean, 2);
		tt += pow(p.p_.t_ - t_mean, 2);
		tt2 += pow(normalizeAngle(p.p_.t_ + M_PI) - t2_mean, 2);
	}

	if(tt > tt2){
		tt = tt2;
		t_mean = normalizeAngle(t2_mean - M_PI);
	}

	x_dev = xx/(particles_.size() - 1);
	y_dev = yy/(particles_.size() - 1);
	t_dev = tt/(particles_.size() - 1);

	double xy, yt, tx;
	xy = yt = tx = 0.0;
	for(const auto &p : particles_){
		xy += (p.p_.x_ - x_mean)*(p.p_.y_ - y_mean);
		yt += (p.p_.y_ - y_mean)*(normalizeAngle(p.p_.t_ - t_mean));
		tx += (p.p_.x_ - x_mean)*(normalizeAngle(p.p_.t_ - t_mean));
	}

	xy_cov = xy/(particles_.size() - 1);
	yt_cov = yt/(particles_.size() - 1);
	tx_cov = tx/(particles_.size() - 1);
}

double Mcl::normalizeAngle(double t)
{
	while(t > M_PI)
		t -= 2*M_PI;
	while(t < -M_PI)
		t += 2*M_PI;

	return t;
}

void Mcl::setScan(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	if(msg->ranges.size() != scan_.ranges_.size())
		scan_.ranges_.resize(msg->ranges.size());

	scan_.seq_ = msg->header.seq;
	for(int i=0; i<msg->ranges.size(); i++)
		scan_.ranges_[i] = msg->ranges[i];

	scan_.angle_min_ = msg->angle_min;
	scan_.angle_max_ = msg->angle_max;
	scan_.angle_increment_ = msg->angle_increment;
	scan_.range_min_= msg->range_min;
	scan_.range_max_= msg->range_max;
}

double Mcl::normalizeBelief(void)
{
	double sum = 0.0;
	for(const auto &p : particles_)
		sum += p.w_;

	if(sum < 0.000000000001)
		return sum;

	for(auto &p : particles_)
		p.w_ /= sum;

	return sum;
}

void Mcl::resetWeight(void)
{
	for(auto &p : particles_)
		p.w_ = 1.0/particles_.size();
}

void Mcl::initialize(double x, double y, double t)
{	
	Pose new_pose(x, y, t);
	Scan scan = scan_;
	for(auto &p : particles_){
		p.p_ = new_pose;
	p.s_ = randomScanRange(scan);
	}

	resetWeight();

	resetWeight();
}

void Mcl::initialize(double x, double y, double t, int cnt)
{	
	Scan scan = scan_;
	int p_cnt = 0;

	if (cnt == 1){
		Pose new_pose_1(x+0.5, y, t);
		for(auto &p : particles_){
			if(p_cnt < particles_.size()/2){
				p.p_ = new_pose_1;
				p.s_ = randomScanRange(scan);
			}
			p_cnt++;
		}
	}

	p_cnt=0;

	if (cnt == 2){
		Pose new_pose_2(x-0.5, y, t);
		for(auto &p : particles_){
			if(p_cnt >= particles_.size()/2 && p_cnt < particles_.size()){
				p.p_ = new_pose_2;
				p.s_ = randomScanRange(scan);
			}
			p_cnt++;
		}
	}

	resetWeight();
}

Scan Mcl::randomScanRange(Scan scan)
{
  std::random_device seed_gen;
  std::default_random_engine engine(seed_gen());

  std::uniform_int_distribution<> dist_angle(0, scan.ranges_.size());

  scan.scan_mask_angle_begin_ = dist_angle(engine);

  // 0~40 angle
  // std::uniform_int_distribution<> dist_angle_size(0, scan.ranges_.size()/9);

  // scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + dist_angle_size(engine);

//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 0;   //360
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 10;   //350
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 20;   //340
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 30;   //330
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 40;   //320
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 50;   //310
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 60;   //300
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 70;   //290
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 80;   //280
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 90;   //270
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 100;   //260
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 110;   //250
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 120;   //240
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 130;   //230
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 140;   //220
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 150;   //210
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 160;   //200
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 170;   //190
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 180;   //180
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 190;   //170
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 200;   //160
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 210;   //150
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 220;   //140
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 230;   //130
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 240;   //120
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 250;   //110
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 260;   //100
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 270;   //90
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 280;   //80
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 290;   //70
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 300;   //60
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 310;   //50
  scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 320;   //40
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 330;   //30
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 340;   //20
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 345; //15
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 350;   //10
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 355;   //5
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 359;   //1
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 360;   //0

//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 0;   //360
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 40;   //320
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 80;   //280
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 120;   //240
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 160;   //200
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 200;   //160
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 240;   //120
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 280;   //80
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 320;   //40
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 360;   //0

//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 89;  //270
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 179; //180
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 269; //90
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 314; //45
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 339; //20
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 344; //15
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 349; //10
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 354; //5
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 356; //3
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 357; //2
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 358; //1
//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 359; //0
//   std::uniform_int_distribution<> ramdom_angle(0, 20);

//   scan.scan_mask_angle_end_ = scan.scan_mask_angle_begin_ + 338 + ramdom_angle(engine); //1~10


  scan.scan_mask_angle_middle_ = false;
  if (scan.scan_mask_angle_end_ >= scan.ranges_.size()){
    scan.scan_mask_angle_middle_ = true;
    scan.scan_mask_angle_end_ = scan.scan_mask_angle_end_ - static_cast<int>(scan.ranges_.size()); 
  }

  return scan;
}

void Mcl::simpleReset(void)
{
	std::vector<Pose> poses;
	map_->drawFreePoses(particles_.size(), poses);

	for(int i=0; i<poses.size(); i++){
		particles_[i].p_ = poses[i];
		particles_[i].w_ = 1.0/particles_.size();
	}
}

}
