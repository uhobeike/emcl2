//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef PF_H__
#define PF_H__

#include <vector>
#include <sstream>
#include <random>

#include "emcl/Particle.h"
#include "emcl/OdomModel.h"
#include "emcl/LikelihoodFieldMap.h"

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

namespace emcl2 {

class Mcl
{
public: 
	Mcl(){}
	Mcl(const Pose &p, int num, const Scan &scan,
			const std::shared_ptr<OdomModel> &odom_model,
			const std::shared_ptr<LikelihoodFieldMap> &map,
			bool handle_unknown_obstacles, double observation_range);
	~Mcl();

	std::vector<Particle> particles_;
	double alpha_;

	void sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv);
	void motionUpdate(double x, double y, double t);

	void initialize(double x, double y, double t);

	void setScan(const sensor_msgs::LaserScan::ConstPtr &msg);
	void meanPose(double &x_mean, double &y_mean, double &t_mean,
			double &x_var, double &y_var, double &t_var,
			double &xy_cov, double &yt_cov, double &tx_cov);

	void simpleReset(void);

	static double cos_[(1<<16)];
	static double sin_[(1<<16)];

	Scan createObservationRange(Scan scan);

	bool handle_unknown_obstacles_;
	int observation_range_;
protected:
	Pose *last_odom_;
	Pose *prev_odom_;

	Scan scan_;
	int processed_seq_;

	double normalizeAngle(double t);
	void resampling(void);
	double normalizeBelief(void);
	void resetWeight(void);
	void resetObservationRange(Scan scan);

	std::shared_ptr<OdomModel> odom_model_;
	std::shared_ptr<LikelihoodFieldMap> map_;
};

double Mcl::cos_[(1<<16)];
double Mcl::sin_[(1<<16)];

}

#endif
