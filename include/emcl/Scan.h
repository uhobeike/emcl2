//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef SCAN_H__
#define SCAN_H__

#include <iostream>
#include <vector>

namespace emcl2 {

class Scan
{
public:
	int seq_;
	int scan_increment_;
	double angle_max_;
	double angle_min_;
	double angle_increment_;
	double range_max_;
	double range_min_;

	int observation_range_begin_;
	bool observation_range_middle_;
	int observation_range_end_;

	double lidar_pose_x_;
	double lidar_pose_y_;
	double lidar_pose_yaw_;

	std::vector<double> ranges_;
	std::vector<uint16_t> directions_16bit_;

	Scan& operator=(const Scan &s);
	Scan& operator+=(const Scan &s);
	int countValidBeams(double *rate = NULL);
	bool valid(double range);
};

}

#endif
