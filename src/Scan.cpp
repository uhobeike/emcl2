//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/Scan.h"
#include <cmath>

namespace emcl2 {

Scan& Scan::operator=(const Scan &s)
{
	if(this == &s)
		return *this;

	seq_ = s.seq_;
	scan_increment_ = s.scan_increment_;
	angle_max_ = s.angle_max_;
	angle_min_ = s.angle_min_;
	angle_increment_ = s.angle_increment_;
	range_max_ = s.range_max_;
	range_min_ = s.range_min_;

  scan_mask_angle_begin_ = s.scan_mask_angle_begin_;
  scan_mask_angle_middle_ = s.scan_mask_angle_middle_;
  scan_mask_angle_end_ = s.scan_mask_angle_end_;

	// It's not thread safe.
	ranges_.clear();
	copy(s.ranges_.begin(), s.ranges_.end(), back_inserter(ranges_) );

	return *this;
}

Scan& Scan::operator+=(const Scan &s)
{
	if(this == &s)
		return *this;

	seq_ = s.seq_;
	scan_increment_ = s.scan_increment_;
	angle_max_ = s.angle_max_;
	angle_min_ = s.angle_min_;
	angle_increment_ = s.angle_increment_;
	range_max_ = s.range_max_;
	range_min_ = s.range_min_;

	lidar_pose_x_ = s.lidar_pose_x_;
	lidar_pose_y_ = s.lidar_pose_y_;
	lidar_pose_yaw_ = s.lidar_pose_yaw_;

	directions_16bit_ = s.directions_16bit_;

	// It's not thread safe.
	ranges_.clear();
	copy(s.ranges_.begin(), s.ranges_.end(), back_inserter(ranges_) );

	return *this;
}

int Scan::countValidBeams(double *rate)
{
	int ans = 0;
	for(int i=0; i<ranges_.size(); i+=scan_increment_)
		if(valid(ranges_[i]))
			ans++;

	if(rate != NULL)
		*rate = (double)ans/ranges_.size()*scan_increment_;

	return ans;
}

bool Scan::valid(double range)
{
	if( std::isnan(range) or std::isinf(range) )
		return false;

	return range_min_ <= range and range <= range_max_;
}

}
