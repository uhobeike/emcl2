// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef MATPLOT_H__
#define MATPLOT_H__

#include <matplotlib_cpp/matplotlibcpp.h>

namespace plt = matplotlibcpp;

namespace emcl2 {

class Matplot {
public:
  Matplot() {}

  void particle_usescan_angle_plot(std::vector<int> particle_num,
                                   std::vector<int> scan_angle);
  void particle_usescan_angle_plot(std::vector<double> particle_num,
                                   std::vector<double> scan_angle);
  void particle_weight_plot(std::vector<double> particle_num,
                            std::vector<double> particle_weight);

  void show();
  void clear();
  void show_weight();
  void clear_weight();
};

} // namespace emcl2

#endif
