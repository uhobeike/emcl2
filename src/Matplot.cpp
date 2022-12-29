#include "emcl/Matplot.h"

namespace plt = matplotlibcpp;

namespace emcl2 {
void Matplot::particle_usescan_angle_plot(std::vector<int> particle_num,
                                          std::vector<int> scan_angle) {
  plt::plot(particle_num, scan_angle, "k|");
}

void Matplot::particle_usescan_angle_plot(std::vector<double> particle_num,
                                          std::vector<double> scan_angle) {
  plt::plot(particle_num, scan_angle, "k|");
}

void Matplot::particle_weight_plot(std::vector<double> particle_num,
                                   std::vector<double> particle_weight) {
  plt::plot(particle_num, particle_weight, "k.");
  show_weight();
  clear_weight();
}

void Matplot::show() {
  plt::ylim(0.0, 1.0);
  plt::legend();
  plt::pause(0.1);
}

void Matplot::clear() {
  plt::clf();
}

void Matplot::show_weight() {
  plt::ylim(0.0, 1.0);
  plt::legend();
  plt::pause(0.01);
}

void Matplot::clear_weight() { plt::clf(); }
} // namespace emcl2