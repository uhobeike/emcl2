#include "emcl/Matplot.h"

namespace plt = matplotlibcpp;

namespace emcl2 {
    void Matplot::particle_usescan_angle_plot(std::vector<int> particle_num, std::vector<int> scan_angle)
    {
        plt::plot(particle_num, scan_angle, "k|");
    }
    
    void Matplot::particle_usescan_angle_plot(std::vector<double> particle_num, std::vector<double> scan_angle)
    {
        plt::plot(particle_num, scan_angle, "k|");
    }

    void Matplot::show()
    {
        plt::ylim(0.0, 1.0);
        plt::legend();
        // plt::show();
        plt::pause(0.01);
    }

    void Matplot::clear()
    {
        plt::clf();
        // plt::cla();
    }
}