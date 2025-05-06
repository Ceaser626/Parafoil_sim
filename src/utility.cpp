#include <iostream>
#include <fstream>
#include "six_dof_parafoil.hpp"


void saveToCSV(const std::vector<parafoil::Parafoil::state_vector_t>& X_sim,
               const std::vector<parafoil::Parafoil::control_vector_t>& U_sim,
               const std::vector<double>& t_sim) {
    std::ofstream state_file("simulation_states.csv");
    state_file << "t,x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz\n";
    for (size_t i = 0; i < X_sim.size(); ++i) {
        state_file << t_sim[i] << ",";
        for (int j = 0; j < X_sim[i].size(); ++j) {
            state_file << X_sim[i](j);
            if (j < X_sim[i].size() - 1) state_file << ",";
        }
        state_file << "\n";
    }
    state_file.close();

    std::ofstream control_file("simulation_controls.csv");
    control_file << "t,delta_l,delta_r\n";
    for (size_t i = 0; i < U_sim.size(); ++i) {
        control_file << t_sim[i] << "," << U_sim[i](0) << "," << U_sim[i](1) << "\n";
    }
    control_file.close();
}
