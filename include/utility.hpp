#include "six_dof_parafoil.hpp"


void saveToCSV(const std::vector<parafoil::Parafoil::state_vector_t>& X_sim,
               const std::vector<parafoil::Parafoil::control_vector_t>& U_sim,
               const std::vector<double>& t_sim);
