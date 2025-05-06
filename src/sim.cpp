#include <iostream>
#include "six_dof_parafoil.hpp"
#include "simulation.hpp"
#include "utility.hpp"


int main()
{
	using namespace parafoil;
	Parafoil model;

    double t = 0;
    double timestep = 1;
    bool stop_sign = false;

	model.SetParameters();
    Parafoil::state_vector_t x = model.p.x_init;

	std::vector<Parafoil::state_vector_t> X_sim;
	std::vector<Parafoil::control_vector_t> U_sim;
	std::vector<double> t_sim;

	// start simulation
    while (!stop_sign)
    {
    	Parafoil::control_vector_t u(0, 0.3); // delta_l, delta_r

        stop_sign = simulate(&model, timestep, u, x);
        t += timestep;

        X_sim.push_back(x);
        U_sim.push_back(u);
        t_sim.push_back(t);

        std::cout << "Distance to landing point:" << std::sqrt(x(0) * x(0) + x(1) * x(1))
                  << " Altitude:" << std::abs(x(2)) << std::endl;
    }

	std::cout << "Touchdown";
	saveToCSV(X_sim, U_sim, t_sim);

	return 0;
}