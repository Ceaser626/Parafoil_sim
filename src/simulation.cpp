#pragma once

#include <Eigen/Dense>
#include "simulation.hpp"
#include "six_dof_parafoil.hpp"


namespace parafoil
{
	bool simulate(Parafoil* model, double dt, const Parafoil::control_vector_t &u, Parafoil::state_vector_t &x)
	{
		for (int i = 0; i < 20; ++i)
  		{
    		double delta_l = u(0), delta_r = u(1);
	        double delta_a = delta_r - delta_l;
	        double delta_s = std::min(delta_l, delta_r);

			Eigen::Vector2d converted_u(delta_a, delta_s);
	        double h = dt / 20;

	        Parafoil::state_vector_t k1 = model->compute_f(x, converted_u);
	        Parafoil::state_vector_t k2 = model->compute_f(x + h * k1 / 2, converted_u);
	        Parafoil::state_vector_t k3 = model->compute_f(x + h * k2 / 2, converted_u);
	        Parafoil::state_vector_t k4 = model->compute_f(x + h * k3, converted_u);

	        x += (h / 6.0) * (k1 + 2*k2 + 2*k3 + k4);

			if(x(2) <= 0)
			{
				return true;
			}
  		}

		return false;
	}
}