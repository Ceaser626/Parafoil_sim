#pragma once

#include "six_dof_parafoil.hpp"


namespace parafoil
{
    bool simulate(Parafoil* model, double dt, const Parafoil::control_vector_t &u, Parafoil::state_vector_t &x);
}
