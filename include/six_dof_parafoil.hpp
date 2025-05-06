#pragma once

#include <Eigen/Dense>
#include "vehicle_dynamics.hpp"


namespace parafoil
{
	enum dimensions
	{
		STATE_DIM = 12,
		CONTROL_DIM = 2
	};

	class Parafoil: public Dynamics<STATE_DIM, CONTROL_DIM>
	{
		public:
    		inline static const std::string modelName = "Parafoil";

    		void SetParameters();

    		state_vector_t compute_f(const state_vector_t &x, const control_vector_t &u);

    		struct Parameters
    		{
        		double rou;
        		double g;
        		double m;
        		Eigen::Matrix3d J;
        		double b;
        		double c;
        		double t;
        		double a;
        		double S_p;
        		double S_s;
        		double mu;
        		Eigen::Matrix3d m_a;
        		Eigen::Matrix3d J_a;
        		Eigen::Vector3d r_B2M;
        		double C_D0;
    			double C_Dalpha2;
    			double C_Ddeltas;
    			double C_Ybeta;
    			double C_L0;
    			double C_Lalpha;
    			double C_Ldeltas;
    			double C_m0;
    			double C_malpha;
    			double C_mq;
    			double C_lbeta;
    			double C_lp;
    			double C_lr;
    			double C_ldeltaa;
    			double C_nbeta;
    			double C_np;
    			double C_nr;
    			double C_ndeltaa;
    			double C_Ds;
				Eigen::Matrix3d C_B2P;
        		Eigen::Matrix3d m_am;
        		Eigen::Matrix3d J_am;
        		Eigen::Matrix3d S_r_B2M;
	            Eigen::Vector3d W;
	            state_vector_t x_init;
    		} p;
	};
}
