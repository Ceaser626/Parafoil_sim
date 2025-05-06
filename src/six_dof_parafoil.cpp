#pragma once

#include <Eigen/Dense>
#include "six_dof_parafoil.hpp"


namespace parafoil
{
	void Parafoil::SetParameters()
	{
		p.rou = 1.225;
	    p.g = 9.8;
	    p.m = 1555;
	    p.J << 1.69e3 + 3.97e3, 0, 0,
    		   0, 8.5e2 + 1.71e3, 0,
    		   0, 0, 2.55e3 + 1.97e3;
	    p.b = 14.3;
	    p.c = 5.8;
	    p.t = 0.87;
	    p.a = 2.16;
	    p.S_p = 100;
	    p.S_s = 20;
	    p.mu = - 12 * M_PI / 180.;
	    p.m_a << 9.39, 0, 0,
	             0, 18.75, 0,
	             0, 0, 336.64;
	    p.J_a << 4.72e3, 0, 0,
	             0, 4.4e2, 0,
	             0, 0, 1.8e2;
	    p.r_B2M << 0.5, 0, -12;
	    p.C_D0 = 0.25;
	    p.C_Dalpha2 = 0.12;
	    p.C_Ddeltas = 0.05;
	    p.C_Ybeta = 0.4;
	    p.C_L0 = 0.091;
	    p.C_Lalpha = 0.9;
	    p.C_Ldeltas = 0.21;
	    p.C_m0 = 0.35;
	    p.C_malpha = - 0.72;
	    p.C_mq = -1.49;
	    p.C_lbeta = - 0.036;
	    p.C_lp = - 0.84;
	    p.C_lr = - 0.082;
	    p.C_ldeltaa = - 0.0035;
	    p.C_nbeta = - 0.0015;
	    p.C_np = - 0.082;
	    p.C_nr = - 0.27;
	    p.C_ndeltaa = 0.0115;
	    p.C_Ds = 0.4;
		p.C_B2P << cos(p.mu), 0, - sin(p.mu),
         		   0, 1, 0,
         		   sin(p.mu), 0, cos(p.mu);
	    p.m_am = p.C_B2P.transpose() * p.m_a * p.C_B2P;
	    p.J_am = p.C_B2P.transpose() * p.J_a * p.C_B2P;
	    p.S_r_B2M << 0, -p.r_B2M(2), p.r_B2M(1),
	                 p.r_B2M(2), 0, -p.r_B2M(0),
	                 -p.r_B2M(1), p.r_B2M(0), 0;
	    p.W << 3, 0, 0;

	    p.x_init << -200, -800, 1200,
        			0, 0, 0,
	                15.65, 0, 8.27,
	                0, 0, 0;
	}

	Parafoil::state_vector_t Parafoil::compute_f(const state_vector_t &x, const control_vector_t &u)
	{
		Eigen::Matrix3d A_11 = p.m * Eigen::Matrix3d::Identity() + p.m_am;
		Eigen::Matrix3d A_12 = - p.m_am * p.S_r_B2M;
		Eigen::Matrix3d A_21 = p.S_r_B2M * p.m_am;
		Eigen::Matrix3d A_22 = p.J + p.J_am - p.S_r_B2M * p.m_am * p.S_r_B2M;
		Eigen::Matrix<double, 6, 6> A;
		A << A_11, A_12, A_21, A_22;

	    Eigen::Matrix<double, 6, 6> A_inv = A.inverse();
		Eigen::Matrix3d A_inv_11 = A_inv.topLeftCorner<3,3>();
		Eigen::Matrix3d A_inv_12 = A_inv.topRightCorner<3,3>();
		Eigen::Matrix3d A_inv_21 = A_inv.bottomLeftCorner<3,3>();
		Eigen::Matrix3d A_inv_22 = A_inv.bottomRightCorner<3,3>();

	    Eigen::Vector3d r = x.segment<3>(0);
		Eigen::Vector3d o = x.segment<3>(3);
		Eigen::Vector3d v = x.segment<3>(6);
		Eigen::Vector3d omiga = x.segment<3>(9);

		double phi = o(0), theta = o(1), psi = o(2);
		double o_x = omiga(0), o_y = omiga(1), o_z = omiga(2);

		Eigen::Matrix3d C_R2B;
		C_R2B << std::cos(psi)*std::cos(theta),  std::sin(psi)*std::cos(theta), -std::sin(theta),
    			 std::cos(psi)*std::sin(theta)*std::sin(phi)-std::sin(psi)*std::cos(phi), std::sin(psi)*std::sin(theta)*std::sin(phi)+std::cos(psi)*std::cos(phi), std::cos(theta)*std::sin(phi),
    			 std::cos(psi)*std::sin(theta)*std::cos(phi)+std::sin(psi)*std::sin(phi), std::sin(psi)*std::sin(theta)*std::cos(phi)-std::cos(psi)*std::sin(phi), std::cos(theta)*std::cos(phi);
		Eigen::Matrix3d C_omiga;
		C_omiga << 1, std::sin(phi)*std::tan(theta), std::cos(phi)*std::tan(theta),
    	  		   0, std::cos(phi), -std::sin(phi),
    			   0, std::sin(phi)/std::cos(theta), std::cos(phi)/std::cos(theta);
		Eigen::Matrix3d S_omiga;
		S_omiga << 0, -o_z, o_y,
	               o_z, 0, -o_x,
	               -o_y, o_x, 0;

	    Eigen::Vector3d v_a = v - C_R2B * p.W;
	    double alpha = std::atan(v_a(2) / v_a(0));
		double beta = std::atan(v_a(1) / std::sqrt(v_a(0)*v_a(0) + v_a(1)*v_a(1)));
		Eigen::Matrix3d C_W2B;
		C_W2B << std::cos(alpha)*std::cos(beta), std::cos(alpha)*std::sin(beta), -std::sin(alpha),
    			 -std::sin(beta), std::cos(beta), 0,
    			 std::sin(alpha)*std::cos(beta), std::sin(alpha)*std::sin(beta), std::cos(alpha);

	    Eigen::Vector3d F_g;
		F_g << -std::sin(theta), std::cos(theta) * std::sin(phi), std::cos(theta) * std::cos(phi);
		F_g = p.m * p.g * F_g;

	    Eigen::Vector3d F_coeffs;
		F_coeffs << p.C_D0 + p.C_Dalpha2 * alpha * alpha + p.C_Ddeltas * u(1),
        			p.C_Ybeta * beta,
    				p.C_L0 + p.C_Lalpha * alpha + p.C_Ldeltas * u(1);
	    Eigen::Vector3d F_a = -0.5 * p.rou * v_a.norm() * v_a.norm() * p.S_p * C_W2B * F_coeffs
							  -0.5 * p.rou * v_a.norm() * p.S_s * p.C_Ds * v_a;
		Eigen::Vector3d M_coeffs;
		M_coeffs << p.b * (p.C_lbeta * beta + p.b / (2 * v_a.norm()) * p.C_lp * o_x + p.b / (2 * v_a.norm()) * p.C_lr * o_z + p.C_ldeltaa * u(0)),
        			p.c * (p.C_m0 + p.C_malpha * alpha + p.c / (2 * v_a.norm()) * p.C_mq * o_y),
        			p.b * (p.C_nbeta * beta + p.b / (2 * v_a.norm()) * p.C_np * o_x + p.b / (2 * v_a.norm()) * p.C_nr * o_z + p.C_ndeltaa * u(0));
	    Eigen::Vector3d M_a = 0.5 * p.rou * v_a.norm() * v_a.norm() * p.S_p * M_coeffs;

		Eigen::Matrix3d B_11 = -S_omiga * (p.m * Eigen::Matrix3d::Identity() + p.m_am);
		Eigen::Matrix3d B_12 = S_omiga * p.m_am * p.S_r_B2M;
		Eigen::Matrix3d B_21 = -p.S_r_B2M * S_omiga * p.m_am;
		Eigen::Matrix3d B_22 = - (S_omiga * (p.J + p.J_am) - p.S_r_B2M * S_omiga * p.m_am * p.S_r_B2M);

		Eigen::Vector3d dr_dt = C_R2B.transpose() * v;
		Eigen::Vector3d do_dt = C_omiga * omiga;
		Eigen::Vector3d dv_dt = A_inv_11 * (F_a + F_g + B_11 * v + B_12 * omiga) + A_inv_12 * (M_a + B_21 * v + B_22 * omiga);
		Eigen::Vector3d domiga_dt = A_inv_21 * (F_a + F_g + B_11 * v + B_12 * omiga) + A_inv_22 * (M_a + B_21 * v + B_22 * omiga);

		state_vector_t dx;
		dr_dt(2) = -dr_dt(2);
		dx.segment<3>(0)  = dr_dt;
		dx.segment<3>(3)  = do_dt;
		dx.segment<3>(6)  = dv_dt;
		dx.segment<3>(9)  = domiga_dt;

		return dx;
	}
}