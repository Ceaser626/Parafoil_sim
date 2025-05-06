#pragma once

#include <Eigen/Dense>


namespace parafoil
{
	template <size_t STATE_DIM, size_t INPUT_DIM>
	class Dynamics
	{
	public:
		using state_vector_t = Eigen::Matrix<double, STATE_DIM, 1>;
		using control_vector_t = Eigen::Matrix<double, INPUT_DIM, 1>;
		using dynamic_vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;

		void initializeModel();

	private:
		bool initialized = false;
	};

	template <size_t STATE_DIM, size_t INPUT_DIM>
	void Dynamics<STATE_DIM, INPUT_DIM>::initializeModel()
	{
		if (initialized)
		{
			return;
		}

		dynamic_vector_t x(STATE_DIM + INPUT_DIM);
		x.setOnes();

		initialized = true;
	}
}
