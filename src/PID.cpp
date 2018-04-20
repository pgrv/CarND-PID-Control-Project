#include "PID.h"
#include <iostream>
#include <cmath>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
	PID::Kp = Kp;
    PID::Ki = Ki;
	PID::Kd = Kd;
	//total_error = 0;
	//dp.resize(3);
	//dp = {0.1*Kp, 0.1*Ki, 0.1*Kd};
	//iter = 0;
}

void PID::UpdateError(double cte)
{
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;

	// Twiddle
/*
	std::vector<double> p(3);

	
	p = {Kp, Ki, Kd};

	if(iter == 1)
	{
		best_error = abs(-p[0] * p_error - p[2] * d_error - p[1] * i_error);
	}

	//double error_sum = abs(dp[0]) + abs(dp[1]) + abs(dp[2]);
	//while ((dp[0] +dp[1] + dp[2]) > 0.2)
	//{
	if(best_error > 0.002)
	{
	for(int i = 0; i < p.size(); i++)
	{
		//std::cout << "blub0" << std::endl;
		p[i] += dp[i];
		total_error = abs(-p[0] * p_error - p[2] * d_error - p[1] * i_error);
		if (total_error < best_error)
		{
			//std::cout << "blub1" << std::endl;
			best_error = total_error;
			dp[i] *= 1.1;
		}
		else
		{
			p[i] -= 2 * dp[i];
			total_error = abs(-p[0] * p_error - p[2] * d_error - p[1] * i_error);
			if (total_error < best_error)
			{
				//std::cout << "blub2" << std::endl;
				best_error = total_error;
				dp[i] *= 1.1;
			}
			else
			{
				//std::cout << "blub3" << std::endl;
				//std::cout << "p[0]: " << p[0] << " p[1]: " << p[1] << " p[2] " << p[2] << std::endl;
				//std::cout << "dp[0]: " << dp[0] << " dp[1]: " << dp[1] << " dp[2] " << dp[2] << std::endl;
				p[i] += dp[i];
				dp[i] *= 0.9;
				//std::cout << "p[0]: " << p[0] << " p[1]: " << p[1] << " p[2] " << p[2] << std::endl;
				//std::cout << "dp[0]: " << dp[0] << " dp[1]: " << dp[1] << " dp[2] " << dp[2] << std::endl;
			}
		}
	//}
	std::cout << "p[0]: " << p[0] << " p[1]: " << p[1] << " p[2] " << p[2] << std::endl;
	std::cout << "dp[0]: " << dp[0] << " dp[1]: " << dp[1] << " dp[2] " << dp[2] << std::endl;
	std::cout << "best_error: " << best_error << " total_error: " << total_error << std::endl;
	}
	Kp = p[0];
	Ki = 0; //p[1];
	Kd = p[2];
	dp[0] = Kp * 0.1;
	dp[1] = Ki * 0.1;
	dp[2] = Kd * 0.1;
	}
	iter++;
	std::cout << " Kp: " << Kp << " Ki: " << Ki << " Kd " << Kd << std::endl;
*/
}

double PID::TotalError()
{
	double steer = -Kp * p_error - Kd * d_error - Ki * i_error;
	return steer;
}

