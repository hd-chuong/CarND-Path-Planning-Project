#include "jerk.h"
#include "Eigen-3.3/Eigen/Dense"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Jerk::Jerk(vector<double> start, vector<double> end, double T)
{
/**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
    double s_init = start[0];
    double v_init = start[1];
    double a_init = start[2];

    double T2 = pow(T, 2.0);
    double T3 = pow(T, 3.0);
    double T4 = pow(T, 4.0);
    double T5 = pow(T, 5.0);

    MatrixXd time_mat = MatrixXd(3,3);
    time_mat << T3, T4, T5,
            3 * T2, 4 * T3, 5 * T4,
            6 * T, 12 * T2, 20 * T3;

    double s_final = end[0];
    double v_final = end[1];
    double a_final = end[2];

    MatrixXd right_mat = MatrixXd(3,1);
    right_mat << s_final - (s_init + v_init * T + 0.5 * a_init * T2),
                v_final - (v_init + a_init * T),
                a_final - a_init;
    
    MatrixXd res_mat = time_mat.inverse() * right_mat;
    coeffs = {
        s_init,
        v_init,
        a_init / 2,
        res_mat(0),
        res_mat(1),
        res_mat(2)
        };
}


double Jerk::at(double x)
{
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); ++i)
        {
                result+= pow(x, i) * coeffs[i];
        }
        return result;
}