#ifndef JERK_H
#define JERK_H
#include <vector>

using std::vector;

class Jerk {
private:
    vector<double> coeffs;

public:
    Jerk(vector<double>  start, vector<double>  end, double T);
    double at(double x);
};

#endif 