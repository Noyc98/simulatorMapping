#pragma once

#include <iostream>
#include <vector>
#include <random>
#include <ctime>
#include <cstdlib>
#include <math.h>
#include <iomanip>
#include <Eigen/Dense>
#include <gsl/statistics/gsl_statistics.h>
#include <gsl/cdf/gsl_cdf.h>

# define M_PI	3.14159265358979323846  /* pi */ 

using namespace std;


class wallHandle {
public:
	bool isNormallyDistributed(const std::vector<double>& data);

	Eigen::Vector4d findMinimizingPlane(const vector<Eigen::Vector3d>& points);

	double angleBetweenPlanes(Eigen::Vector3d normalVector);

	bool wallDetector(vector <Eigen::Vector3d>& points);

};


