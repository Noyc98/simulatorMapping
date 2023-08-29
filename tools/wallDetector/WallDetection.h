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
	Eigen::Vector3d normalizePoint(Eigen::Vector3d& point);

	double computeMean(const std::vector<double> value);

	double computeStdDeviation(const std::vector<double> values, double mean);

	//std::vector<double> filterNumbersByStdDeviation(const std::vector<double> numbers);
	vector<Eigen::Vector3d> filterNumbersByStdDeviation(vector<Eigen::Vector3d>& points, const std::vector<double> numbers, double stdWallDetector);

	bool isNormallyDistributed(const std::vector<double>& data);

	Eigen::Vector4d findMinimizingPlane(const vector<Eigen::Vector3d>& points);

	double angleBetweenPlanes(Eigen::Vector3d firstNormalVector, Eigen::Vector3d SecNormalVector);

	bool wallDetector(vector <Eigen::Vector3d>& points, double stdWallDetector, vector <Eigen::Vector3d>& normalize_points);

	double getAverageCord(int index, vector <Eigen::Vector3d>& points);

};


