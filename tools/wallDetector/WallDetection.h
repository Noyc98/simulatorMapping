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
	double computeMean(const std::vector<double> values);
	double computeStdDeviation(const std::vector<double> values, double mean);
	vector<Eigen::Vector3d> filterNumbersByStdDeviation(vector<Eigen::Vector3d>& points, const std::vector<double> numbers, double std_wall_detector);
	//bool isNormallyDistributed(const std::vector<double>& data) {
	Eigen::Vector4d findMinimizingPlane(const vector<Eigen::Vector3d>& points);
	double angleBetweenPlanes(Eigen::Vector3d first_normal_vector, Eigen::Vector3d second_normal_vector);
	Eigen::Vector3d normalizePoint(Eigen::Vector3d& point);
	bool wallDetector(vector <Eigen::Vector3d>& points, double std_wall_detector, vector <Eigen::Vector3d>& filtered_points);
	double getAverageCord(int index, vector<Eigen::Vector3d>& points);






};


