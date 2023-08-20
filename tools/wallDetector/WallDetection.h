#pragma once
#include "IncludeLibraries.h"

class wallHandle {
public:
	bool isNormallyDistributed(const std::vector<double>& data);

	Eigen::Vector4d findMinimizingPlane(const vector<Eigen::Vector3d>& points);

	double angleBetweenPlanes(Eigen::Vector3d normalVector);

	bool wallDetector(vector <Eigen::Vector3d>& points);

};


