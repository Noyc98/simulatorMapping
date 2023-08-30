#include "WallDetection.h"

/* 
    Compute the mean of a vector
 Input:
       const std::vector<double>    value

 Output:
        double                      sum / double(value.size())
*/
double wallHandle::computeMean(const std::vector<double> values)
{
    double sum = 0;
    for (auto val : values) 
    { 
        sum += val;
    }
    return sum / double(values.size());
}

/*
    Compute the Std Deviation of a vector
 Input:
       const std::vector<double>    values
       double                       mean

 Output:
        double                      sqrt(variance)
*/
double wallHandle::computeStdDeviation(const std::vector<double> values, double mean)
{
    double variance_sum = 0;

    for (auto value : values) 
    { 
        variance_sum += (value - mean) * (value - mean); 
    }

    double variance = variance_sum / double(values.size());

    return sqrt(variance);
}

/*
    3D vector of points filltered by std_wall_detector to ignore outliers
 Input:
       vector<Eigen::Vector3d>&     points
       const std::vector<double>    numbers
       double                       std_wall_detector

 Output:
        dvector<Eigen::Vector3d>    filtered_points
*/
vector<Eigen::Vector3d> wallHandle::filterNumbersByStdDeviation(vector<Eigen::Vector3d>& points, const std::vector<double> numbers, double std_wall_detector)
{
    double mean = computeMean(numbers);
    double std_deviation = computeStdDeviation(numbers, mean);

    // Create a new group for filtered points
    vector<Eigen::Vector3d> filtered_points;

    // Filter numbers within ±2 standard deviations
    for (auto i = 0; i < numbers.size(); i++) {
        if (std::abs(numbers[i] - mean) <= std_wall_detector * std_deviation) {
            filtered_points.push_back(points[i]);
        }
    }

    return filtered_points;
}

/*
    Checks if data is normally distributed
 Input:
       const std::vector<double>&       data

 Output:
        bool                            test_statistic <= critical_value
*/
/*bool wallHandle::isNormallyDistributed(const std::vector<double>& data) {
    double significance_level = 0.05;
    if (data.empty()) {
        // Empty data vector, cannot perform the test
        return false;
    }

    // Sort the data vector in ascending order
    std::vector<double> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());

    // Calculate the mean and standard deviation of the data
    double mean = gsl_stats_mean(&sorted_data[0], 1, sorted_data.size());
    double std_deviation = gsl_stats_sd(&sorted_data[0], 1, sorted_data.size());

    // Calculate the test statistic (D) and p-value
    double test_statistic = 0.0;
    for (size_t i = 0; i < sorted_data.size(); ++i) {
        double F_obs = gsl_cdf_ugaussian_P((sorted_data[i] - mean) / std_deviation);
        double F_exp = (i + 1.0) / sorted_data.size();
        double diff = std::abs(F_obs - F_exp);
        if (diff > test_statistic) {
            test_statistic = diff;
        }
    }

    // Calculate the critical value for the given significance level and sample size
    double critical_value = gsl_cdf_ugaussian_Pinv(1.0 - significance_level / 2.0) / std::sqrt(sorted_data.size());

    return test_statistic <= critical_value;
}*/

/* 
    Function that find the plane that minimizes the distance to a set of points
 Input:
       const vector<Eigen::Vector3d>&       points

 Output:
        Eigen::Vector4d   plane equation (ax+by+cz+d=0)
*/
Eigen::Vector4d wallHandle::findMinimizingPlane(const vector<Eigen::Vector3d>& points) 
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> A(points.size(), 3);

    // Fill the matrix A with points
    for (size_t i = 0; i < points.size(); ++i) {
        A.row(i) = points[i].transpose();
    }

    // Compute the centroid of the points
    Eigen::Vector3d centroid = A.colwise().mean();

    // Subtract the centroid from each point to center them around the origin
    A.rowwise() -= centroid.transpose();

    // Compute the singular value decomposition (SVD) of A
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // The last column of V contains the normal vector of the plane
    Eigen::Vector3d normal = svd.matrixV().col(2);

    // The plane equation: ax + by + cz + d = 0
    // Compute the d value (distance from origin to the plane)
    double d = -normal.dot(centroid);

    // Return the plane as a 4D vector (a, b, c, d)
    return Eigen::Vector4d(normal.x(), normal.y(), normal.z(), d);
}

/* 
    Function that calculate the angle between 2 planes
 Input:
       Eigen::Vector3d  first_normal_vector
       Eigen::Vector3d  second_normal_vector

 Output:
        double          degrees_angle  
*/
double wallHandle::angleBetweenPlanes(Eigen::Vector3d first_normal_vector, Eigen::Vector3d second_normal_vector = Eigen::Vector3d(0, 1, 0))
{
    double dot_prod = (first_normal_vector[0] * second_normal_vector[0]) + (first_normal_vector[1] * second_normal_vector[1]) + (first_normal_vector[2] * second_normal_vector[2]);
    double mag1 = sqrt((first_normal_vector[0] * first_normal_vector[0]) + (first_normal_vector[1] * first_normal_vector[1]) + (first_normal_vector[2] * first_normal_vector[2]));
    double mag2 = sqrt((second_normal_vector[0] * second_normal_vector[0]) + (second_normal_vector[1] * second_normal_vector[1]) + (second_normal_vector[2] * second_normal_vector[2]));

    double cos_angle = dot_prod / (mag1 * mag2);
    double radian_angle = acos(cos_angle);

    // Convert the angle from radians to degrees
    double degrees_angle = radian_angle * (180.0 / M_PI);
    
    return degrees_angle;
}

/*
    Function that calculate the normal value of point
 Input:
       Eigen::Vector3d&     point

 Output:
        Eigen::Vector3d     normalize_point
*/
Eigen::Vector3d wallHandle::normalizePoint(Eigen::Vector3d& point) 
{
    Eigen::Vector3d normalize_point = Eigen::Vector3d(0,0,0);
    //for (auto i = 0; i < point.size(); i++) {
    //    double result = point[i] / point.norm();
    //    normalize_point[i] = result;
    //}

    // Normalize the vector between 0 and 1
    for (auto i = 0; i < point.size(); i++) {
        double result = (point[i] - DBL_MIN) / (DBL_MAX- DBL_MIN);
        normalize_point[i] = result;
    }
    return normalize_point;
}

/* 
    Function that for given points - decides whether they are a wall
 Input:
      vector <Eigen::Vector3d>&     points
      double                        std_wall_detector
      vector <Eigen::Vector3d>&     filtered_points

 Output:
        bool                        is_wall
*/
bool wallHandle::wallDetector(vector <Eigen::Vector3d>& points, double std_wall_detector, vector <Eigen::Vector3d>& filtered_points)
{
    bool is_wall = false;
    vector<Eigen::Vector3d> normalize_points;
    // Normalize points values between 0 and 1
    for (auto point : points)
    {
        normalize_points.push_back(normalizePoint(point));
    }

    std::vector<double> z_cord;
    for (Eigen::Vector3d point : points) {
        z_cord.push_back(point.z());
    }

    filtered_points = filterNumbersByStdDeviation(points, z_cord, std_wall_detector);
    
   /*if (!isNormallyDistributed(z_cord))
    {
        std::cout << "is not a wall-1" << std::endl;
        is_wall = false;
        return is_wall;
    }*/

    // Find the plane that minimizes the distance to the points
    Eigen::Vector4d plane = findMinimizingPlane(filtered_points);
    Eigen::Vector3d plane_normal = Eigen::Vector3d(plane[0], plane[1], plane[2]);

    // Find the angle between the plane and XZ-plane
    double angle_between_plane_and_XZplane = angleBetweenPlanes(plane_normal);
    std::cout << "Angle between minimize plane and XZplane: " << angle_between_plane_and_XZplane << std::endl;
    if (angle_between_plane_and_XZplane >= 88 && angle_between_plane_and_XZplane <= 92)
    {
        is_wall = true;
    }
    else {
        is_wall = false;
    }
    return is_wall;
}

/*
    Extracts vector and compute his average
 Input:
      int                           index
      vector<Eigen::Vector3d>&      points

 Output:
        double                      computeMean(cord)
*/
double wallHandle::getAverageCord(int index, vector<Eigen::Vector3d>& points)
{
    std::vector<double> cord;
    for (Eigen::Vector3d point : points) {
        cord.push_back(point[index]);
    }
    
    return computeMean(cord);
}
