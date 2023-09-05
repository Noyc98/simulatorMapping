#include "WallDetection.h"


// **** Math Functions **** //
/*
    Compute the mean of a vector
 Input:
       vector<double>   value

 Output:
        double          sum / double(value.size())
*/
Eigen::Vector3d WallHandler::computeMean(vector<Eigen::Vector3d>& points)
{
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (Eigen::Vector3d& point : points) {
        sum += point;
    }
    // Calculate the average by dividing the sum by the number of points
    int numVectors = points.size();
    Eigen::Vector3d mean = sum / numVectors;

    return mean;
}

/*
    Compute the Std Deviation of a vector
 Input:
       vector<double>   values
       double           mean

 Output:
        double          sqrt(variance)
*/
Eigen::Vector3d WallHandler::computeStdDeviation(vector<Eigen::Vector3d>& points, Eigen::Vector3d mean)
{
    Eigen::Vector3d variance_sum = Eigen::Vector3d::Zero();
    for (Eigen::Vector3d& point : points)
    {
        variance_sum += (point - mean).cwiseProduct(point - mean);
    }
    Eigen::Vector3d std_deviation = (variance_sum / double(points.size())).array().sqrt();

    return std_deviation;
}

/*
    Function that calculate the normal value of point
 Input:
       Eigen::Vector3d&     point

 Output:
        Eigen::Vector3d     normalize_point
*/
Eigen::Vector3d WallHandler::normalizePoint(Eigen::Vector3d& point)
{
    Eigen::Vector3d normalize_point = Eigen::Vector3d(0, 0, 0);

    // Normalize the vector between 0 and 1 with mam-min value of double
    for (auto i = 0; i < point.size(); i++) {
        double result = (point[i] - DBL_MIN) / (DBL_MAX - DBL_MIN);
        normalize_point[i] = result;
    }

    return normalize_point;
}

// **** Help Functions for Calculate **** //
/*
    3D vector of points filltered by std_wall_detector to ignore outliers
 Input:
       vector<Eigen::Vector3d>&     points
       vector<double>               numbers
       double                       std_wall_detector

 Output:
        vector<Eigen::Vector3d>    filtered_points
*/
vector<Eigen::Vector3d> WallHandler::filterNumbersByStdDeviation(vector<Eigen::Vector3d>& points, const std::vector<double> numbers, double std_wall_detector)
{
    // Calculate mean
    Eigen::Vector3d mean = computeMean(points);
    // Calculate standard deviation
    Eigen::Vector3d std_deviation = computeStdDeviation(points, mean);

    // Create a new group for filtered points
    vector<Eigen::Vector3d> filtered_points;

    // Filter numbers within ±std_wall_detector standard deviations
    for (Eigen::Vector3d& point : points){
        Eigen::Vector3d vec1 = (point - mean).array().abs();
        std_deviation = std_deviation * std_wall_detector;
        if (vec1[0] < std_deviation[0] && vec1[1] < std_deviation[1] && vec1[2] < std_deviation[2])
        {
            filtered_points.push_back(point);
        }
    }

    return filtered_points;
}

/*
    Function that find the plane that minimizes the distance to a set of points
 Input:
       vector<Eigen::Vector3d>&     points

 Output:
        Eigen::Vector4d             plane equation (ax+by+cz+d=0)
*/
Eigen::Vector4d WallHandler::findMinimizingPlane(const vector<Eigen::Vector3d>& points)
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> matrix(points.size(), 3);

    // Fill the matrix with points
    for (size_t i = 0; i < points.size(); ++i) {
        matrix.row(i) = points[i].transpose();
    }

    // Compute the centroid of the points
    Eigen::Vector3d centroid = matrix.colwise().mean();

    // Subtract the centroid from each point to center them around the origin
    matrix.rowwise() -= centroid.transpose();

    // Compute the singular value decomposition (SVD) of the matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

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
double WallHandler::angleBetweenPlanes(Eigen::Vector3d first_normal_vector, Eigen::Vector3d second_normal_vector = Eigen::Vector3d(0, 1, 0))
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


// **** Main Function - Wall Detector **** //
/*
    Function that for given points - decides whether they are a wall
 Input:
      vector <Eigen::Vector3d>&     points
      double                        std_wall_detector
      vector <Eigen::Vector3d>&     filtered_points

 Output:
        bool                        is_wall
*/
bool WallHandler::wallDetector(vector <Eigen::Vector3d>& points, double std_wall_detector, vector <Eigen::Vector3d>& filtered_points)
{
    bool is_wall = false;
    vector<Eigen::Vector3d> normalize_points;

    // Normalize points
    for (auto point : points)
    {
        normalize_points.push_back(normalizePoint(point));
    }

    // Filter the points to ignore outliers
    std::vector<double> z_cord;
    for (Eigen::Vector3d point : points) {
        z_cord.push_back(point.z());
    }
    filtered_points = filterNumbersByStdDeviation(points, z_cord, std_wall_detector);

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