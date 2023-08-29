#include "WallDetection.h"



double wallHandle::computeMean(const std::vector<double> value)
{
    double sum = 0;
    for (auto e : value) { sum += e; }
    return sum / value.size();
}

double wallHandle::computeStdDeviation(const std::vector<double> values, double mean)
{
    double varianceSum = 0;
    for (auto value : values) 
    { 
        varianceSum += (value - mean) * (value - mean); 
    }
    double variance = varianceSum / values.size();
    return sqrt(variance);
}

std::vector<double> wallHandle::filterNumbersByStdDeviation(const std::vector<double> numbers)
{
    double mean = computeMean(numbers);
    double stdDev = computeStdDeviation(numbers, mean);

    // Create a new group for filtered numbers
    std::vector<double> filteredNumbers;

    // Filter numbers within ±2 standard deviations
    for (double num : numbers) {
        if (std::abs(num - mean) <= 2 * stdDev) {
            filteredNumbers.push_back(num);
        }
    }

    return filteredNumbers;
}

bool wallHandle::isNormallyDistributed(const std::vector<double>& data) {
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
    double stddev = gsl_stats_sd(&sorted_data[0], 1, sorted_data.size());

    // Calculate the test statistic (D) and p-value
    double D = 0.0;
    for (size_t i = 0; i < sorted_data.size(); ++i) {
        double F_obs = gsl_cdf_ugaussian_P((sorted_data[i] - mean) / stddev);
        double F_exp = (i + 1.0) / sorted_data.size();
        double diff = std::abs(F_obs - F_exp);
        if (diff > D) {
            D = diff;
        }
    }

    // Calculate the critical value for the given significance level and sample size
    double critical_value = gsl_cdf_ugaussian_Pinv(1.0 - significance_level / 2.0) / std::sqrt(sorted_data.size());

    return D <= critical_value;
}

/* Function that find the plane that minimizes the distance to a set of points
 input:
       vector<Eigen::Vector3d> points

 output:
        Eigen::Vector4d   plane equation (ax+by+cz+d=0)
*/
Eigen::Vector4d wallHandle::findMinimizingPlane(const vector<Eigen::Vector3d>& points) {
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

/* Function that calculate the angle between given plane to X - Z Plane
 input:
       Eigen::Vector3d& plane (coefficients a,b,c of the plane equation)

 output:
        double    degrees_angle (in degrees)   
*/
double wallHandle::angleBetweenPlanes(Eigen::Vector3d firstNormalVector, Eigen::Vector3d SecNormalVector = Eigen::Vector3d(0, 1, 0))
{
    double dot_prod = (firstNormalVector[0] * SecNormalVector[0]) + (firstNormalVector[1] * SecNormalVector[1]) + (firstNormalVector[2] * SecNormalVector[2]);
    double mag1 = sqrt((firstNormalVector[0] * firstNormalVector[0]) + (firstNormalVector[1] * firstNormalVector[1]) + (firstNormalVector[2] * firstNormalVector[2]));
    double mag2 = sqrt((SecNormalVector[0] * SecNormalVector[0]) + (SecNormalVector[1] * SecNormalVector[1]) + (SecNormalVector[2] * SecNormalVector[2]));

    double cos_angle = dot_prod / (mag1 * mag2);
    double radian_angle = acos(cos_angle);

    // Convert the angle from radians to degrees
    double degrees_angle = radian_angle * (180.0 / M_PI);
    return degrees_angle;
}


void wallHandle::normalizeVector(std::vector<double>& vec) {
    // Find the minimum and maximum values in the vector
    double minVal = *std::min_element(vec.begin(), vec.end());
    double maxVal = *std::max_element(vec.begin(), vec.end());

    // Normalize the vector between 0 and 1
    double range = maxVal - minVal;
    if (range > 0.0) {
        for (double& value : vec) {
            value = (value - minVal) / range;
        }
    }
}

/* Function that given points - decides whether they are a wall
 input:
       vector<Eigen::Vector3d> points

 output:
        bool  is_wall
*/
bool wallHandle::wallDetector(vector <Eigen::Vector3d>& points)
{
    bool is_wall = false;

    std::vector<double> z_cord;
    for (Eigen::Vector3d point : points) {
        z_cord.push_back(point.z());
    }

    // Normalize z_cord values between 0 and 1
    normalizeVector(z_cord);

   /* if (!isNormallyDistributed(z_cord))
    {
        std::cout << "is not a wall-1" << std::endl;
        is_wall = false;
        return is_wall;
    }*/

    // Find the plane that minimizes the distance to the points
    Eigen::Vector4d plane = findMinimizingPlane(points);
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

double wallHandle::getAverageCord(int index, vector<Eigen::Vector3d>& points)
{
    std::vector<double> cord;
    for (Eigen::Vector3d point : points) {
        cord.push_back(point[index]);
    }
    
    return computeMean(cord);
}


//
//int main() 
//{
//    vector<Test> tests;
//    vector<bool> actual_labels;
//    vector<bool> predicted_labels;
//    int numClasses = 2;
//
//    for (int i = 0; i < 10000; i++) {
//
//        if (i < 5000) {
//            tests.emplace_back(true);
//            actual_labels.push_back(true);
//        }
//        else {
//            tests.emplace_back(false);
//            actual_labels.push_back(false);
//        }
//    }
//    int counter = 0;
//    for (auto test : tests)
//    {
//        predicted_labels.push_back(wallDetector(test.points));
//        counter++;
//    }
//
//    vector<vector<int>> confusionMatrix(numClasses, vector<int>(numClasses, 0));
//
//    // Update the confusion matrix based on the predicted and actual labels
//    for (size_t i = 0; i < predicted_labels.size(); ++i) {
//        int actual_class = actual_labels[i];
//        int predicted_class = predicted_labels[i];
//        confusionMatrix[actual_class][predicted_class]++;
//    }
//
//    return 0;
//}
//
