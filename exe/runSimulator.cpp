//
// Created by tzuk on 6/4/23.
//
#include "simulator.h"
#include "include/Auxiliary.h"

/*
    Moves the drone back if the distance is small
 Input:
       wallHandle&               wall
       vector <Eigen::Vector3d>  points
       Simulator&                simulator
*/
void goBack(wallHandle& wall, vector <Eigen::Vector3d> points, Simulator& simulator)
{
    /* Tryibg fixing absolute location or relative location
    cv::Mat camera_center_mat = simulator.GetSLAM()->GetTracker()->getLastKeyFrame()->GetCameraCenter();
    current_location_mat -= CameraCenter;
    float x = current_location_mat.at<float>(0, 3) - camera_center_mat.at<float>(0,0);
    float y = current_location_mat.at<float>(1, 3) - camera_center_mat.at<float>(1 , 0 );
    float z = current_location_mat.at<float>(2, 3) - camera_center_mat.at<float>(2 , 0 );

    float x = camera_center_mat.at<float>(0,0);
    float y = camera_center_mat.at<float>(1 , 0 );
    float z = camera_center_mat.at<float>(2 , 0 );
    Eigen::Vector3d camera_center = Eigen::Vector3d(x, y, z);

        for (auto point : points) {
        point -= camera_center;
    }*/

    /* old way to calc avrage
    float average_x = wall.getAverageCord(0, points);
    float average_y = wall.getAverageCord(1, points);
    float average_z = wall.getAverageCord(2, points);
    Eigen::Vector3d average_point = Eigen::Vector3d(average_x, average_y, average_z);
    */

    //Save current location of the camera
    cv::Mat current_location_mat = simulator.getCurrentLocation();
    Eigen::Vector3d current_location = Eigen::Vector3d(current_location_mat.at<float>(0, 3), current_location_mat.at<float>(1, 3), current_location_mat.at<float>(2, 3));

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    // Calculate the sum of all vectors
    for (Eigen::Vector3d point : points) {
        sum += point;
    }
    // Calculate the average by dividing the sum by the number of points
    int numVectors = points.size();
    Eigen::Vector3d average = sum / numVectors;

    // Euclidean distance
    float dx = average[0] - current_location[0];
    //double dy = average[1] - current_location[1];
    float dz = average[2] - current_location[2];

    float distance = std::sqrt((dx * dx) + (dz * dz));
    ////double distance = (current_location - average_point).norm();

    if (distance >= 1.5)
    {
        std::string c = "back 0.5";
        simulator.command(c);
        std::cout << "Go Back!" << std::endl;
    }
}

/*
    check if the drone facing a wall
 Input:
       wallHandle&               wall
       vector <Eigen::Vector3d>  points
       Simulator&                simulator
*/
void wallDetector(Simulator& simulator, double std_wall_detector) {
    while (!simulator.getStopFlag())
    {
        if (!simulator.getIsLocolaized())
        {
            Sleep(1000); // Sleep for 1 second
            Sleep(500); // Sleep for 1 second
            continue;
        }

        auto lastKeyFrame = simulator.GetSLAM()->GetTracker()->getLastKeyFrame();
        auto currentPoints = lastKeyFrame->GetMapPoints();
        std::vector<Eigen::Vector3d> points;
        for (auto point : currentPoints)
        {
            points.emplace_back(ORB_SLAM2::Converter::toVector3d(point->GetWorldPos()));
        }

        vector <Eigen::Vector3d> filltered_points;
        wallHandle wall;
        bool isWall = wall.wallDetector(points, std_wall_detector, filltered_points);
        if (isWall)
        {
            std::cout << "It is a wall!" << std::endl;
            goBack(wall, filltered_points, simulator);
        }
        else
        {
            std::cout << "It is not a wall!" << std::endl;
        }
        Sleep(1000); // Sleep for 1 second
    }
}

int main(int argc, char** argv)
{
    std::ifstream programData(argv[1]);
    nlohmann::json data;
    programData >> data;
    programData.close();

    std::string configPath = data["DroneYamlPathSlam"];
    std::string modelTextureNameToAlignTo = data["modelTextureNameToAlignTo"];
    std::string model_path = data["modelPath"];
    std::string vocabulary_path = data["VocabularyPath"];
    std::string simulatorOutputDir = data["simulatorOutputDir"];
    double movementFactor = data["movementFactor"];
    bool trackImages = data["trackImages"];
    double std_wall_detector = data["stdWallDetector"];

    Simulator simulator(configPath, model_path, modelTextureNameToAlignTo, trackImages, false, simulatorOutputDir, false, "", movementFactor, vocabulary_path);
    auto simulatorThread = simulator.run();
    //simulator.simulatorRunThread();
    while (!simulator.isReady())
    { // wait for the 3D model to load
        Sleep(1);
    }

    std::cin.get();
    // Check if there is a wall
    std::thread detectionThread(wallDetector, std::ref(simulator), std_wall_detector);

    /* simulator.setTrack(true);
     int currentYaw = 0;
     int angle = 10;
     cv::Mat currentLocation;
     for (int i = 0; i < std::ceil(360 / angle); i++)
     {
         std::string c = "forward 0.5";
         simulator.command(c);
         currentLocation = simulator.getCurrentLocation();
         c = "back 0.5";
         simulator.command(c);
         currentLocation = simulator.getCurrentLocation();
         std::cout << "currentLocation: " << currentLocation << std::endl;
         c = "cw " + std::to_string(angle);
         simulator.command(c);
         currentLocation = simulator.getCurrentLocation();
     }
     auto scanMap = simulator.getCurrentMap();*/

    simulatorThread.join();
}