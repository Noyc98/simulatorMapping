//
// Created by tzuk on 6/4/23.
//
#include "simulator.h"
#include "include/Auxiliary.h"

void goBack(wallHandle& wall, vector <Eigen::Vector3d> points, Simulator& simulator)
{
    cv::Mat camera_center_mat = simulator.GetSLAM()->GetTracker()->getLastKeyFrame()->GetCameraCenter();
    cv::Mat current_location_mat = simulator.getCurrentLocation();

    /*float x = current_location_mat.at<float>(0, 3) - camera_center_mat.at<float>(0,0);
    float y = current_location_mat.at<float>(1, 3) - camera_center_mat.at<float>(1 , 0 );
    float z = current_location_mat.at<float>(2, 3) - camera_center_mat.at<float>(2 , 0 );*/

    float x = camera_center_mat.at<float>(0,0);
    float y = camera_center_mat.at<float>(1 , 0 );
    float z = camera_center_mat.at<float>(2 , 0 );
    Eigen::Vector3d camera_center = Eigen::Vector3d(x, y, z);
    Eigen::Vector3d current_location = Eigen::Vector3d(current_location_mat.at<float>(0, 3), current_location_mat.at<float>(1, 3), current_location_mat.at<float>(2, 3));

    for (auto point : points) {
        point -= camera_center;
    }
    //cv::Mat CameraCenter = simulator.GetSLAM()->GetTracker()->getLastKeyFrame()->GetCameraCenter();

    //cv::Mat current_location_mat = simulator.getCurrentLocation();
    //Eigen::Vector3d current_location = Eigen::Vector3d(current_location_mat.at<float>(0, 3), current_location_mat.at<float>(1, 3), current_location_mat.at<float>(2, 3));
    //current_location_mat -= CameraCenter;
    
    float average_x = wall.getAverageCord(0, points);
    float average_y = wall.getAverageCord(1, points);
    float average_z = wall.getAverageCord(2, points);
    Eigen::Vector3d average_point = Eigen::Vector3d(average_x, average_y, average_z);

    // Euclidean distance
    float dx = average_point[0]- current_location[0];
    //double dy = average_point[1] - current_location[1];
    float dz = average_point[2] - current_location[2];

    float distance =  std::sqrt((dx * dx) + (dz * dz));
    ////double distance = (current_location - average_point).norm();
    //
    if (distance >= 1.5)
    {
        std::string c = "back 0.5";
        simulator.command(c);
        std::cout << "Go Back!" << std::endl;
    }
}

void wallDetector(Simulator &simulator, double std_wall_detector) {
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

        /*std::cout << "z-coord: " << std::endl;
        for (const auto& point : points) {
            std::cout << point.z() << ","<< std::endl;
        }*/
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
    std::thread detectionThread(wallDetector,std::ref(simulator), std_wall_detector);


   
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