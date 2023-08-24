//
// Created by tzuk on 6/4/23.
//
#include "simulator.h"
#include "include/Auxiliary.h"
void wallDetector(Simulator &simulator) {
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

        wallHandle wall;
        bool isWall = wall.wallDetector(points);
        if (isWall) {
            std::cout << "It is a wall!" << std::endl;
        }
        else {
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

    Simulator simulator(configPath, model_path, modelTextureNameToAlignTo, trackImages, false, simulatorOutputDir, false, "", movementFactor, vocabulary_path);
    auto simulatorThread = simulator.run();
    //simulator.simulatorRunThread();
    while (!simulator.isReady())
    { // wait for the 3D model to load
        Sleep(1);
    }

    std::cin.get();
    std::thread detectionThread(wallDetector,std::ref(simulator));
   
    simulator.setTrack(true);
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
        c = "cw " + std::to_string(angle);
        simulator.command(c);
        currentLocation = simulator.getCurrentLocation();
    }
    auto scanMap = simulator.getCurrentMap();

    simulatorThread.join();
}