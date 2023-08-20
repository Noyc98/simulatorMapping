//
// Created by tzuk on 6/4/23.
//
#include "simulator.h"
#include "include/Auxiliary.h"

int main(int argc, char **argv)
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

    auto lastKeyFrame = simulator.GetSLAM()->GetTracker()->getLastKeyFrame();
    auto currentPoints = lastKeyFrame->GetMapPoints();
    std::vector<Eigen::Vector3d> points;
    for (auto point : currentPoints)
    {
        points.emplace_back(ORB_SLAM2::Converter::toVector3d(point->GetWorldPos()));
    }

    wallHandle wall;
    wall.wallDetector(points);

    //simulatorThread.join();
}