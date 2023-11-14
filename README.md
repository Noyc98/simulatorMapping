# wall detector integreted in simulator mapping
Authors:

- Michal Marom
- Noy Cohen

## project flow

### step 1: creating data for testing
random 2000 points 
 translate the project report here!

## describe the algorithm + photos
detecting a wall (depending of distance):
![](https://github.com/Noyc98/simulatorMapping/assets/110714301/c84bf759-5a1f-400a-a665-787bc133ce25)
![](https://github.com/Noyc98/simulatorMapping/assets/110714301/009cd5b2-9360-4764-9719-25efae72bb28)


## Installation and how to run the project?

### Running the Simulator

1. Start by running the installation script:

   ```bash
   ./install.sh

2. After installing the program, follow these steps to run the simulator:

Offline ORB-SLAM Video
i. Extract ORB vocabulary:

    cd Vocabulary
    tar -xf ORBvoc.txt.tar.gz
    cd ..

ii. Update generalSettings.json:

  - Set VocabularyPath to ${simulatorMappingDir}/Vocabulary/ORBvoc.txt

  - Set DroneYamlPathSlam to ${simulatorMappingDir}/config/tello_9F5EC2_640.yaml

  - Set offlineVideoTestPath to the path where the video is saved
    
  - Set simulatorOutputDir to the directory where the result will be saved

  > Note: Sample videos are available in the shared simulator Google Drive directory.

iii. Run the offline ORB-SLAM:
  
    ```bash
    build/offline_orb_slam


Adjusting Simulator Settings
i. Update the result directory in generalSettings.json to your desired path and name.

ii. Update generalSettings.json:

  - Set mapInputDir to the location where the offline result is saved.
  - Set initial simulator position and orientation parameters:
    - startingCameraPosX
    - startingCameraPosY
    - startingCameraPosZ
    - yawRad
    - pitchRad
    - rollRad

iii. Set movement and rotation scales:

  - Set __movingScale__ to control movement when pressing moving buttons.
  - Set __rotateScale__ to control rotation when using rotate buttons (in radians).


I) Change the result of the offline_orb_slam dir to the path and the name you want it to be

II) Change in generalSettings.json: *mapInputDir* to where the result of the offline saved.

III) Change *startingCameraPosX*, *startingCameraPosY*, *startingCameraPosZ*, *yawRad*, *pitchRad* and *rollRad* to where you want the first position of the simulator to be.

IV) Change *movingScale* to how much you want to move when press on moving button and *rotateScale* to how much you want to rotate in rotate buttons(in radians)

3. Now we want to run ./mapping to open orb-slam.

I) Run ./mapping

II) Click Open Simulator button when you want to and now the simulator opens in the first position you defined

III) Move with moving buttons and rotate with rotate buttons

