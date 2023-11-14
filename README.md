# Wall detector integreted in simulator mapping
Authors:

- Michal Marom
- Noy Cohen

## Project flow

### Step 1: Creating Data for Testing
Generate 2000 random points, evenly split into "wall" and "not wall" categories, following these criteria:
1. Coordinates X and Y are uniformly drawn from the range [0, 1].
2. Z-coordinate:
   - If labeled as "wall," draw from a normal distribution.
   - If labeled as "not wall," draw uniformly.

### Step 2: Wall Detection on the Created Data
Define the wall identification problem through two tests:
1. Verify normal distribution of the Z-axis, as walls are expected to be straight without depth.
2. Check that the plane of the given points is perpendicular to the Z-X plane, considering a wall perpendicular to the floor (angle range: 88-92 degrees).
Run the code and obtain the confusion matrix:

<img src="https://github.com/Noyc98/simulatorMapping/assets/110714301/ca09439a-cbb7-44d5-9992-1d25187a06fa" width="200">

### Step 3: Integration of Wall Detection Algorithm in the Simulator
- Run the simulator.
- Add the WallHandler class to identify walls using simulator points.
- Discover that the initial test for wall identification by normal distribution is not relevant due to noise in orb-slam.
- Investigate and find that the data creation test worked well because it assumed a normal distribution for the Z-axis coordinate.
- Remove the Z-axis normal distribution test, resulting in successful and accurate wall identification:

<img src="https://github.com/Noyc98/simulatorMapping/assets/110714301/7842395c-1ed0-4bd2-82a0-5c33c3cecb0c" width="400">
<br>
<img src="https://github.com/Noyc98/simulatorMapping/assets/110714301/a82bbfd0-57d7-426c-bd9c-52e43e698973" width="400">

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
   ```
 
 ### Adjusting Simulator Settings
  1. i. Update the result directory in generalSettings.json to your desired path and name.
  
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
 
 2. Run the mapping script to open ORB-SLAM:
 
       ```bash
       ./mapping
       ```
 
 3. Click the "Open Simulator" button to launch the simulator in the defined initial position.
 
 4. Move using the provided buttons for translation and rotation.

