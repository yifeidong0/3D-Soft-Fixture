![example](results/ICRA2024/others/soft-fixture-scenarios.png)

This repo provides code for the paper - Soft Fixtures: Practical Caging-Based Manipulation of Rigid and Deformable Objects

[[PREPRINT]](...) [[WEBSITE]](https://sites.google.com/view/softfixture/home) [[VIDEOS]](https://www.youtube.com/playlist?list=PLYP3395168-Swf4ZDVI4SwARKKHaQ84HB)

## Installation instructions:

Tested with:<br>
**Python 3.10**<br>
**Ubuntu 22.04**

### 0. Create a virtual environment (optional but recommended)

You can create a virtual environment using tools like virtualenv or conda. 

### 1. Install OMPL with its Python bindings

Please first refer to the official installation [instructions](https://ompl.kavrakilab.org/installation.html)

For Ubuntu systems, you could simply do:

1.1 Download the OMPL [installation script](https://ompl.kavrakilab.org/install-ompl-ubuntu.sh)

1.2 Make it executable:
```
chmod u+x install-ompl-ubuntu.sh
```

1.3 Install the latest release of OMPL.app with Python bindings:
```
./install-ompl-ubuntu.sh --app
```

You can also choose to install from source.

### 2. Install other dependencies
```
pip install -r requirements.txt
```

## Usage

### 1. Run escape energy analysis over a scenario

Here is a table of arguments you could use to customize your algorithm.

| Argument        | Short Option | Long Option       | Default Value       | Choices                      | Description                                                                                                                                      |
|-----------------|--------------|-------------------|---------------------|------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| scenario        | -c           | --scenario        | ShovelFish          | ShovelFish, MaskEar, StarfishBowl, HookFishHole, HandbagGripper, BimanualRubic | Specify the scenario for the demo.                                                                                                                |
| planner         | -p           | --planner         | BITstar             | BITstar, InformedRRTstar, RRTstar, RRT, AITstar | Specify the optimal planner to use.                                                                                                              |
| object          | -j           | --object          | Fish                | Fish, MaskBand, Starfish, FishHole, Chain, Rubic | Specify the object to cage.                                                                                                                      |
| obstacle        | -l           | --obstacle        | Shovel              | Shovel, Ear, LeftHandAndBowl, Hook, 3fGripper | Specify the obstacle that cages the object.                                                                                                      |
| runtime         | -t           | --runtime         | 30                  | N/A                          | Specify the runtime in seconds. Must be greater than 0.                                                                                          |
| visualization   | -v           | --visualization   | True                   | False, True                  | Specify whether to visualize the PyBullet GUI.                                                                                                   |

A sample command is provided as follows,
```
python3 src/runScenario.py -c ShovelFish -p BITstar -j Fish -l Shovel -t 30 -v 1 
```

After running it, you will obtain results as following,

![example](results/ICRA2024/videos/scooping_fish.gif)

Please follow the correspondance in the arguments as below,

| Scenario        | Object             | Obstacle     |
|-----------------|--------------------|--------------|
| ShovelFish      | Fish               | Shovel       |
| MaskEar         | MaskBand           | Ear          |
| StarfishBowl    | Starfish           | LeftHandAndBowl |
| HookFishHole    | FishHole           | Hook         |
| HandbagGripper  | Chain              | 3fGripper    |
| BimanualRubic   | Rubic              | 3fGripper    |

### 2. Physical experiment

To reproduce our results on a Emika Franka Panda 7-axis robot arm, please 

2.1 Refer to Franka FCI [instructions](https://frankaemika.github.io/docs/installation_linux.html#building-from-source) and build libfranka

2.2 Copy the cpp script src/franka/generate_cartesian_pose_random_shaking.cpp to [the path](https://github.com/frankaemika/libfranka/tree/master/examples) and add it to CMakeLists.txt

2.3 Rebuild the project
```
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
cmake --build .
```

2.4 Run the script
```
~/libfranka/build$ ./examples/generate_cartesian_pose_motion <robot-ip>
```
## Acknowledgement
The code is partly built upon the repo - [pybullet_ompl](https://github.com/lyfkyle/pybullet_ompl.git), which provides interface to use OMPL for motion planning inside PyBullet.

## Contact or Support
Please contact Yifei Dong at yifeid@kth.se or Prof. Florian Pokorny at fpokorny@kth.se

## Contributors
The authors are with the division of Robotics, Perception and Learning, KTH Royal Institute of Technology, 100 44 Stockholm, Sweden. Funded by the European Commission under the Horizon Europe Framework Programme project [SoftEnable](http://softenable.eu/), grant number 101070600.
