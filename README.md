![example](results/ICRA2024/others/soft-fixture-scenarios.png)

This repo provides code for the paper - Soft Fixtures: Practical Caging-Based Manipulation of Rigid and Deformable Objects

[[PREPRINT]](...) [[WEBSITE]](https://sites.google.com/view/softfixture/home) [[VIDEOS]](https://www.youtube.com/playlist?list=PLYP3395168-Swf4ZDVI4SwARKKHaQ84HB)

# Environment
Tested with:<br>
**Python 3.10**<br>
**Ubuntu22.04**

# Installation instructions:

0. Create a virtual environment (optional but recommended)

You can create a virtual environment using tools like virtualenv or conda. 

1. Install OMPL with its Python bindings

Please first refer to the official installation [instructions](https://ompl.kavrakilab.org/installation.html)

For Ubuntu systems, you could simply do:

1.1 Download the OMPL (installation script)[https://ompl.kavrakilab.org/install-ompl-ubuntu.sh]

1.2 Make it executable:
```
chmod u+x install-ompl-ubuntu.sh
```

1.3 Install the latest release of OMPL.app with Python bindings:
```
./install-ompl-ubuntu.sh --app
```

You can also choose to install from source.

2. Install other dependencies
```
pip install -r requirements.txt
```

# Usage
Two examples are provided.
This demo plans the arm motion of a Franka robot.
```
python examples/demo_franka.py
```

This demo plans whole-body motion of a planar 4-link snake-like robot.
```
python examples/demo_planar.py
```

# Additional Information
1. Currently tested planners include PRM, RRT, RRTstar, RRTConnect, EST, FMT* and BIT*. But all planners in OMPL should work. Just add them in the set_planner API in PbOMPL class.
2. To work with other robot, you might need to inherit from PbOMPLRobot class in PbOMPL and override several functionalities. Refer to my_planar_robot.py for an example. Refer to demo_plannar.py for an example.

# Acknowledgement
The code is partly built upon the repo - [pybullet_ompl](https://github.com/lyfkyle/pybullet_ompl.git), which provides interface to use OMPL for motion planning inside PyBullet.

# Contact or Support
Please contact Yifei Dong at yifeid@kth.se or Prof. Florian Pokorny at fpokorny@kth.se

# Contributors
The authors are with the division of Robotics, Perception and Learning, KTH Royal Institute of Technology, 100 44 Stockholm, Sweden. Funded by the European Commission under the Horizon Europe Framework Programme project [SoftEnable](http://softenable.eu/), grant number 101070600.