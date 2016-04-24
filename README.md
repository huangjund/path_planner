### Hybrid A* Path Planner for the KTH Research Concept Vehicle

The code in this repository is the result of my master's thesis which I am currently writing at the Integrated Research Lab (ITRL) at KTH Royal Institute of Technology.


The goal of the thesis and hence this code is to create a real-time path planning algorithm for the nonholonomic Research Concept Vehicle (RCV). The algorithm uses a binary obstacle map as an input, generated using LIDAR mounted on top of the vehicle. The algorithm is being developed using C++ due to real-time requirements in combination with ROS to ensure modularity and portability as well as using RViz as a visualization/simulation environment.

##### Key Characteristics
* Sampling in continuous space with 72 different headings per cell (5° discretization)
* Constrained Heuristic - _nonholonomic without obstacles_: Dubin's Path&mdash;[Video] (https://www.youtube.com/watch?v=VNo9fU6XEGE)
* Unconstrained Heuristic - _holonomic with obstacles_: 2D A* Search&mdash;[Video] (https://www.youtube.com/watch?v=Ip2iUrVoFXc)
* Dubin's Shot
* C++ real-time implementation (~10 Hz)

##### Videos
* [Path Planning with Sensor Fusion] (https://www.youtube.com/watch?v=GwIU00jukO4)
* [Path Planning with Search Visualization] (https://www.youtube.com/watch?v=1WZEQtg8ZZ4)
* [Path Planning without Search Visualization] (https://www.youtube.com/watch?v=QfxHnb6IQzM)

Large parts of the implementation are closely related to the hybrid A* algorithm developed by Dmitri Dolgov and Sebastian Thrun (_Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments_ DOI: 10.1177/0278364909359210)