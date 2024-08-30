Differential Wheeled Robot with Mapping and Autonomous Navigation Capabilities
Team members (G3): Mingyang Li,  Yuxuan Ling,  Zixuan He
1.Introduction
1.1 Project Overview:
The main objective of our project was to design an autonomous mobile robot, specifically modeled as a robot vacuum cleaner, which focused on creating a robot that could autonomously navigate and clean a given environment, similar to commercially available robotic vacuums. This required the robot to map its surroundings, avoid obstacles, and ensure complete coverage of the area, all within a simulated environment built in Gazebo.
1.2 Project Goals:
The primary objectives of the project were:
1. Robot Modelling: Develop a differential wheel robot and equip it with a variety of sensors, such as USB Cameras, Infrared cameras and Lidar.
2. Construction of Gazebo virtual environment: Build a Gazebo simulation environment containing a variety of obstacles to simulate the robot, and verify the performance of different algorithms, including SLAM algorithm and automatic navigation algorithm
3. Validation of SLAM algorithms: For mapping, we plan to test and compare the performance of various SLAM algorithms, such as Hector SLAM, Google's open-source Cartographer (known for its efficiency), and RTAB-Map.
4. Autonomous Navigation: Develop a robot that can navigate through an environment without human intervention, using sensor data to make real-time decisions.
5. Full Coverage Path Planning: Implement an algorithm that ensures the robot can cover all areas of the environment while avoiding obstacles and returning to its starting point.
6. Real-Time Obstacle Avoidance: Ensure that the robot can dynamically detect and avoid obstacles that were not initially included in the map.
These goals required the integration of advanced robotics concepts, including SLAM, path planning, and sensor-based control.
1.3 Team Members and Their Roles:
Our team was composed of several members, each contributing to different aspects of the project:
●[YUXUAN LING]: 
1.CPP Algorithm Design and Implementation: I designed and implemented three different algorithms in Python: Spanning Tree, Genetic Algorithm (GA), and Deep Q-Network (DQN). These algorithms were tested for their effectiveness in Python simulations to determine the best approach for full coverage path planning.
2.Coordinate Transformation Function: I developed a function to convert grid-based coordinates into real-world coordinates, which were then sent to the robot for execution. This function was critical for ensuring the robot could accurately follow the planned path in the Gazebo simulation.
3.Gazebo Simulation of Python Algorithms: I implemented and tested the C++ algorithms in the Gazebo environment. The robot was able to follow the pre-planned paths and automatically avoid simple obstacles during its operation.
●[MINGYANG LI]: 
 1. Use urdf to model the robot(version one) and realize the simulation of the robot sensor in RVIZ. 
 2. Built a virtual environment in GazeBo for the verification of slam algorithm and automatic navigation algorithm. 	
 3. Verification of SLAM algorithms: Compare the performance and mapping effect of different SLAM algorithms.
●[ZIXUAN HE]: 
 1. Design and implement the mopping unit and serving unit by using Solidworks and RVIZ
 2. Upgrade robot(version one) into a multifunctional one(version two) in order to well satisfy the actual application situations. 
 3. Test whether the new robot model function well in Gazebo physical simulation environment and can be operated successfully then modify the parameters of the robot.

2.Robot Design and Description
2.1 Wheel Configuration
2.1.1 Differential Wheels:
●Type: Cylindrical
●Quantity: 2 (Left and Right)
●Position: The wheels are mounted on the left and right sides of the robot's base, positioned 0.19 meters from the center. This configuration allows the robot to move forward and steer by adjusting the rotational speed of each wheel.
●Dimensions: Each wheel has a length of 0.06 meters and a radius of 0.06 meters.
●Mass: Each wheel weighs 0.2 kilograms.
2.1.2 Omnidirectional Wheels:
●Type: Spherical
●Quantity: 2 (Front and Rear)
●Position: These wheels are mounted at the front and rear of the robot’s base, with a distance of 0.18 meters from the center. They provide additional support and allow for multi-directional movement, enhancing the robot’s balance and stability during operation.
●Dimensions: The radius of each wheel is 0.015 meters.
●Mass: Each wheel weighs 0.1 kilograms.
           
             Figure 2.1: Schematic of the Differential Wheel     Figure 2.2: Schematic of the Omnidirectional Wheels
2.2 Base Structure
2.2.1 Base:
●Shape: Box size
●Dimensions: The base is 0.45 meters long, 0.45 meters wide, and 0.16 meters high.
●Mass: The base weighs 2 kilogram.
●Material: The base is colored red, which increases the robot’s visibility in its environment.
2.2.2 Stage:
●Shape: Cylindrical
●Dimensions: The stage has a length of 0.12 meters and a radius of 0.1 meters.
●Mass: The stage weighs 0.5 kilograms.
●Mounting: It is mounted on top of the base, providing additional space and functionality, such as sensor placement. It is necessary to be implemented in case that the lidar sensor will be obscured by the kinect camera.
2.3 Sensor Configuration
2.3.1 Kinect Camera:
●Horizontal Field of View (FOV): 60 degrees, which allows the camera to capture a broad area in front of the robot.
●Update Rate: The camera operates at 20 Hz, capturing 20 frames per second, which provides smooth and continuous visual data.
●Image Resolution: 640 pixels in width and 480 pixels in height.
●Clip Distances: The camera can detect objects at distances ranging from 0.05 meters (near) to 8.0 meters (far).
2.3.2 Lidar Sensor:
●Scanning Parameters: The Lidar sensor captures 360 horizontal samples with a 1-degree resolution, covering an angular range from -3 to 3 degrees.
●Range: The sensor has a detection range from 0.10 meters (minimum) to 6.0 meters (maximum), with a resolution of 0.01 meters.
●Update Rate: The Lidar updates at 5.5 Hz, providing regular scans of the environment.
2.3.3 USB Camera:
●Horizontal Field of View (FOV): Approximately 80 degrees (1.3962634 radians).
●Image Resolution: The camera captures images at a resolution of 1280 pixels in width and 720 pixels in height.
●Image Format: The images are in R8G8B8 format, representing the RGB color model.
●Clip Distances: The camera’s detection range spans from 0.02 meters (near) to 300 meters (far).
●Update Rate: Operating at 30 Hz, the camera captures 30 frames per second, ensuring high-frequency data acquisition.
  
               Figure 2.3: Kinect Camera       Figure 2.4: Lidar Sensor           Figure 2.5: Lidar Sensor
2.4 Extra functional units
2.4.1 Mobile serving unit:
●Shape: Box size
●Dimensions: The base is 0.45 meters long, 0.45 meters wide, and 0.01 meters high.
●Mass: Every platform weighs 0.5 kilogram.
●Mobility: Every mobile serving platform can slides on the perpendicular direction of the plane itself , ranging between -0.1m to 0.1m by using a prismetic joint.
                           
                                 Figure 2.6: Illustration of the funtion & parameters of the serving platform 
2.4.1 Mopping unit :
●Mobility: The mobbing brush(blue) can revolute clockwisely (if inspected on the right side of the robot) when operated to mob the floor. And the dust bag behind the brush can easily collect what collected by the brush.
●Angular velocity : 3.14 rad/s
●Mass:
Cleaning unit base : 1.0 kg
Brush wheel : 0.09 kg
               
      Figure 2.7: Mobbing unit illustrated in RVIZ                Figure 2.7: Mobbing unit illustrated in Solidworks 

3.Project Objective and Robot Tasks
3.1 Project Specific Objectives:
The main objective of our project was to design and implement a fully autonomous robot capable of navigating and covering an entire environment while avoiding obstacles. This required the development and integration of several advanced algorithms and systems, including SLAM (Simultaneous Localization and Mapping) and full coverage path planning. The robot needed to perform in a simulated environment created in Gazebo, which closely mimics real-world scenarios.
3.2 Assigned Tasks for the Robot:
The key tasks assigned to the robot included:
1.Robot Modelling:Designing a differential wheeled robot with the necessary sensors and components to achieve autonomous navigation. Designing multifunctional units and implementing them into the robot.
2.Model Testing: Putting robot models into physical simulation in order to test and preview how it works in reality. And modifying the parameters of the robot to make sure it functions well.
3.Construction of Gazebo Virtual Environment:Creating a simulated environment in Gazebo for the robot to operate within, which mimics real-world conditions for testing and development.
4.Verification of SLAM Algorithms:Implementing and validating SLAM (Simultaneous Localization and Mapping) algorithms such as gmapping and hector_slam to ensure accurate mapping and localization within the environment.
5.Full Coverage of the Environment: The robot was required to autonomously cover all open spaces within the room, ensuring no areas were left unexplored.
6.Obstacle Avoidance: The robot needed to detect and avoid both mapped and unmapped obstacles (such as small objects that might not appear in the initial SLAM-generated map).
7.Dynamic Response to Unseen Obstacles: If the robot encountered small obstacles that were not included in the initial SLAM map, it was expected to recognize and avoid them without deviating significantly from its planned path.

3.3 Challenges Faced:
During the implementation phase, several challenges arose that impacted the performance of the path planning and SLAM systems:
1.Inaccurate Initial SLAM Mapping : The initial SLAM map sometimes failed to accurately represent the environment, particularly with fine details. This inaccuracy led to errors in the path planning process, causing the robot to miss areas or collide with obstacles. 
2.Poor Algorithm Performance with Complex Obstacles : The full coverage path planning algorithm struggled when encountering complex or irregularly shaped obstacles. The robot would either spend too much time navigating around these obstacles or fail to fully cover areas near them. Improvements were made by tweaking the path planning algorithm to better handle these scenarios.
3.Correlation Between Downsampled Grid Map and Real-World Environment : Translating the downsampled grid map used by the planning algorithm into effective real-world navigation proved challenging. The robot sometimes misinterpreted the map, leading to less efficient coverage. By enhancing the mapping resolution and improving grid-to-world alignment, this issue was largely resolved.

4.Motion Planning and Control Methods
4.1 Overview of the Motion Planning Algorithm:
In our project, the primary motion planning strategy was based on the full coverage path planning algorithm. The goal was to ensure that the robot could cover the entire area of a room efficiently while avoiding obstacles. The algorithm we developed focused on generating an optimal sequence of target points that the robot needed to visit to achieve complete coverage. This sequence was calculated using a downsampled grid map, where each grid cell was approximately the size of the robot itself. The robot would start from any unvisited cell and systematically visit all unvisited cells, ensuring no area was left uncovered. The traversal was similar to a depth-first search (DFS) approach, where the robot would push the starting point onto a stack, visit the neighboring unvisited points, and continue until all points were visited and the robot returned to the starting position.
4.2 Control Methods for Movement and Task Execution:
For the actual navigation and movement control, we utilized the move_base package, a powerful ROS tool that integrates sensor data with the global map to achieve efficient point-to-point navigation. The move_base package handled real-time obstacle avoidance and path planning based on sensor feedback, allowing the robot to adapt to dynamic changes in the environment.
The process of full coverage was as follows:
1.Global Path Planning: The coverage algorithm first generated a set of target points based on the downsampled SLAM map. These points represented the critical locations the robot needed to visit for full coverage.
2.Sequential Navigation: The robot then sequentially navigated to each of these target points, with move_base managing the local path planning and real-time obstacle avoidance.
3.Obstacle Handling: If the robot encountered an obstacle during its journey to a target point, move_base adjusted the path dynamically, allowing the robot to bypass the obstacle and continue towards its goal.
4.Return to Start: After visiting all target points, the robot navigated back to its starting position, completing the full coverage cycle.

5.Programming/Coding Process
5.1 Code Development Structure and Main Script Descriptions:
The development of our project was structured around a set of ROS nodes and launch files, each serving specific functions to achieve autonomous navigation, SLAM, and motion planning. The key scripts and launch files include:
5.1.1 mbot_teleop.py
The mbot_teleop.py script is designed for controlling the movement of a differential drive robot using keyboard input. This Python script provides real-time control over the robot's linear and angular velocities, making it suitable for both testing and manual operation scenarios. It uses ROS (Robot Operating System) to publish velocity commands via the Twist message type. In summary, the mbot_teleop.py script offers a straightforward and effective method for controlling a differential drive robot via keyboard input. It provides real-time control with smooth speed adjustments, enhancing the operational experience. Future improvements could include support for additional input devices, more precise speed adjustments, or integrating visual inputs with movement control for more complex operation modes.
    
Figure 5.1 Partial Display of the mbot_teleop.py file

5.1.2 exploring_house.py 
The exploring_house.py file implements a robot autonomous navigation algorithm using the ROS framework within the Gazebo simulation environment. The program first reads and processes a map image to generate a grid map, which is then downsampled based on the robot's size. The algorithm performs a coverage path planning on the downsampled map and generates a series of target points. These target points are converted into world coordinates, and the ROS move_base package is used to navigate the robot to each target point sequentially through an Action client. During this process, the robot avoids obstacles and logs the runtime upon reaching each target. If the robot fails to reach a target point within a specified time, it skips the current target and moves on to the next one.

Figure 5.2 Partial Display of the exploring_house.py file

5.1.3 mbot_base.xacro
The mbot_base.xacro file is an XML-based configuration file used for defining the robot model in ROS (Robot Operating System). This file utilizes the Xacro (XML Macros) syntax to simplify and modularize the creation of URDF (Unified Robot Description Format) models. The mbot_base.xacro defines the structural and visual components of a differential drive robot, including its base, wheels, caster wheels, and various sensors.In summary, the mbot_base.xacro file is a comprehensive URDF model that provides a detailed description of the robot's physical structure and sensors. By using Xacro macros, it efficiently organizes and reuses definitions for various components, including the base, wheels, caster wheels, and sensors. This modular approach simplifies the robot modeling process and facilitates easy adjustments and extensions.
   
   
Figure 5.3 Partial Display of the mbot_base.xacro
5.1.4 mbot_gazebo.launch
The mbot_gazebo.launch file is designed to initiate a Gazebo simulation environment for the mbot robot. It includes launching the Gazebo world, loading the robot's model description, and starting essential ROS nodes like joint_state_publisher and robot_state_publisher to manage the robot's joint states and transformations. Additionally, it spawns the robot model within the Gazebo simulation, allowing for real-time interaction and testing of the robot's behavior.
   
Figure 5.4 Partial Display of the mbot_gazebo.launch
5.1.5 gmapping.launch
The gmapping.launch file is used to initialize the Gmapping SLAM (Simultaneous Localization and Mapping) algorithm in a simulated environment. It configures various parameters related to the robot's odometry, laser scan data, and SLAM performance. The launch file also remaps the laser scan topic and allows for the generation of a 2D occupancy grid map as the robot navigates through its environment.
  

Figure 5.5 Partial Display of the gmapping.launch

5.1.6 hector_slam.launch
The hector_slam.launch file is used to launch the Hector SLAM algorithm in a virtual environment. It initializes the hector_mapping node for SLAM processing, configuring parameters like map resolution, size, and update thresholds. Additionally, the file launches an rviz node for visualizing the generated map in real-time.
    
Figure 5.6  Partial Display of the hector_slam.launch

5.2 Key Algorithms and Their Implementation:
1.Full Coverage Path Planning Algorithm: This custom algorithm generates a sequence of target points that the robot needs to visit to cover the entire area. It operates on a downsampled grid map, where each cell represents an area equal to the robot's footprint. The algorithm follows a DFS-like approach, starting from an unvisited point, pushing it onto a stack, and continuing until all points are visited. This sequence of points is then fed to the move_base node for navigation.
2.DWA Local Planner: Configured within the move_base.launch file, the DWA local planner is a crucial component that calculates the optimal velocity commands for the robot to avoid obstacles while following the global path. It balances path adherence, speed, and obstacle avoidance, ensuring smooth and efficient navigation.
5.3 Software Tools:
●ROS Version: The project was developed using ROS Noetic, chosen for its compatibility with Ubuntu 20.04 and broad support for various robotic libraries.
5.4 Simulation Environment:
●Gazebo: The primary simulation environment, Gazebo was used to create a virtual world where the robot could be tested. We defined the environment using .world and .sdf files, which included the layout of rooms, walls, and obstacles. These files provided a realistic testing ground for our algorithms, closely mimicking the conditions the robot would face in a real-world scenario.
5.5 Problem-Solving in the Coding Process:

6.Learning Outcomes
6.1 Acquired Knowledge and Technical Skills:
Throughout the course of this project, team members gained extensive knowledge and hands-on experience in several key areas of robotics and software engineering:
1.ROS (Robot Operating System): We developed a strong understanding of ROS, including how to configure and launch nodes, manage topics, services, and parameters, and how to use various ROS tools like rviz for visualization and rosbag for data recording.
2.RVIZ Simulation: We learn how to build a brand new robot and simulate it in RVIZ by using URDF language and manually translate it into an XACRO language in order to run a simulation in Gazebo. It's important to note that since Gazebo only supports XACRO for physical simulations, while RVIZ only supports URDF, you'll need to complete the physical parameters, add Gazebo tags, configure the transmission devices, and include controller plugins to successfully run the simulation.
3.Gazebo Simulation: The team learned how to create and manipulate virtual environments in Gazebo, design robot models using URDF, and simulate realistic robotic behaviors, including navigation and sensor data processing.
4.SLAM (Simultaneous Localization and Mapping): We gained a deep understanding of SLAM algorithms, particularly Hector SLAM and Gmapping. This included learning how to implement and fine-tune these algorithms to create accurate maps of the robot's environment in real-time. Our experience with SLAM also extended to comparing the performance of different algorithms and understanding their strengths and limitations in various scenarios.
5.Motion Planning and Path Planning: 

6.2 Problem-Solving and Debugging Experience:
The project posed several challenges, which provided valuable problem-solving and debugging experience:
1.ROS Integration Issues: Early in the project, we encountered difficulties with integrating various ROS packages and ensuring consistent communication between nodes. We learned how to diagnose and resolve issues related to ROS topics, parameter configurations, and node synchronization.
2.Robot modeling issues: Learning how to use URDF is a completely new knowledge since we only know how to use Solidworks to build an already existed robot, which can not be run in RVIZ. While solving this problem, we need to apply not only what we learned at lecture 2 and 3, but also make ourselves accustomed to the IDE of URDF and how the "links" and "joints" fix to each other.
3.Translation of URDF into XACRO: XACRO is a advanced format of robot modeling language which is more concise and useful, but with more complexity. We compare URDF and XACRO, notise the difference between them and successfully translate our robot in URDF into XACRO.
4.Algorithm Optimization: We faced challenges in optimizing the full coverage path planning algorithm to minimize redundancy and improve efficiency. Through iterative testing and refinement, we developed strategies for enhancing the algorithm's performance, particularly in complex environments.
5.Simulation-to-Real-World Translation: Bridging the gap between simulation results in Gazebo and real-world applications was another significant challenge. We learned how to adjust simulation parameters, such as sensor noise and map resolution, to better reflect real-world conditions.

6.3 Achievements and Milestones:
1.Successful Implementation of Full Coverage Path Planning: The project achieved the goal of developing a fully functional path planning algorithm that ensures complete coverage of the environment while avoiding obstacles.
2.Integration of SLAM with Autonomous Navigation: We successfully integrated SLAM with the move_base package, allowing the robot to navigate dynamically within an environment, even as it simultaneously mapped that environment.
3.Simulation Environment Setup: We established a robust simulation environment in Gazebo that accurately represented real-world conditions, allowing us to test and validate our algorithms under various scenarios.

7.Conclusion and Future Work
7.1 Conclusion
●Verification of SLAM algorithm
1.Gmapping algorithm (2 Inputs: Data from Laser Scan and Odometry)
In this experiment, the GMapping algorithm successfully generated a 2D occupancy grid map of the testing environment. The map clearly outlines the main structures in the environment, including the room contours, walls, and some indoor obstacles. The GMapping algorithm effectively combined the laser scanner data with the robot's pose information, using probabilistic methods to continually optimize and refine the map, resulting in accurate mapping of the unknown environment.
The generated map shows well-defined boundaries of the walls and accurately reflects the overall structure of the rooms. This indicates that the coordination between the laser scanner and the GMapping algorithm was successful in perceiving and modeling the environment. The continuity of the laser scanner data and the localization accuracy of the GMapping algorithm were both effectively demonstrated in this mapping process.
However, the map also exhibits some discontinuous fan-shaped patterns, mainly in the peripheral areas of the robot's movement path. These discontinuities may have been caused by factors such as the scanning angle range of the laser scanner, the robot's movement speed, and the presence of complex obstacles in the environment. Despite these discontinuities, they did not significantly impact the overall completeness and accuracy of the map.
In conclusion, the GMapping algorithm demonstrated strong environmental perception and map-building capabilities in this experiment, producing a map that can be effectively used for subsequent robot navigation and path planning tasks. The results validate the practicality of the GMapping algorithm in indoor environments and provide a solid foundation for further research on autonomous robot navigation.

Figure 7.1 Gmapping Algorithm Map Construction Result


2.Hector SALM algorithm (Only need data from Laser Scan with timestamp for input)
The Hector SLAM algorithm demonstrates impressive mapping capabilities, particularly in open and spacious environments. It efficiently generates 2D maps that accurately capture the layout of walls, room structures, and other large features. The high level of detail in these areas highlights the algorithm's effectiveness in interpreting and mapping the surrounding space.
However, the generated map also reveals some limitations of the algorithm. Notably, there are discontinuous fan-shaped scanning artifacts that appear in the final map. These artifacts may result from rapid movements or sharp turns made by the robot during scanning, leading to incomplete data capture in certain areas. These discontinuities can affect the overall clarity and continuity of the map, potentially hindering its usefulness in some applications.
Furthermore, the Hector SLAM algorithm can encounter difficulties in narrow or complex environments. In such scenarios, the algorithm may struggle to maintain consistent accuracy, resulting in potential inconsistencies or missing details in the map. This limitation suggests that while Hector SLAM is highly effective in more open spaces, it may require complementary techniques or adjustments when applied to environments with intricate or tightly confined spaces.
Overall, while Hector SLAM provides strong performance in generating detailed maps, attention should be given to the specific environment in which it is deployed, as certain conditions may challenge its accuracy and completeness.


Figure 7.2 Hector Slam Algorithm Map Construction Result

●Coverage Path Planning

Creating the Grid
The grid is generated from a static map of the environment, which is constructed using the GMapping algorithm.
To optimize the planning process, we downsample the original high-resolution map to match the resolution of our grid.This downsampling reduces the number of grid cells while retaining essential environmental features, ensuring efficient path planning.


Figure 7.3 Downsample Result of Grid Map

Path Planning Algorithms                                                      1. Spanning Tree Algorithm
●Performance:Strengths: Effective in simple scenarios with quick computation times.
○Limitations: When faced with complex obstacles, the algorithm can lead to long, inefficient paths.
●Issues Encountered:Path Jumping: The algorithm may cause significant path jumps, particularly when nearby nodes have already been visited.
○Cause: The lack of available unvisited nodes in close proximity forces the algorithm to select distant nodes, resulting in inefficient routing.


2. Genetic Algorithm (GA)
●Performance:Strengths: Able to produce solutions in simple grid environments.
○Limitations: Computation is slow, and the quality of the resulting paths deteriorates as the complexity of the grid increases.
●Issues Encountered:Poor Path Quality: The routes generated are often suboptimal, particularly in more complex environments.
○Cause: The GA implementation likely lacks the necessary refinements to handle the specific challenges of full coverage path planning. Key aspects of the algorithm, such as fitness function design and crossover/mutation strategies, may not be fully optimized for this application.


3. Deep Q-Network (DQN)
●Performance:Strengths: DQN has the potential for learning complex navigation strategies, but this potential was not fully realized in this experiment.
○Limitations: The algorithm frequently fails to converge, often getting stuck without finding a valid solution.
●Issues Encountered:Stagnation: The DQN often gets stuck and fails to produce a valid path, especially in more challenging environments.
○Cause: The problem may stem from incorrect parameter settings or issues with the environment definition. These factors likely prevent the DQN from effectively learning and optimizing the navigation policy.


Simulation Results Summary
●Robot Speed:
○Observation: The robot’s movement speed during the simulation was relatively slow.
○Impact: This limits the efficiency of the path execution and overall task completion time.
○Possible Causes:Conservative path planning to avoid obstacles.
■Inherent limitations of the DQN-based controller in dynamic environments.
●Obstacle Avoidance Issues:
○Observation: The robot failed to avoid obstacles that were not detected by its sensors, especially those at different heights.
○Impact: The robot may collide with objects that are either too low or too high to be detected by its primary sensors, leading to potential damage or mission failure.





●Overall Summary
This project successfully demonstrated the design and implementation of a differential wheeled robot with autonomous navigation and mapping capabilities. By integrating various sensors and advanced algorithms, we were able to create a robot that could autonomously navigate and clean a simulated environment in Gazebo. The project met its primary objectives, including effective SLAM implementation, full coverage path planning, and real-time obstacle avoidance. The successful integration of these systems not only showcased the potential for developing intelligent, autonomous robots but also provided valuable insights into the complexities of robot navigation and control in dynamic environments.

7.2 Limitations of the Study
1.Simulated Environment: The project was conducted entirely in a simulated environment using Gazebo. While this allowed for controlled testing and experimentation, the results may not fully translate to real-world scenarios where unpredictable factors such as varying surface conditions, lighting, and sensor noise could impact the robot's performance.
2.Sensor Limitations: The sensors used in the simulation, such as the Lidar and Kinect camera, have predefined characteristics that might differ from their real-world counterparts. This could lead to discrepancies between the simulation results and actual performance in a physical environment.
3.Algorithmic Complexity: The full coverage path planning algorithm implemented in this study, while effective, is relatively basic and may not be optimal for more complex environments. Additionally, the project did not explore more advanced machine learning techniques that could enhance the robot's decision-making capabilities.

7.3 Future Work
1.Real-World Testing: Transitioning the robot from a simulated environment to a physical one would be a crucial next step. This would involve deploying the robot in real-world settings and refining the algorithms to handle the challenges posed by real environments.
2.Advanced Path Planning Algorithms: Future work could explore more sophisticated path planning algorithms, such as those utilizing machine learning or artificial intelligence, to optimize the robot's navigation and obstacle avoidance strategies, particularly in complex or dynamic environments.
3.Multi-Robot Systems: Expanding the project to include multiple robots working collaboratively could provide insights into distributed SLAM, multi-robot coordination, and cooperative coverage strategies, further pushing the boundaries of autonomous robotics.

8.Supporting Materials




Reference:
[1]《ROS机器人开发实践》
[2] http://wili.ros.org/naviagation
[3] https://github.com/koide3/slam_docker_collection
[4] http://t.csdnimg.cn/jlxZU

