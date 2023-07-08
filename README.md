# 34757-Unmanned-Autonomous-Systems
Drone and Ship Control for the DTU June Course 34757 Unmanned Autonomous Systems 

#### Motivation
The course was laid out to facilitate the option of either choosing a regular track or a Side-Track for the final project in order to be able to pass the course. The regular track (or R-track) was chosen by students that would prefer working on a Crazyflie drone to develop path-planning algorithms so as to navigate around a maze and through a bunch of hoops.
On the other hand, students could choose to pursue a Side-track project. 

Our team decided to undertake the Side-Track project-5 titled USV Navigation and Control. The idea was to develop a controller for a prototype of the ship with the aim of maneuvering along complex trajectories while being positioned in accordance with when and where the drone swarms, responsible for detecting suspicious objects from target vessels, wish to deposit them. However, owing to time constraints, the team decided to work on a simple controller that facilitated movement of the USV in some standard trajectories like going around in a square, going straight for some distance, turning and then retracing its path etc. Hence, the objectives of the project were revised to be:
− Formulate a mathematical model to represent the Unmanned Surface vehicle so as to ease the development of a control architecture for its navigation.
− Develop a direction controller(i.e. a P/PI/PID controller) in SIMULINK using the model developed above.
− Implement the above model on the actual USV prototype used for testing, after it successfully runs in SIMULINK.
− Enable communication between the USV client and the Opti-Track server in ASTA such that a stream of absolute position and orientation data can be received as feedback to the position controller developed.
