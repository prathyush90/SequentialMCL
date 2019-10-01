# SequentialMCL

This project tries to estimate the state vector of redball w.r.t blue ball(ground truth). The state vector can be expressed as [x, y, orientation]T .This can be applied for three dimensional case also. The same logic can be applied for sensor measurements with map of a place instead of relying on the blue ball's pose as the ground truth value for weight updates of particle filter. Blue ball can be moved with up,right,left and w keys. W will add random direction to the blue ball, R will reset the pose of the blue ball to random location(to test robot kidnapping problem) . The particles will localize themselves accordingly.

This project is built with inspiration from the following lecture https://www.youtube.com/watch?v=3muh7tAlV2Q

To run the project 
1) clone the repository
2) run python3 renderer.py

Uses pygame to render the balls and particles onto the screen.

When r is pressed the blue ball is transported to some random location on the screen, from there the redball will predict the pose  in few steps. You can read more about robot Kidnapping from here https://en.wikipedia.org/wiki/Kidnapped_robot_problem


Green Circles represent particles. Blue ball the actual location and red ball the predicted location from the set of the particles. The sticks on blue ball points to the actual heading direction and the stick on the red ball points to the predicted heading direction


Find the screenshots attached below.

1) Initialization step

![Initialization](https://github.com/prathyush90/SequentialMCL/blob/master/Images/Init.png)

2) Before Convergence


![Before Convergence](https://github.com/prathyush90/SequentialMCL/blob/master/Images/beforeconverged.png)

3) After Convergence


![After Convergence](https://github.com/prathyush90/SequentialMCL/blob/master/Images/converged.png)

4) Robot Kidnapped


![kidnapped](https://github.com/prathyush90/SequentialMCL/blob/master/Images/robot_kidnapped.png)

3) Converging after Kidnap


![afterkidnap](https://github.com/prathyush90/SequentialMCL/blob/master/Images/convergingfrom_kidnap.png)
