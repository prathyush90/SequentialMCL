# SequentialMCL

This project tries to estimate the state vector of redball w.r.t blue ball ground truth. The state vector can be expressed as [x, y, orientation]T .This can be applied for three dimensional case also.Blue ball can be moved with up,right,left and w keys w will add random direction to the blue ball, r will reset the blue ball to handle robot kidnapping problem . The particles will still be able to localize themselves.

This project is built with inspiration from the following lecture https://www.youtube.com/watch?v=3muh7tAlV2Q

To run the project 
1) clone the repository
2) run python3 rendere.py

Uses pygame to render the balls and particles onto the screen.

When r is pressed the blue ball is transported to some random place on the screen, from there the redball will predict the pose  in few steps.
