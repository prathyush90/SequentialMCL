# ParticleFilterOrientationEstimation

Particle filter's are  amazing for localization in general. But i was facing a problem when working on indoor positioning because mobile phone's sensors are not reliable and 
the way the user holds the phone matters. Handling all the cases was getting difficult to manage. So i built this with the idea that
the Principal Component axis on particles location can give details about the user orientation.The first axis must be dependent on the distance of robot and particle
so first component didn't work well while simulating, but the second component of PCA worked really good. This can be used for calculating user orientation irrespective of the mobile orientation or position held if the map of the area is given or if there is any information of known landmarks.

Steps to run the code:

1) git clone the repository
2) python renderer.py

This is just a naive idea, would love to hear some feedback
# SequentialMCL
