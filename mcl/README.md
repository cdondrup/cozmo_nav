## Monte Carlo Localisation for cozmo

This package provides a simple monte calro localisation for the cozmo based on orb slam2.
A particle filter is used to track the current position of the robot and updated based 
on its odometry and camera pose as caluclated by the robot's wheel encoders and orb slam,
respectively.

Particle filter pseudo-code:

```
Initiliase filter based on current orb slam position
Whenever a new odometry message is received, update the constant velocity model
Whenever a new orb slam pose is received, update the weights and resample
Loop:
  Predict new particle position based on constant velocity model
```

### Initialising the particles

To start the MCL, the particles need to be initialised with a position in the map.
We do this by waiting for the first message from orb slam to be published after the 
node has been started. Once this message is received, we create a Gaussian (or Normal)
distribution in x and y direction and sample particles reandomly. Addtionally, we also
create a 1D Gaussian distribution for the rotation angle theta around the z-axis and
sample new values from it.


### Constant velocity model

In orsder to be able to predict where the robot will move inbetween receiving message
from the odomotery and orb slam, we assume a constant velocity. Whnever a new odometry
message is received, the constant velocity model is updated based on the current and 
the previous position according to the odometry.

### Predicting particle movement

The main prediction loop runs at 30hz (3 times the speed of the odometry and orb slam)
and uses the constant velocity model to predict the movement of the particles between
messages. This increased publishing rate allows for faster position updates and supports
a more reactive navigation.

### Making observations and resampling particles

Whenever a new orb slam pose is received, a list of weights for the particles is calculated
based on the distance of the particle to the observed orb slam pose. A Gaussian distribution
is used to create the values for the weights biasing them to be clustered closer to the 
observed pose. Once the weights are updated, new particles are drawn with replacement based 
on those weights. To prevent particle "starvation" where the particle cloud condenses
to a few particles and, therefore, becomes worse at generalising and dealing with abnormal
situation, we use a so-called starvation factor that defines the percentage of particles
that are drawn based on the calculated weights. The remainder of the particles is drawn 
randomly like during intialisation but based on the current position and not the orb slam position.



