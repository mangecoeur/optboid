Optimized Boids (2D)
====================

run with python engine.py

An optimized version of  C. Reynolds flocking simulation which uses "boids"
with simple rules to reproduce the behaviour of flocking creatures.

This uses a grid data structure which allows position information about boids
to be stored implicitly through their position in the grid. This makes it much
faster to look up the boids nearest neighbours which influence its movement.

Speedups of the order of 4x have been seen in some very approximate benchmarks,
using the pyglet-based opengl 2D canvas viewer

The nice thing about this approach is that the performance of normal boid 
algorithms is O(n^2) (N is the number of boids) while using a grid the performance
actually depends on "boid density" - the maximum number of boids within its 
influence range, which in turn depends on the minimum distance allowed between boids.

Essentially, the flocking algorithm itself is still O(n^2) but n is now the maximum number
of boids within the influence range rather than the total number of boids in the
simulation (people with more computer science could probably determine this much more
rigourously)