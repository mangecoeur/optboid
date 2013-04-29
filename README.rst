Optimized Boids (2D)
====================

There are two GUIs for the simulation. 
One based on PyGlet which must first be installed. Run this with

python glboid.py

The second is a curses (terminal) interface. Run with

python curseboid.py


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
rigourously).

The actual flocking algorithm is taken from the Processing examples, since it appeared
to be more stable than anything I came up with ;)


Some interesting observations:
The neighbour search function returns the boids' current grid cell if the search
radius is <= the search radius. However in the case the boid is near the edge of
the grid cell, this will leave out boids in the next grid cell which would normally
fall inside its "line of site". So it is possible to make sure the grid cells are
smaller than the search radius. In this case the search returns the current grid
cell and the 8 surrounding cells. This is quite fast thanks to the use of python's
collections.deque type, but it is still *significantly slower* than returning only
one cell. 

Empirical tests indicate that the simulation is just as convincing when
using the former, less accurate method.

*Pypy*
I feel this little program is a good test of a real-ish world use case for pypy.
It involves some commonly used bits of the standard lib, some slightly less common,
a significant external lib (Pyglet), and a simulation who's performance is measured
not so much in raw timings but in user experience.

I found pypy provided some spectacular performance improvements with this flocking sim.
With 400 boids, CPython acheived a barely-animated 10fps while munching one full CPU
core. Pypy managed to reach the 60fps cap and have a bit of CPU to spare (the CPU
in question is an middling core2duo in a 2010 macbook pro). However it was very
(painfully) clear to see the JIT spin up. For the first few seconds the framerate
chugged on at 3-5 then rapidly increasing to 60 as the runtime optimization kicked
 in. For game development this is important to consider, you may for instance want
to "spin up" you code in the background before showing anything to the user.
This also happens on repeat runs of unmodified code, suggesting pypy needs a better
way to "remember" it's JIT work across runs to skip doing it every time (like
CPython does with pyc files) 
