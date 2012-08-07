from __future__ import division
from __future__ import print_function

from math import atan2, sin, cos, floor, ceil
import random
from euclid import Vector2
import numpy as np

"""
Optimization idea
Most of time is spent in interact operation
grid structure is pretty efficient, doesn't take much time now.
Want to use numpy to boost perf because quite a lot of time is spent
doing vector operations with the eucild module
Can do several things.
make vectors numpy vectors, use numpy funcs to add, multiply etc

could store pos, vel, acc of boids in their own numpy array- an array of vectors.
use "find near" to return a view of this array (arr[[...inds...]]) and apply 
operations to whole array. 

e.g. cohesion sum is simply sum of positions - v. fast  when pos are in 2xN
array, sum down columns.

Means stronger coupling between boids and swarm - swarm holds list of pos etc
as well as boid obj.

"""
def limit(vector,lim):
    """
    limit a vector to a given magnitude
    this is an 'in place' function, modifies the vector supplied.
    """
    if abs(vector) > lim:
        vector.normalize()
        vector *= lim
        
class BoidSwarm(object):
    """
    The grid to hold the boids
    """
    
    def __init__(self,width, cell_w):
        """
        Create data structure to hold the things. At the base is a table of cell nodes
        each containing an arbitrary number of units. Need whole number of cells, so cell width
        is the total width (for now in pixel units) divded by the divisions.
        The structure is always square. It can provide boundaries though good level design
        should mean you don't ever hit them.
        """
        
        self.boids = [] #list of all the boids
        
        divs = int(floor(width/cell_w))
        self.divisions = divs
        self.cell_width = cell_w
        
        self.cell_table = {}
        self.num_cells = divs*divs
        for i in range(divs):
            for j in range(divs):
                self.cell_table[(i,j)] = []                

    def add_boids(self,point_list):
        """
        add boids at the given locations.
        Do it this way so that we can maintain numpy arrays
        of positions, vel, acc, and boid objects
        """
        self.positions = np.asarray(point_list)
        num_boids = len(point_list)
        self.vels = np.zeros((num_boids,2))
        self.accels = np.zeros((num_boids,2))

        for i,p in enumerate(self.positions):
            v = self.vels[i]
            a = self.accels[i]
            self.boids.append( Boid(p,v,a,i))
        #self.boids = np.asarray(self.boids)
        self.rebuild()

    def get_neighbour_data(self, x, y, radius):
        indexes = self.find_near(x,y,radius)
        return self.positions[indexes],self.vels[indexes]

    def cell_num(self,x, y):
        """
        Find table cell containing position x,y
        Forces units into border cells if they hit an edge"""
        i = int(floor(x / self.cell_width))
        j = int(floor(y / self.cell_width))
        #deal with boundary conditions
        if i < 0: i=0
        if j < 0: j=0
        if i >= self.divisions: i = self.divisions-1 #zero indexing
        if j >= self.divisions: j = self.divisions-1 #zero indexing
        return (i,j)
    
    def find_cell_containing(self, x, y):
        """returns the cell containing a position x,y"""
        return self.cell_table[self.cell_num(x,y)]

    def find_near(self, x, y, radius):
        """return indexes of objects within radius influence_range of point x,y"""
        if radius == 0:
            nearObjects = self.find_cell_containing(x,y)
        elif radius <= self.cell_width:
            nearObjects = self.get_neighbour_cells(x,y)
        else: 
            ext = ceil(radius/self.cell_width)
            nearObjects = self.get_extended(x,y,ext)
            
        return nearObjects

    def get_neighbour_cells(self, x, y):
        return self.cell_table[self.cell_num(x,y)]
        
    def get_extended(self, x, y,  d):
        """
        use to find set of cells surrounding the cell containing position
        x,y
        """
        I,J = self.cell_num(x,y)
        d=int(d)
        group = []
        for i in range(I-d,I+d):
            for j in range(J-d,J+d):
                if (i,j) in self.cell_table:
                    group += self.cell_table[(i,j)] #list concat
        return group

    def rebuild(self): 
        for cell in self.cell_table.values():
            cell[:] = []
        for b in self.boids:
            c = self.find_cell_containing(b.position[0], b.position[1])
            c.append(b.id) #only store indexes in the table, not objects

class Boid(object):
    """
    Boids class
    """
    influence_range = 40
    minsep = 25.0
    damp = 90.0    
    max_force = 20.0
    max_speed = 100.0
    max_steering_force = 20.0
    drag = 0.9
    cohesion_strength = 1.0 
    align_strength = 1.0
    sep_strength = 1.0
    
    cohesion_strength *=  max_steering_force
    align_strength *=  max_steering_force
    sep_strength *= max_steering_force
    
    # Get and set x and y as alternative to pos vector
    # probably don't need/shudn't use this :S
    def _getx(self): return self.position[0]
    def _setx(self, x): self.position[0] = x
    x = property(_getx,_setx)

    def _gety(self): return self.position[1]
    def _sety(self, y): self.position[1] = y
    y = property(_gety,_sety)
    
    # Get and set the speed as a scalar
    def _get_speed(self):
        return np.sqrt(self.velocity.dot(self.velocity))
    
    def _set_speed(self,s):
        v = np.sqrt(self.velocity.dot(self.velocity))
        if v == 0:
            velocity = np.array([1,0])
        self.velocity /= v
        self.velocity *= s
        
    speed = property(_get_speed,_set_speed)
    
    #get and set the rotation as an angle
    def _get_rotation(self):
        return atan2(self.velocity[1],self.velocity[0])
    
    def _set_rotation(self,r):
        old_speed = self.speed
        #set the direction as a unit vector
        self.velocity[0] = cos(r)
        self.velocity[1] = sin(r)

        self.speed=old_speed 
    
    rotation = property(_get_rotation,_set_rotation)
    
    def __init__(self,pos,v,a, index):
        self.position = pos #init pos vector
        """ create a new boid at x,y """
        self.x = pos[0]
        self.y = pos[1]
        self.acceleration = a
        self.velocity = v #should point this to numpy array slice
        self.velocity[0] = random.uniform(-self.max_speed, self.max_speed)
        self.velocity[1] = random.uniform(-self.max_speed, self.max_speed)
        self.id = index

    
    def __repr__(self):
        return 'id %d'%self.id

    def borders(self, top, bottom, left, right):
        """
        Assume a square field
        """
        m = np.identity(2) 
        # 1 0
        # 0 1
        if self.position[0] < left:
            self.velocity += m[0,:] * self.speed / 5
        if self.position[1] < top:
            self.velocity +=  m[1,:] * self.speed / 5
        if self.position[0] > right:
            self.velocity -=   m[0,:] * self.speed / 5
        if self.position[1] > bottom:
            self.velocity -=  m[1,:] * self.speed / 5
    
    def update(self,t):
        """
        Method to update position by computing displacement from velocity and acceleration
        """
        self.velocity += self.acceleration * t *self.drag
        if self.speed > self.max_speed:
            self.velocity *= 0.9
        #limit(self.velocity,self.max_speed)
        self.position += self.velocity * t

    def interact(self, positions, velocities):
        """
        Unit-unit interaction method, combining a separation force, and velocity
        alignment force, and a cohesion force
        Accepts numpy arrays of positions and velocities of neighbouring boids
        We don't need to know anything else about the neighbours in this simple example
        For more complex systems, could also pass an array of neighbour boid objects to
        make use of their properties (e.g. friend/foe)
        """
        #using numpy to vectorize algorithm
        #Cohesion force. Need to get the "average" position of neighbourhood
        delta = self.position - positions #array of vector deltas
        dist = np.sum(np.abs(delta)**2,axis=-1)**(1./2)

        select = np.where((dist < self.influence_range) & (0<dist) )

        dist = dist[select]
        filtered_delta = delta[select]

        ds_new=dist[:,np.newaxis]
        filtered_delta /= ds_new #normalize each row vector
        #divide again by distance to weight force
        # otherwise, all diff magnitudes are equal to 1 (normalized)
        filtered_delta /= ds_new #select correct elements from diff and ds
        filtered_delta = np.clip(filtered_delta,0,self.max_steering_force)
        s=np.where(dist < self.minsep)
        if s[0].size != 0:
            filtered_delta[s] += self.max_steering_force

        filtered_positions = positions[select]
        filtered_velocities = velocities[select]
        count = filtered_positions.size
        if not count: return

        cohesion_sum = filtered_positions.sum(axis=0)
        cohesion_sum /= count
        #calc the average position and calc steering vector towards it
        cohesion_force = self.steer(cohesion_sum,True)
        cohesion_force *= self.cohesion_strength

        #calc the average of the separation vector
        separate_force = filtered_delta.sum(axis=0) / count
        separate_force *= self.sep_strength

        #calc the average direction (normed avg velocity)
        align_force =  filtered_velocities.sum(axis=0) /count
        align_force *= self.align_strength

        # add the forces
        sum = separate_force + cohesion_force + align_force

        #smooth the acceleration var
        self.acceleration = 0.02*self.acceleration + sum
    
    def steer(self, desired, slowdown=False):
        """
        A helper method that calculates a steering vector towards a target
        If slowdown is true the steering force is reduced as it approaches the target
        """
        desired -= self.position
        d = np.linalg.norm(desired)
        #If the distance is greater than 0, calc steering (otherwise return zero vector)
        if d > 0:
            desired /= d
            if slowdown and (d < 30.0):
                desired *= self.max_steering_force * d / self.damp
            else:
                desired *= self.max_steering_force
        #TODO: wtf?
            steer = desired - self.velocity
        else:
            steer = np.zeros(2)
        return steer

    def seek(self, target, m=1):
        self.acceleration += self.steer(target,False)
        self.acceleration = self.acceleration * m
    
 
class FlockSimulation(object):
    """
    Ties the BoidSwarm with the boids.
    boidswarm just holds the data, boids know how to interact with each
    other. This class keeps the two separated
    """
    
    def __init__(self,starting_units = 100, field_size = 800):
        """
        """
        self.swarm = BoidSwarm(field_size+2*40,Boid.influence_range+5)
        self.field_size = field_size
        self.pad = self.swarm.cell_width #use to keep boids inside the play field
        starting_points = []
        for i in range(starting_units):
            starting_points.append([random.uniform(100, 600), random.uniform(100, 600)])
        self.swarm.add_boids(starting_points)
        self.swarm.rebuild()

    def update(self,dt):
        """dt is in seconds"""
        for b in self.swarm.boids:
            positions, velocities = self.swarm.get_neighbour_data(b.position[0],b.position[1],b.influence_range)
            b.interact(positions, velocities)
            b.update(dt) 
            w=self.field_size
            p=self.pad
            b.borders(p,w-p,p,w-p) #keep the boids inside the borders
        
        #rebuild the swarm once we've updated all the positions
        self.swarm.rebuild() 
