from __future__ import division

from math import atan2, sin, cos, floor, ceil
import random
from euclid import Vector2

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

    def add_boid(self,b):
        self.boids.append(b)
        self._insert(b)
        
    def _insert(self, b):
        c = self.find_cell_containing(b.x, b.y)
        c.append(b)
        
    def cell_num(self,x, y):
        """Forces units into border cells if they hit an edge"""
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

    def find_near(self, x, y, influence_range):
        """return objects within radius influence_range of point x,y"""
        if influence_range == 0:  
            _nearObjects = self.find_cell_containing(x,y)
        elif influence_range <= self.cell_width:
            _nearObjects = self.find_neighbour_cells(x,y)
        else: 
            ext = ceil(influence_range/self.cell_width)
            _nearObjects = self.find_extended(x,y,ext)
            
        return _nearObjects

    def find_neighbour_cells(self, x, y):
        return self.cell_table[self.cell_num(x,y)]
        
    def find_extended(self, x, y,  d):
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
            self._insert(b)

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
    
    # Get and set the position as a vector
    def _get_pos(self):
        return Vector2(self.x,self.y)
        
    def _set_pos(self,p):
        self.x = p.x
        self.y = p.y
        
    position = property(_get_pos,_set_pos)
    
    # Get and set the speed as a scalar
    def _get_speed(self):
        return abs(self.velocity)
    
    def _set_speed(self,s):
        if abs(self.velocity) == 0:
            velocity = Vector2(1,0)
        
        self.velocity.normalize()
        self.velocity *= s
        
    speed = property(_get_speed,_set_speed)
    
    #get and set the rotation as an angle
    def _get_rotation(self):
        return atan2(self.velocity.y,self.velocity.x)
    
    def _set_rotation(self,r):
        old_speed = self.speed
        #set the direction as a unit vector
        velocity.x = cos(r)
        velocity.y = sin(r)

        self.speed=old_speed 
    
    rotation = property(_get_rotation,_set_rotation)
    
    def __init__(self,x, y):
        """ create a new boid at x,y """
        self.x = x
        self.y = y
        self.acceleration = Vector2(0,0)
        self.velocity = Vector2(random.uniform(-self.max_speed, self.max_speed),
                                random.uniform(self.max_speed, self.max_speed))
    
    def __repr__(self):
        return 'id %d'%self.id

    def borders(self, top, bottom, left, right):
        """
        Assume a square field
        """
        if self.x < left:
            self.velocity += Vector2(1,0) * self.speed / 5
        if self.y < top:
            self.velocity += Vector2(0,1) * self.speed / 5
        if self.x > right: 
            self.velocity -=  Vector2(1,0) * self.speed / 5
        if self.y > bottom: 
            self.velocity -= Vector2(0,1) * self.speed / 5
    
    def update(self,t):
        """
        Method to update position by computing displacement from velocity and acceleration
        """
        self.velocity += self.acceleration * t *self.drag
        if self.speed > self.max_speed:
            self.velocity *= 0.9
        #limit(self.velocity,self.max_speed)
        self.position += self.velocity * t

    def interact(self, actors):
        """
        Unit-unit interaction method, combining a separation force, and velocity
        alignment force, and a cohesion force
        """
        separate_force = Vector2(0,0)
        align_force =  Vector2(0,0)
        cohesion_sum =  Vector2(0,0)
        cohesion_force = Vector2(0,0)
        
        count = 0

        for other in actors:
            #vector pointing from neighbors to self
            diff = self.position - other.position 
            d = abs(diff)

            #Only perform on "neighbor" actors, i.e. ones closer than arbitrary
            #dist or if the distance is not 0 (you are yourself)
            if d > 0 and d < self.influence_range:
                count = count+1
                
                diff.normalize()
                if d < self.minsep:
                    diff *= self.max_steering_force
                else:
                    diff /= d # Weight by distance
                
                cohesion_sum += other.position # Add position
                    
                separate_force += diff

                #Align - add the velocity of the neighbouring actors, then average
                align_force += other.velocity

        if count > 0:
            #calc the average of the separation vector
            separate_force /=count
            separate_force *= self.sep_strength
            
            #calc the average direction (normed avg velocity)
            align_force /=count
            align_force *= self.align_strength
            
            #calc the average position and calc steering vector towards it
            cohesion_sum /= count
            cohesion_force = self.steer(cohesion_sum,True)   
            cohesion_force *= self.cohesion_strength

            #finally add the velocities
            sum = separate_force + cohesion_force + align_force

            #smooth the acceleration var
            self.acceleration = 0.2*self.acceleration + sum
     
    def steer(self, desired, slowdown=False):
        """
        A helper method that calculates a steering vector towards a target
        If slowdown is true the steering force is reduced as it approaches the target
        """
        desired -= self.position
        d = abs(desired)
        #If the distance is greater than 0, calc steering (otherwise return zero vector)
        if d > 0:
            desired.normalize()
            if slowdown and (d < 30.0):
                desired *= self.max_steering_force*d/self.damp
            else:
                desired *= self.max_steering_force
                
            steer = desired - self.velocity
        else:
            steer = Vector2(0,0)
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
        
        for i in range(starting_units):
            b = Boid(random.uniform(100, 600), 
                     random.uniform(100, 600))
            self.swarm.add_boid(b)
        
        self._cumltime = 0 #calculation var
        
    def update(self,dt):
        """dt is in seconds"""
        for b in self.swarm.boids:
            #close_boids = self.swarm.find_near(b.x,b.y,b.influence_range)
            #b.interact(close_boids)
            b.interact(self.swarm.boids)
            b.update(dt) 
            w=self.field_size
            p=self.pad
            b.borders(p,w-p,p,w-p) #keep the boids inside the borders
        
        #rebuild the swarm once we've updated all the positions
        #self.swarm.rebuild() 
