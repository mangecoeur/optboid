#
# This is where the fun stuff happens - simulating the boids
# 
# This file includes Vector classes borrowed from Cocos2D
#
# This library is free software; you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as published by the
# Free Software Foundation; either version 2.1 of the License, or (at your
# option) any later version.
#
# This library is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License
# for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this library; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

from __future__ import division, print_function, absolute_import, unicode_literals

import math
import operator
import types
from math import atan2, sin, cos, floor, ceil
import random
import collections


def limit(vector, lim):
    """
    limit a vector to a given magnitude
    this is an 'in place' function, modifies the vector supplied.
    """
    if abs(vector) > lim:
        vector.normalize()
        vector *= lim


# Some magic here.  If _use_slots is True, the classes will derive from
# object and will define a __slots__ class variable.  If _use_slots is
# False, classes will be old-style and will not define __slots__.
#
# _use_slots = True:   Memory efficient, probably faster in future versions
#                      of Python, "better".
# _use_slots = False:  Ordinary classes, much faster than slots in current
#                      versions of Python (2.4 and 2.5).
_use_slots = True

# If True, allows components of Vector2 and Vector3 to be set via swizzling;
# e.g.  v.xyz = (1, 2, 3).  This is much, much slower than the more verbose
# v.x = 1; v.y = 2; v.z = 3,  and slows down ordinary element setting as
# well.  Recommended setting is False.
_enable_swizzle_set = False

# Requires class to derive from object.
if _enable_swizzle_set:
    _use_slots = True


# Implement _use_slots magic.
class _EuclidMetaclass(type):
    def __new__(cls, name, bases, dct):
        if _use_slots:
            return type.__new__(cls, name, bases + (object,), dct)
        else:
            del dct['__slots__']
            return types.ClassType.__new__(types.ClassType, name, bases, dct)
__metaclass__ = _EuclidMetaclass


class Vector2(object):
    __slots__ = ['x', 'y']

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __copy__(self):
        return self.__class__(self.x, self.y)

    copy = __copy__

    def __repr__(self):
        return 'Vector2(%.4f, %.4f)' % (self.x, self.y)

    def __eq__(self, other):
        if isinstance(other, Vector2):
            return self.x == other.x and self.y == other.y
        else:
            assert hasattr(other, '__len__') and len(other) == 2
            return self.x == other[0] and self.y == other[1]

    def __neq__(self, other):
        return not self.__eq__(other)

    def __nonzero__(self):
        return self.x != 0 or self.y != 0

    def __len__(self):
        return 2

    def __getitem__(self, key):
        return (self.x, self.y)[key]

    def __setitem__(self, key, value):
        l = [self.x, self.y]
        l[key] = value
        self.x, self.y = l

    def __iter__(self):
        return iter((self.x, self.y))

    def __getattr__(self, name):
        try:
            return tuple([(self.x, self.y)['xy'.index(c)] for c in name])
        except ValueError:
            raise AttributeError(name)

    if _enable_swizzle_set:
        # This has detrimental performance on ordinary setattr as well
        # if enabled
        def __setattr__(self, name, value):
            if len(name) == 1:
                object.__setattr__(self, name, value)
            else:
                try:
                    l = [self.x, self.y]
                    for c, v in map(None, name, value):
                        l['xy'.index(c)] = v
                    self.x, self.y = l
                except ValueError:
                    raise AttributeError(name)

    def __add__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x + other.x,
                           self.y + other.y)
        else:
            assert hasattr(other, '__len__') and len(other) == 2
            return Vector2(self.x + other[0],
                           self.y + other[1])
    __radd__ = __add__

    def __iadd__(self, other):
        if isinstance(other, Vector2):
            self.x += other.x
            self.y += other.y
        else:
            self.x += other[0]
            self.y += other[1]
        return self

    def __sub__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x - other.x,
                           self.y - other.y)
        else:
            assert hasattr(other, '__len__') and len(other) == 2
            return Vector2(self.x - other[0],
                           self.y - other[1])

    def __rsub__(self, other):
        if isinstance(other, Vector2):
            return Vector2(other.x - self.x,
                           other.y - self.y)
        else:
            assert hasattr(other, '__len__') and len(other) == 2
            return Vector2(other.x - self[0],
                           other.y - self[1])

    def __mul__(self, other):
        assert type(other) in (int, long, float)
        return Vector2(self.x * other,
                       self.y * other)

    __rmul__ = __mul__

    def __imul__(self, other):
        assert type(other) in (int, long, float)
        self.x *= other
        self.y *= other
        return self

    def __div__(self, other):
        assert type(other) in (int, long, float)
        return Vector2(operator.div(self.x, other),
                       operator.div(self.y, other))

    def __rdiv__(self, other):
        assert type(other) in (int, long, float)
        return Vector2(operator.div(other, self.x),
                       operator.div(other, self.y))

    def __floordiv__(self, other):
        assert type(other) in (int, long, float)
        return Vector2(operator.floordiv(self.x, other),
                       operator.floordiv(self.y, other))

    def __rfloordiv__(self, other):
        assert type(other) in (int, long, float)
        return Vector2(operator.floordiv(other, self.x),
                       operator.floordiv(other, self.y))

    def __truediv__(self, other):
        assert type(other) in (int, long, float)
        return Vector2(operator.truediv(self.x, other),
                       operator.truediv(self.y, other))

    def __rtruediv__(self, other):
        assert type(other) in (int, long, float)
        return Vector2(operator.truediv(other, self.x),
                       operator.truediv(other, self.y))

    def __neg__(self):
        return Vector2(-self.x, -self.y)

    __pos__ = __copy__

    def __abs__(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    magnitude = __abs__

    def clear(self):
        self.x = 0
        self.y = 0

    def magnitude_squared(self):
        return self.x ** 2 + self.y ** 2

    def normalize(self):
        d = self.magnitude()
        if d:
            self.x /= d
            self.y /= d
        return self

    def normalized(self):
        d = self.magnitude()
        if d:
            return Vector2(self.x / d, self.y / d)
        return self.copy()

    def dot(self, other):
        assert isinstance(other, Vector2)
        return self.x * other.x + self.y * other.y

    def cross(self):
        return Vector2(self.y, -self.x)

    def reflect(self, normal):
        # assume normal is normalized
        assert isinstance(normal, Vector2)
        d = 2 * (self.x * normal.x + self.y * normal.y)
        return Vector2(self.x - d * normal.x, self.y - d * normal.y)


class BoidSwarm(object):

    """
    The grid to hold the boids
    """

    def __init__(self, width, cell_w):
        """
        Create data structure to hold the things. At the base is a table of cell nodes
        each containing an arbitrary number of units. Need whole number of cells, so cell width
        is the total width (for now in pixel units) divded by the divisions.
        The structure is always square. It can provide boundaries though good level design
        should mean you don't ever hit them.
        """

        self.boids = []  # list of all the boids

        divs = int(floor(width/cell_w))
        self.divisions = divs
        self.cell_width = cell_w

        self.cell_table = {}
        self.num_cells = divs*divs
        for i in range(divs):
            for j in range(divs):
                # use deque for fast appends of several deque
                self.cell_table[(i, j)] = collections.deque()

    def cell_num(self, x, y):
        """Forces units into border cells if they hit an edge"""
        i = int(floor(x / self.cell_width))
        j = int(floor(y / self.cell_width))
        # deal with boundary conditions
        if i < 0:
            i = 0
        if j < 0:
            j = 0
        if i >= self.divisions:
            i = self.divisions-1  # zero indexing
        if j >= self.divisions:
            j = self.divisions-1  # zero indexing
        return (i, j)

    def find_cell_containing(self, x, y):
        """returns the cell containing a position x,y"""
        return self.cell_table[self.cell_num(x, y)]

    def find_near(self, x, y, influence_range):
        """return objects within radius influence_range of point x,y"""
        if influence_range == 0:
            _nearObjects = self.find_cell_containing(x, y)
        elif influence_range <= self.cell_width:
            _nearObjects = self.find_neighbour_cells(x, y)
        else:
            ext = ceil(influence_range/self.cell_width)
            _nearObjects = self.find_extended(x, y, ext)

        return _nearObjects

    def find_neighbour_cells(self, x, y):
        return self.cell_table[self.cell_num(x, y)]

    def find_extended(self, x, y, d):
        """
        use to find set of cells surrounding the cell containing position x,y
        """
        I, J = self.cell_num(x, y)
        d = int(d)
        group = collections.deque()
        for i in range(I-d, I+d):
            for j in range(J-d, J+d):
                if (i, j) in self.cell_table:
                    group.extend(self.cell_table[(i, j)])  # merge deque
        return group

    def rebuild(self):
        for cell in self.cell_table.values():
            cell.clear()
        for b in self.boids:
            c = self.find_cell_containing(b.position.x, b.position.y)
            c.append(b)


class Boid(object):

    """
    Boids class
    """
    influence_range = 90
    minsep = 25.0
    max_force = 20.0
    max_speed = 80.0
    drag = 0.9
    cohesion_strength = 1.0
    align_strength = 1.0
    sep_strength = 1.5

    # cohesion_strength *=  max_force
    # align_strength *=  max_force
    # sep_strength *= max_force

    # Get and set the speed as a scalar
    def _get_speed(self):
        return abs(self.velocity)

    def _set_speed(self, s):
        if abs(self.velocity) == 0:
            self.velocity = Vector2(1, 0)

        self.velocity.normalize()
        self.velocity *= s

    speed = property(_get_speed, _set_speed)

    # get and set the rotation as an angle
    def _get_rotation(self):
        return atan2(self.velocity.y, self.velocity.x)

    def _set_rotation(self, r):
        old_speed = self.speed
        # set the direction as a unit vector
        self.velocity.x = cos(r)
        self.velocity.y = sin(r)

        self.speed = old_speed

    rotation = property(_get_rotation, _set_rotation)

    def __init__(self, x, y):
        """ create a new boid at x,y """
        self.neighbors = 0

        self.position = Vector2(x, y)
        self.acceleration = Vector2(0, 0)
        self.velocity = Vector2(random.uniform(-self.max_speed, self.max_speed),
                                random.uniform(-self.max_speed, self.max_speed))

    def __repr__(self):
        return 'id %d' % self.id

    def borders(self, top, bottom, left, right):
        """
        Assume a square field
        if (self.position.x < left and self.velocity.x < 0):
            self.velocity.x = self.max_speed
        if (self.position.x > right and self.velocity.x > 0):
            self.velocity.x = - self.max_speed
        if (self.position.y < top and self.velocity.y < 0):
            self.velocity.y = self.max_speed
        if (self.position.y > bottom and self.velocity.y > 0):
            self.velocity.y = - self.max_speed
        """
        if self.position.x < left:
            self.position.x = right
        if self.position.x > right:
            self.position.x = left
        if self.position.y < top:
            self.position.y = bottom
        if self.position.y > bottom:
            self.position.y = top

    def update(self, t):
        """
        Method to update position by computing displacement from velocity and acceleration
        """
        self.velocity += self.acceleration * t
        limit(self.velocity, self.max_speed)
        self.position += self.velocity * t

    # Calculation variables for interact method - init once instead of on each call
    _sep_f = Vector2(0, 0)
    _align_f = Vector2(0, 0)
    _cohes_sum = Vector2(0, 0)

    def interact(self, actors):
        """
        Unit-unit interaction method, combining a separation force, and velocity
        alignment force, and a cohesion force.

        Many examples separate these into different functions for clarity
        but combining them means we need fewer loops over the neibor list
        """

        self._sep_f.clear()
        self._align_f.clear()
        self._cohes_sum.clear()

        count = 0
        self.neighbors = len(actors)

        for other in actors:
            # vector pointing from neighbors to self
            diff = self.position - other.position
            d = abs(diff)

            # Only perform on "neighbor" actors, i.e. ones closer than arbitrary
            # dist or if the distance is not 0 (you are yourself)
            if 0 < d < self.influence_range:
                count += 1

                diff.normalize()
                if d < self.minsep:
#                    diff *= self.max_force
#                else:
                    diff /= d  # Weight by distance
                self._sep_f += diff

                self._cohes_sum += other.position  # Add position

                # Align - add the velocity of the neighbouring actors, then average
                self._align_f += other.velocity

        if count > 0:
            # calc the average of the separation vector
            # self._sep_f /=count don't div by count if normalizing anyway!
            self._sep_f.normalize()
            self._sep_f *= self.max_speed
            self._sep_f -= self.velocity
            limit(self._sep_f, self.max_force)

            # calc the average direction (normed avg velocity)
            # self._align_f /= count
            self._align_f.normalize()
            self._align_f *= self.max_speed
            self._align_f -= self.velocity
            limit(self._align_f, self.max_force)

            # calc the average position and calc steering vector towards it
            self._cohes_sum /= count
            cohesion_f = self.steer(self._cohes_sum, True)

            self._sep_f *= self.sep_strength
            self._align_f *= self.align_strength
            cohesion_f *= self.cohesion_strength

            # finally add the velocities
            sum = self._sep_f + cohesion_f + self._align_f

            self.acceleration = sum

    def steer(self, desired, slowdown=False):
        """
        A helper method that calculates a steering vector towards a target
        If slowdown is true the steering force is reduced as it approaches the target
        """
        desired -= self.position
        d = abs(desired)
        # If the distance is greater than 0, calc steering (otherwise return zero vector)
        if d > 0:
            desired.normalize()
            if slowdown and (d < self.minsep):
                desired *= self.max_speed*d / self.minsep
            else:
                desired *= self.max_speed

            steer = desired - self.velocity
            limit(steer, self.max_force)
        else:
            steer = Vector2(0, 0)
        return steer


class FlockSimulation(object):

    """
    Ties the BoidSwarm with the boids.
    boidswarm just holds the data, boids know how to interact with each
    other. This class keeps the two separated
    """

    def __init__(self, starting_units=100, field_size=800):
        """
        """
        self.swarm = BoidSwarm(field_size+2*40, Boid.influence_range+5)  # /2
        self.field_size = field_size
        self.pad = 40  # use to keep boids inside the play field

        for i in range(starting_units):
            b = Boid(random.uniform(100, 400),
                     random.uniform(100, 400))
            self.swarm.boids.append(b)
        self.swarm.rebuild()
        self._cumltime = 0  # calculation var

    def update(self, dt):
        """dt is in seconds"""
        for b in self.swarm.boids:
            close_boids = self.swarm.find_near(b.position.x, b.position.y, b.influence_range)
            b.interact(close_boids)
            b.update(dt)
            w = self.field_size
            p = self.pad
            b.borders(p, w-p, p, w-p)  # keep the boids inside the borders

        # rebuild the swarm once we've updated all the positions
        self.swarm.rebuild()
