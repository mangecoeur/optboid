#!/usr/bin/env python
#
# euclid graphics maths module
#
# Copyright (c) 2006 Alex Holkner
# Alex.Holkner@mail.google.com
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

"""
A simple pyglet engine to do some 2D rendering.
Used to display ebtutues at given positions
"""
from __future__ import division

from pyglet import *
from pyglet.gl import *
from math import *
import random
import optboids


class World(object):
    """
    world class with draw functions for entities
    """
    #create verticies in opengl for more efficient drawing
    verts = [0.5, 0.0,-0.5, -0.2,-0.5, 0.2]
    vertsGl = (GLfloat * len(verts))(*verts)
    
    def __init__(self, swarm):
        self.swarm = swarm
        self.ents = swarm.boids #this will point to a list of boids
        self.ent_size=15.0
        self.fps = clock.ClockDisplay()

    def draw_entity(self,ent):
        """ Draws a boid """
        glLoadIdentity()
        glTranslatef(ent.x, ent.y, 0.0)
        glRotatef(ent.rotation*180/pi, 0, 0, 1)
        glScalef(self.ent_size, self.ent_size, 1.0)
        glColor4f(0,0,0,0)
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointer(2, GL_FLOAT, 0, self.vertsGl)
        glDrawArrays(GL_TRIANGLES, 0, len(self.vertsGl) // 2)
        
    def draw_grid(self):
        cw=self.swarm.cell_width
        w=cw*self.swarm.divisions
        for i in range(self.swarm.divisions):
            xy = i*cw
            glLoadIdentity()
            glBegin(GL_LINES)
            glColor4f(1,1,1,0)
            glVertex2f(0,xy)
            glVertex2f(w,xy)
            glEnd()

            glBegin(GL_LINES)
            glColor4f(0.5,0.5,0.5,0)
            glVertex2f(xy,0)
            glVertex2f(xy,w)
            glEnd()
            
    def draw(self):
        glClearColor(1.0, 1.0, 1.0, 0.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        self.fps.draw()
        
        #self.draw_grid()
        for ent in self.ents:
            self.draw_entity(ent)

simulation = optboids.FlockSimulation(200)
world = World(simulation.swarm)

window = pyglet.window.Window(800, 750, vsync=True)    

@window.event
def on_draw():
    window.clear()
    world.draw()

def update(dt):
    simulation.update(dt)

def idle(dt):
    pass        

clock.schedule(update)  
clock.schedule(idle)

if __name__=='__main__':
    pyglet.app.run()  
