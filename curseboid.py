#!/usr/bin/python
# -*- coding: utf8 -*-
from __future__ import (division, print_function,
                        absolute_import)

import curses
import simulation
import math
from time import time


class World(object):
    """
    world class with draw functions for entities
    """
    def __init__(self, swarm, width):
        self.width = width
        self.swarm = swarm
        self.ents = swarm.boids  # this will point to a list of boids
        self.ent_char = '^'
        self.num_ents = len(swarm.boids)
        #self.fps = clock.ClockDisplay()
        self._basescr = curses.initscr()
        self.scr = curses.newwin(80, 80, 0, 0)
        self.scr.border(0)
        self.scr.refresh()

        self._update_time = 0
        self._fps = 0

    def draw_entity(self, e):
        """ Draws a boid """
        x = int(math.floor(e.position.x / self.width))
        y = int(math.floor(e.position.y / self.width))
        self.scr.addstr(x, y, self.ent_char)

    def draw_fps(self, dt):
        self._update_time += dt
        if self._update_time > 0.5:
            self._fps = 1 / dt
            self._update_time = 0

        self.scr.addstr(3, 3, '%.1f fps' % self._fps)
        self.scr.refresh()

    def draw(self, dt):
        self.scr.clear()
        #self.draw_grid()
        for ent in self.ents:
            self.draw_entity(ent)
        self.draw_fps(dt)
        self.scr.refresh()


sim = simulation.FlockSimulation(100, 700)
world = World(sim.swarm, 10)


def run():
    go = True
    prev_time = time()
    print('starting')
    try:
        while go:
            now_time = time()
            dt = now_time - prev_time
            prev_time = now_time
            sim.update(dt)
            world.draw(dt)
            #x = world.scr.getkey()
            #if x == 'e':
            #    go = False
    except KeyboardInterrupt:
        curses.endwin()


if __name__ == '__main__':
    run()
