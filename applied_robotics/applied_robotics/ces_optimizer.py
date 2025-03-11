"""CES Optimizer for CAP6990 Applied Robotics Assignment_5

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 3/10/2025

This module defines a 'bubble generator' and convex problem solver constrained to fit the 
CES Optimization problem presented in class. The CESOptimizer takes in a path, inflates each
point to a bubble, then finds the minimal path through the bubbles.
"""

import math
import cvxpy as cp
import numpy as np
from shapely.geometry import Point, Polygon

class BubbleGenerator():
    '''
    Generates bubbles for path optimization
    Methods:
        inflate_obstacles: Expands obstacles
        generate_bubble: Creates a bubble
        translate_bubble: Adjusts small bubbles
        create_bubbles: Generates feasible bubbles
        get_path: Returns bubbles
    Attributes:
        P: Reference trajectory
        O: Obstacles
        Q: Bubble centers to radii
        rl: Min radius
        ru: Max radius
    '''
    def __init__(self, P, O, rl, ru):
        '''
        Initialize bubble generator
        '''
        self.P = P  # Reference trajectory
        self.O = O  # Obstacles
        self.Q = {}  # Bubble centers mapped to radii
        self.rl = rl  # Minimum bubble radius
        self.ru = ru  # Maximum bubble radius

    def inflate_obstacles(self):
        '''
        Expands obstacles for collision prevention
        '''
        size = 1.25  # Buffer size for obstacles
        self.O = [poly.buffer(size) for poly in self.O]

    def generate_bubble(self, point):
        '''
        Creates a bubble with radius from nearest obstacle
        args: point (x, y)
        return: bubble center, radius
        '''
        min_dist = float('inf')
        for obs in self.O:
            dist = obs.exterior.distance(Point(point))
            min_dist = min(min_dist, dist)
        radius = min(min_dist, self.ru)
        return point, radius  # Return bubble center and radius

    def translate_bubble(self, center, radius):
        '''
        Adjusts bubble center if radius is too small (ie too close to an obs)
        args: center (x, y), radius
        return: new center, adjusted radius
        '''
        if radius >= self.rl:
            return center, radius
        
        closest_obs = min(self.O, key=lambda obs: obs.exterior.distance(Point(center)))
        nearest_point = closest_obs.exterior.interpolate(closest_obs.exterior.project(Point(center)))
        
        direction = np.array(center) - np.array([nearest_point.x, nearest_point.y])
        norm = np.linalg.norm(direction)
        direction /= norm
        new_center = tuple(np.array(center) + direction * (self.rl - radius))
        return new_center, self.rl

    def create_bubbles(self):
        '''
        Generates bubbles ensuring min radius constraints
        '''
        self.inflate_obstacles()
        last_center, last_radius = None, None

        for point in self.P:
            point = point[:2]
            if last_center and np.linalg.norm(np.array(point) - np.array(last_center)) < 0.5 * last_radius:
                continue
            center, radius = self.generate_bubble(point)
            if radius < self.rl:
                center, radius = self.translate_bubble(center, radius)
                if last_center and np.linalg.norm(np.array(center) - np.array(last_center)) < 0.5 * last_radius:
                    continue
            self.Q[center] = radius
            last_center, last_radius = center, radius
            
    def get_path(self):
        '''
        Returns generated bubbles
        return: {center: radius}
        '''
        return self.Q

class CESOptimizer():
    '''
    Optimizes path smoothing using CES
    Methods:
        optimize: Solves optimization problem
        get_path: Returns optimized path
    Attributes:
        Q: Bubble centers
        d: Avg step distance
        obj_F: Smoothness objective
        constraints: Optimization constraints
    '''
    def __init__(self, path, obs, alpha_k, rmin, velocity, rl, ru):
        '''
        Initialize CES optimizer
        args: path, obs, alpha_k smoothness weight, rmin min radius, velocity, rl lower radius, ru upper radius
        '''
        self.alpha_k = alpha_k  # Smoothness weight
        self.rmin = rmin  # Minimum bubble radius
        self.v = velocity  # Velocity
        self.d = 0  # Step distance
        
        bubble_gen = BubbleGenerator(P=path, O=obs, rl=rl, ru=ru)
        bubble_gen.create_bubbles()
        bubbles = bubble_gen.get_path()
        n = len(bubbles)

        self.Q = cp.Variable((n, 2)) 

        P = np.array(list(bubbles.keys()))

        start, goal = P[0], P[-1]

        v1 = (P[1] - P[0]) / np.linalg.norm(P[1] - P[0])
        vn = (P[-1] - P[-2]) / np.linalg.norm(P[-1] - P[-2])

        for k in range(n-1):
            self.d += np.linalg.norm(P[k+1] - P[k]) / (n-1)

        #objective function
        self.obj_F = cp.Minimize(sum(cp.norm(2*self.Q[k] - self.Q[k-1] - self.Q[k-2])**2 for k in range(1, n-1)))
        
        self.constraints = []

        #start point constraints
        self.constraints.append(self.Q[0] == start)
        self.constraints.append(self.Q[1] == start + self.d * v1)

        #end point constraints
        self.constraints.append(self.Q[-1] == goal)
        self.constraints.append(self.Q[-2] == goal - self.d * vn)

        #Bubble constraint
        for k in range(2, n-1):
            self.constraints.append(cp.norm(self.Q[k] - P[k]) <= bubbles[tuple(P[k])])

        #Smoothing constraint
        for k in range(1, n-1):
            expression = cp.norm(2*self.Q[k] - self.Q[k-1] - self.Q[k+1])
            self.constraints.append(expression <= min(self.d**2 / self.rmin, self.alpha_k * self.d / self.v))

    def optimize(self):
        '''
        Solves CES optimization problem, checks feasibility
        '''
        prob = cp.Problem(self.obj_F, self.constraints)
        prob.solve()

    def get_path(self):
        '''
        Returns optimized path
        return: array of points
        '''
        return np.array(self.Q.value)
