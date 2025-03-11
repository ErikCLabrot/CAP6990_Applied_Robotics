"""ces_test for CAP6990 Applied Robotics Assignment_5

Author: Erik C. LaBrot
Email: ecb25@students.uwf.edu
Date: 3/10/2025

This module defines a test main function that allows for the parameters of the 
CES optimization algorithm to be tested with and plotted without ROS integration
"""


from ces_optimizer import CESOptimizer, BubbleGenerator
from shapely.geometry import Polygon
import matplotlib.pyplot as plt

def create_obstacles(centers, size):
    obstacle_width = size/2
    # Convert obstacle centers into 1m square polygons
    obstacles = [
        Polygon([
            (x - obstacle_width, y - obstacle_width), (x + obstacle_width, y - obstacle_width), 
            (x + obstacle_width, y + obstacle_width), (x - obstacle_width, y + obstacle_width)
        ]) 
        for x, y in centers
    ]
    return obstacles

def plot_path(raw_path, optimized_path, obs):
    fig, ax = plt.subplots()

    #Plot obstacles
    for obs in obs:
        x,y = obs.exterior.xy
        ax.fill(x,y,color='red')

    # Plot original path
    path_x, path_y = zip(*raw_path)
    ax.plot(path_x, path_y, 'r--o', label="Original Path")

    #Plot optimized path
    op_x, op_y = optimized_path[:, 0], optimized_path[:, 1]
    ax.plot(op_x, op_y, 'b-o', label="Optimized Path")
    
    ax.legend()
    ax.grid(True)
    plt.show()


def main():

    #Shapely Obstacles created from center points
    O_centers = [(5,5)]
    O_size = 5
    O = create_obstacles(O_centers, O_size)

    #Test Path
    P = [
    (0.0, 0.0), (0.0, 0.75), (0.0, 1.5), (0.0, 2.25), (0.0, 3.0),
    (0.0, 3.75), (0.0, 4.5), (0.0, 5.25), (0.0, 6.0), (0.0, 6.75),
    (0.0, 7.5), (0.0, 8.25), (0.0, 9.0), (0.0, 9.75), (0.0, 10.5),
    (0.75, 10.0), (1.5, 10.0), (2.25, 10.0), (3.0, 10.0), (3.75, 10.0),
    (4.5, 10.0), (5.25, 10.0), (6.0, 10.0), (6.75, 10.0), (7.5, 10.0),
    (8.25, 10.0), (9.0, 10.0), (9.75, 10.0), (10.5, 10.0)
    ]

    #CES Algorithm Parameters
    alpha_k = 10.1
    r_min = 5.0
    velocity = 1.0

    rl = 0.1
    ru = 3

    ces_optimizer = CESOptimizer(path = P, obs = O, alpha_k = alpha_k, rmin = r_min, velocity = velocity, rl=rl, ru=ru) 
    ces_optimizer.optimize()
    optimized_path = ces_optimizer.get_path()

    plot_path(P, optimized_path, O)

if __name__ == "__main__":
    main()