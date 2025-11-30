"""
Example script to visualize a motion plan of a car using Meshcat

Usage: car_vis.py <env.yaml> <plan.yaml>
"""

import argparse
import yaml
import time
import numpy as np

# visualization related
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('tree', help='input file with tree')
    args = parser.parse_args()

    with open(args.tree, "r") as f:
        lines = f.readlines()

    n_vertices = int(lines[0])
    vertices = np.loadtxt(lines[1:n_vertices+1], dtype=np.float32)
    #3d or 2d
    #vertices[:,2]=0
    edges = np.loadtxt(lines[n_vertices+1:], dtype=int)
    if edges.ndim == 1:
        edges = edges.reshape(1, 2)

    tree= []
    for i, j in edges:
        tree.append(vertices[i])
        tree.append(vertices[j])
    tree = np.array(tree)
    vis = meshcat.Visualizer().open()
    vis.set_object(g.LineSegments(
        g.PointsGeometry(tree.T,color=tree.T),
        g.PointsMaterial(1)
    ))

    input("Press Enter to exit and close the visualizer...")

if __name__ == "__main__":
    main()

