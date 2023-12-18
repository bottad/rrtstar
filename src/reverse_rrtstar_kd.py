import random, math
from math import sqrt, cos, sin, atan2
import shapely
import heapq
from scipy import spatial
import numpy as np

from shapely.geometry.base import BaseGeometry

import colors

class Node:
    x = 0
    y = 0
    cost = 0
    parent = None

    def __init__(self, xcoord, ycoord, parent=None):
        self.x = xcoord
        self.y = ycoord
        self.parent = parent

    def set_parent(self, parent_node):
        self.parent = parent_node


def distance(n1: Node, n2: Node) -> float:
    return sqrt((n2.x - n1.x) * (n2.x - n1.x) + (n2.y - n1.y) * (n2.y - n1.y))

def shift_point(x, y, xlim, ylim):
    new_x = x - xlim[0]
    new_y = y - ylim[0]
    return [new_x, new_y]

class RRT_Solver:
    tree = []
    kdtree: spatial.cKDTree     # KDtree to store the sampled points
    obstacles: shapely.STRtree  # STRtree of shapely objects

    path = []                   # list of path coordinate points ([x, y])

    start: Node                 # node
    goal_geo: BaseGeometry      # shapely polygon
    goal: Node

    XLIMIT = []                 # [min_x, max_x]
    XDIM = 0                    # difference between x limits
    YLIMIT = []                 # [min_y, max_y]
    YDIM = 0                    # difference between y limits

    EPSILON = 0                 # stepsize
    NUMNODES = 0                # number of random samples
    RADIUS = 0                  # radius to be considered a neighbor

    ROBOTRADIUS = 2             # robots size assuming its a circle

    def __init__(
        self,
        goalregion: BaseGeometry,
        xlim: [int, int],
        ylim: [int, int],
        obstacles: shapely.STRtree,
        robot_radius: float,
        num_nodes=2000,
        epsilon=0.1,
        radius=1,
        startstate = None
    ):
        if startstate != None:
            self.start = Node(startstate[0], startstate[1])
        self.goal_geo = goalregion
        goal_center = self.goal_geo.centroid
        self.goal = Node(goal_center.x, goal_center.y)
        # Initialize tree with goal state
        self.tree.append(
            self.goal
        )
        self.kdtree = spatial.cKDTree([[goal_center.x, goal_center.y]])
        self.obstacles = obstacles
        self.XLIMIT = xlim
        self.XDIM = xlim[1] - xlim[0]
        self.YLIMIT = ylim
        self.YDIM = ylim[1] - ylim[0]
        self.NUMNODES = num_nodes
        self.EPSILON = epsilon
        self.RADIUS = radius
        self.ROBOTRADIUS = robot_radius

    def set_start(self, startstate):
        self.start = Node(startstate[0], startstate[1])


    def get_random_node(self) -> Node:
        rand_x = random.random() * self.XDIM + self.XLIMIT[0]
        rand_y = random.random() * self.YDIM + self.YLIMIT[0]

        return Node(rand_x, rand_y)
    
    def get_nearest_neighbors_to_start(self, n):
        _, indices = self.kdtree.query([self.start.x, self.start.y], n)

        # Retrieve the nearest neighbors and their costs
        n_nearest_neighbors = [self.tree[i] for i in indices]
        return n_nearest_neighbors

    def take_step(self, n1: Node, n2: Node):
        if distance(n1, n2) < self.EPSILON:
            n2.set_parent(n1)
            return n2
        # or check collision and add node to tree
        else:
            theta = math.atan2(n2.y - n1.y, n2.x - n1.x)
            return Node(
                n1.x + self.EPSILON * cos(theta), n1.y + self.EPSILON * sin(theta), n1
            )

    def collision_check(self, n1: Node, n2: Node) -> bool:
        segment = shapely.LineString([[n1.x, n1.y], [n2.x, n2.y]])
        robot = segment.buffer(self.ROBOTRADIUS)

        nearest_obstacle_index = self.obstacles.nearest(robot)
        if self.obstacles.geometries.take(nearest_obstacle_index).intersects(robot):
            return False
        return True
    
    def choose_parent(self, nn, newnode):
        indices_within_radius = self.kdtree.query_ball_point([newnode.x, newnode.y], self.RADIUS)

        # Retrieve the nodes within the radius
        nodes_within_radius = [self.tree[i] for i in indices_within_radius]

        for p in nodes_within_radius:
            if (self.collision_check(p, newnode)
            and p.cost + distance(p, newnode) < nn.cost + distance(nn, newnode)
            ):
                nn = p
        newnode.cost = nn.cost + distance(nn, newnode)
        newnode.set_parent(nn)
        return newnode, nn
    
    def re_wire(self, newnode, pygame, screen):
        indices_within_radius = self.kdtree.query_ball_point([newnode.x, newnode.y], self.RADIUS)

        for index in indices_within_radius:
            p = self.tree[index]
            if (self.collision_check(p, newnode)
                and p != newnode.parent
                and newnode.cost + distance(p, newnode) < p.cost
            ):
                pygame.draw.line(screen, colors.WHITE, shift_point(p.x,p.y, self.XLIMIT, self.YLIMIT), shift_point(p.parent.x, p.parent.y, self.XLIMIT, self.YLIMIT), 2)
                p.set_parent(newnode)
                p.cost = newnode.cost + distance(p, newnode)
                self.tree[index] = p
                pygame.draw.line(screen, colors.BLACK, shift_point(p.x,p.y, self.XLIMIT, self.YLIMIT), shift_point(newnode.x, newnode.y, self.XLIMIT, self.YLIMIT))

    def check_goal_reachable(self, pygame, screen) -> bool:
        found_path = False
        print("[INFO]\tChecking if goal reached: ...")
        if self.start == None:
            print("[INFO]\t... Start is not defined!\r\n")
            return False
        # choose n closest nodes
        nearest_neighbors = self.get_nearest_neighbors_to_start(10)
        # check which of the paths is the shortest by adding cost from node and distance to start
        min_cost = float('inf')
        best_neigbor = None
        for nn in nearest_neighbors:
            if self.collision_check(nn, self.start):
                proposed_cost = nn.cost + distance(nn, self.start)
                if proposed_cost < min_cost:
                    min_cost = proposed_cost
                    best_neigbor = nn
                    found_path = True

        if found_path:
            self.start.cost = min_cost
            self.start.set_parent(best_neigbor)
            self.tree.append(self.start)
            self.build_kd_tree()
            print("[INFO]\t... reached goal sucessfully!\r\n")

            self.construct_path(self.start, pygame, screen)
            return True
        else:
            # No path to obstacle found!
            return False

    def construct_path(self, start_node, pygame, screen):
        print("[INFO]\tConstructing path: ...")
        current_node = start_node
        i = 1
        while current_node != self.goal:
            x = current_node.x
            y = current_node.y
            pygame.draw.line(screen, colors.PINK, shift_point(x, y, self.XLIMIT, self.YLIMIT), shift_point(current_node.parent.x, current_node.parent.y, self.XLIMIT, self.YLIMIT), 3)
            self.path.append((x, y))
            print(f"\tComputed {i} path segments", end="\r")
            i += 1
            pygame.display.update()
            current_node = current_node.parent
        self.path.append((current_node.x, current_node.y))
        print(f"\tComputed {i} path segments", end="\r")
        print("                                        ", end="\r")
        print("[INFO]\t... complete!\r\n")

    def extend_tree(self, iter, pygame, screen) -> bool:
        for i in range(iter):
            random_node = self.get_random_node()
            nn = self.tree[0]
            for p in self.tree:
                if distance(p, random_node) < distance(nn, random_node):
                    nn = p
            newnode = self.take_step(nn, random_node)

            if self.collision_check(nn, random_node):
                newnode, nn = self.choose_parent(nn, newnode)
                
                self.tree.append(newnode)
                pygame.draw.line(screen, colors.BLACK, shift_point(nn.x, nn.y, self.XLIMIT, self.YLIMIT), shift_point(newnode.x, newnode.y, self.XLIMIT, self.YLIMIT))
                self.build_kd_tree()
                self.re_wire(newnode, pygame, screen)
                pygame.display.update()
            print(f"\t{i} iterations complete", end="\r")
        print(f"[INFO]\t... {iter} additional iterations complete!\r\n")
        return self.check_goal_reachable(pygame, screen)
    
    def solve(self, pygame, screen, start = None):
        if start != None:
            self.set_start(start)

        if len(self.tree) < 2:
            self.build_tree(pygame, screen)

        if self.start != None:
            if self.check_goal_reachable(pygame, screen):
                return True
            else:
                print("[INFO]\t... Path not yet found starting additional iterations: ...")
                for j in range(3):
                    if self.extend_tree(200, pygame, screen):
                        self.build_kd_tree()
                        return True
                return False  
        print("[WARNING]\t... No start defined!")
        return False

    def build_kd_tree(self):
        #print("[INFO]\tStart building the KD-tree: ...")
        # Extract x, y coordinates from nodes and organize them into a NumPy array
        coordinates = np.array([(node.x, node.y) for node in self.tree])

        # Build cKDTree from the NumPy array
        self.kdtree = spatial.cKDTree(coordinates)
        #print("[INFO]\t... complete!\r\n")

    def build_tree(self, pygame, screen):
        print("[INFO]\tStart building the RRt* tree: ...")
        for i in range(self.NUMNODES):
            random_node = self.get_random_node()
            nn = self.tree[0]
            # connecting to nearest neighbor
            for p in self.tree:
                if distance(p, random_node) < distance(nn, random_node):
                    nn = p
            newnode = self.take_step(nn, random_node)

            # adding connection to tree if collisionfree
            if self.collision_check(nn, random_node):
                newnode, nn = self.choose_parent(nn, newnode)
                
                self.tree.append(newnode)
                self.build_kd_tree()
                pygame.draw.line(screen, colors.BLACK, shift_point(nn.x, nn.y, self.XLIMIT, self.YLIMIT), shift_point(newnode.x, newnode.y, self.XLIMIT, self.YLIMIT))
                # optimize tree with respect to new Node
                self.re_wire(newnode, pygame, screen)
                pygame.display.update()
            print(f"\t{i} iterations complete", end="\r")
        print(f"[INFO]\t... {self.NUMNODES} iterations complete!\r\n")