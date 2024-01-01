import random, math
from math import sqrt, cos, sin, atan2
import shapely
from scipy import spatial
import numpy as np

from shapely.geometry.base import BaseGeometry

class Node:
    x: float
    y: float
    cost: float
    parent =  None

    def __init__(self, xcoord, ycoord, parent=None):
        self.x = xcoord
        self.y = ycoord
        self.cost = 0
        self.parent = parent

    def set_parent(self, parent_node):
        self.parent = parent_node

def distance(n1: Node, n2: Node) -> float:
    return sqrt((n2.x - n1.x) * (n2.x - n1.x) + (n2.y - n1.y) * (n2.y - n1.y))

def coord_distance(x1, y1, x2, y2) -> float:
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))

def shift_point(x, y, xlim, ylim):
    new_x = x - xlim[0]
    new_y = y - ylim[0]
    return [new_x, new_y]

class RRT_Solver:
    kdtree: spatial.cKDTree     # KDtree to store the sampled points
    obstacles: shapely.STRtree  # STRtree of shapely objects

    start: Node                 # node
    goal_geo: BaseGeometry      # shapely polygon
    goal: Node

    XLIMIT = []                 # [min_x, max_x]
    XDIM = 0                    # difference between x limits
    YLIMIT = []                 # [min_y, max_y]
    YDIM = 0                    # difference between y limits

    EPSILON = 0                 # stepsize
    EPSILON_EXTENSION = 0       # stepsize for tree extension
    NUMNODES = 0                # number of random samples
    RADIUS = 0                  # radius to be considered a neighbor
    RADIUS_EXTENSION = 0        # radius to be considered a neighbor when extending the tree

    ROBOTRADIUS = 2             # robots size assuming its a circle

    RESOLVE_RADIUS = 0  # radius to rebuild a path when a collsion is imminent

    def __init__(
        self,
        goalregion: BaseGeometry,
        xlim: [int, int],
        ylim: [int, int],
        obstacles: shapely.STRtree,
        robot_radius: float,
        num_nodes,
        epsilon,
        radius,
        resolve_radius,
        startstate = None
    ):
        self.tree = []
        self.path = []              # list of path coordinate points ([x, y])
        self.optimized_path = []    # list of optimized path coordinate points ([x, y])
        if startstate != None:
            self.start = Node(startstate[0], startstate[1])
        else:
            self.start = None
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
        self.EPSILON_EXTENSION = epsilon
        self.RADIUS = radius
        self.RADIUS_EXTENSION = radius
        self.ROBOTRADIUS = robot_radius
        self.RESOLVE_RADIUS = resolve_radius

    def set_extension_params(self, epsilon, radius):
        self.EPSILON_EXTENSION = epsilon
        self.RADIUS_EXTENSION = radius

    def set_start(self, startstate):
        self.start = Node(startstate[0], startstate[1])

    def get_random_node(self) -> Node:
        rand_x = random.random() * self.XDIM + self.XLIMIT[0]
        rand_y = random.random() * self.YDIM + self.YLIMIT[0]

        return Node(rand_x, rand_y)
    
    def get_random_node_in_range(self, start: (float, float)):
        rand_x = random.random() * self.RESOLVE_RADIUS + start[0]
        rand_y = random.random() * self.RESOLVE_RADIUS + start[1]

        rand_x = max(self.XLIMIT[0], rand_x)
        rand_x = min(self.XLIMIT[1], rand_x)

        rand_y = max(self.YLIMIT[0], rand_y)
        rand_y = min(self.YLIMIT[1], rand_y)

        return Node(rand_x, rand_y)
    
    def get_nearest_neighbors_to_start(self, n):
        if n > len(self.tree):
            n = len(self.tree)
            if n == 1:
                return [self.tree[0]]
        _, indices = self.kdtree.query([self.start.x, self.start.y], n)

        # Retrieve the nearest neighbors
        n_nearest_neighbors = [self.tree[i] for i in indices]
        return n_nearest_neighbors

    def take_step(self, n1: Node, n2: Node):
        if distance(n1, n2) < self.EPSILON:
            n2.set_parent(n1)
            return n2
        else:
            theta = math.atan2(n2.y - n1.y, n2.x - n1.x)
            return Node(
                n1.x + self.EPSILON * cos(theta), n1.y + self.EPSILON * sin(theta), n1
            )
        
    def take_extention_step(self, n1: Node, n2: Node):
        if distance(n1, n2) < self.EPSILON_EXTENSION:
            n2.set_parent(n1)
            return n2
        else:
            theta = math.atan2(n2.y - n1.y, n2.x - n1.x)
            return Node(
                n1.x + self.EPSILON_EXTENSION * cos(theta), n1.y + self.EPSILON_EXTENSION * sin(theta), n1
            )

    def collision_check(self, n1: Node, n2: Node) -> bool:
        segment = shapely.LineString([[n1.x, n1.y], [n2.x, n2.y]])
        robot = segment.buffer(self.ROBOTRADIUS)

        nearest_obstacle_index = self.obstacles.nearest(robot)
        if self.obstacles.geometries.take(nearest_obstacle_index).intersects(robot):
            return False
        return True
    
    def path_collision_check(self, p1: (int, int), p2: (int, int)) -> bool:
        segment = shapely.LineString([p1, p2])
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
    
    def re_wire(self, newnode):
        indices_within_radius = self.kdtree.query_ball_point([newnode.x, newnode.y], self.RADIUS)

        for index in indices_within_radius:
            p = self.tree[index]
            if (self.collision_check(p, newnode)
                and p != newnode.parent
                and newnode.cost + distance(p, newnode) < p.cost
            ):
                p.x = self.tree[index].x
                p.y = self.tree[index].y
                p.set_parent(newnode)
                p.cost = newnode.cost + distance(p, newnode)
                self.tree[index] = p

    def re_wire_extention(self, newnode):
        indices_within_radius = self.kdtree.query_ball_point([newnode.x, newnode.y], self.RADIUS_EXTENSION)

        for index in indices_within_radius:
            p = self.tree[index]
            if (self.collision_check(p, newnode)
                and p != newnode.parent
                and newnode.cost + distance(p, newnode) < p.cost
            ):
                p.x = self.tree[index].x
                p.y = self.tree[index].y
                p.set_parent(newnode)
                p.cost = newnode.cost + distance(p, newnode)
                self.tree[index] = p

    def check_goal_reachable(self) -> bool:
        found_path = False
        print("[INFO]\tChecking if goal reached: ...")
        if self.start == None:
            print("[ERROR]\t... Start is not defined!\r\n")
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
            self.construct_path()
            return True
        else:
            print("[WARNING]\t... No path to goal found!\r\n")
            return False

    def construct_path(self):
        if self.start == None:
            print("[ERROR]\t... Start is not defined!\r\n")
            return False
        self.path = []
        print("[INFO]\tConstructing path: ...")
        current_node = self.start
        i = 1
        while current_node != self.goal:
            x = current_node.x
            y = current_node.y
            self.path.append((x, y))
            print(f"\tComputed {i} path segments", end="\r")
            i += 1
            current_node = current_node.parent
        self.path.append((current_node.x, current_node.y))
        print(f"\tComputed {i} path segments", end="\r")
        print("                                        ", end="\r")
        print("[INFO]\t... complete!\r\n")
        return True

    def optimize_path(self) -> bool:
        print("[INFO]\tOptimizing path: ...")
        if self.path == []:
            print("[ERROR]\t... There is no path to optimize!\r\n")
            return False
        current_point = (self.start.x, self.start.y)
        opt_path = [current_point]
        i = 1
        j_start = 0
        while current_point != self.path[-1]:
            opt_step = current_point
            for j in range(j_start, len(self.path)):
                if self.path_collision_check(current_point, self.path[j]):
                    opt_step = self.path[j]
                    j_start = j
            print(f"\tComputed {i} optimized path segments", end="\r")
            i += 1
            current_point = opt_step
            opt_path.append(current_point)
        print(f"\tComputed {i} optimized path segments", end="\r")
        print("                                        ", end="\r")
        self.optimized_path = opt_path
        print(f"[INFO]\t... complete!\r\n")
        return True

    def extend_tree(self, iter) -> bool:
        for i in range(iter):
            random_node = self.get_random_node()
            # connecting to nearest neighbor
            _, index = self.kdtree.query([random_node.x, random_node.y], 1)
            nn = self.tree[index]
            newnode = self.take_extention_step(nn, random_node)

            if self.collision_check(nn, newnode):
                newnode, nn = self.choose_parent(nn, newnode)
                
                self.tree.append(newnode)
                self.build_kd_tree()
                self.re_wire_extention(newnode)
            print(f"\t{i} iterations complete", end="\r")
        print(f"[INFO]\t... {iter} additional iterations complete!\r\n")
        return self.check_goal_reachable()
    
    def solve(self, start = None):
        if start != None:
            self.set_start(start)

        if len(self.tree) < 2:
            self.build_tree()

        if self.start != None:
            if self.check_goal_reachable():
                return True
            else:
                print("[INFO]\t... Path not yet found starting additional iterations: ...")
                for j in range(3):
                    if self.extend_tree(200):
                        self.build_kd_tree()
                        return True
                return False  
        print("[WARNING]\t... No start defined!")
        return 
    
    def resolve_path(self, position: [float, float], robots: [BaseGeometry]):
        print("[INFO]\tResolving path: ...")
        if self.path == []:
            print("[ERROR]\nThere is no path to resolve!")
            return False
        
        search_point = shapely.Point(position[0], position[1])
        done_flag = False

        interim_goal = [self.goal.x, self.goal.y]

        print(f"[INFO]\t... Searching for first segment after intersection from point {position}")

        for idx in range(len(self.path)):
            current_point = shapely.Point(self.path[idx])
            is_intersecting = False
            if search_point.distance(current_point) <= self.RADIUS_EXTENSION:
                is_intersecting = True

            if is_intersecting:
                done_flag = True
            if not is_intersecting and done_flag:
                interim_goal = self.path[idx]
                print(f"[INFO]\t... interim goal set to point {idx} of the path ...")
                break

        if coord_distance(interim_goal[0], interim_goal[1], position[0], position[1]) > self.RESOLVE_RADIUS:
            self.RESOLVE_RADIUS = coord_distance(interim_goal[0], interim_goal[1], position[0], position[1])

        print(f"[INFO]\t... Trying to reach the point {interim_goal} from {position}...")

        mini_solver = mini_RRT_Solver(position, interim_goal, self.XLIMIT, self.YLIMIT, self.obstacles, robots, self.ROBOTRADIUS, 150, 10, 40, self.RESOLVE_RADIUS)

        counter = 2

        done = False

        while not done:
            print(f"[INFO]\tNo valid path found, using {counter}x search radius ...\r\n")
            mini_solver.enlarge_radius()
            done = mini_solver.solve()
            counter *= 2
            if counter > 8:
                mini_solver.NUMEXT = 200
                if not mini_solver.solve():
                    print("[WARNING]\tNo path found!")
                    return []

        print("[INFO]\t... done!\r\n")

        return mini_solver.path

    def build_kd_tree(self):
        #print("[INFO]\tStart building the KD-tree: ...")
        # Extract x, y coordinates from nodes and organize them into a NumPy array
        coordinates = np.array([(node.x, node.y) for node in self.tree])

        # Build cKDTree from the NumPy array
        self.kdtree = spatial.cKDTree(coordinates)
        #print("[INFO]\t... complete!\r\n")

    def build_tree(self):
        print("[INFO]\tStart building the RRT* tree: ...")
        for i in range(self.NUMNODES):
            random_node = self.get_random_node()
            # connecting to nearest neighbor
            _, index = self.kdtree.query([random_node.x, random_node.y], 1)
            nn = self.tree[index]
            newnode = self.take_step(nn, random_node)

            # adding connection to tree if collisionfree
            if self.collision_check(nn, newnode):
                newnode, nn = self.choose_parent(nn, newnode)
                
                self.tree.append(newnode)
                self.build_kd_tree()
                # optimize tree with respect to new Node
                self.re_wire(newnode)
            print(f"\t{i} iterations complete", end="\r")
        print(f"[INFO]\t... {self.NUMNODES} iterations complete!\r\n")


class mini_RRT_Solver:
    kdtree: spatial.cKDTree     # KDtree to store the sampled points
    obstacles: shapely.STRtree  # STRtree of shapely objects
    robots: [BaseGeometry]      # List of robot geometries

    start: (float, float)
    goal: (float, float)

    XLIMIT = []                 # [min_x, max_x]
    YLIMIT = []                 # [min_y, max_y]
    SOLVE_RADIUS = 0

    EPSILON = 0                 # stepsize
    EPSILON_EXTENSION = 0       # stepsize for tree extension
    NUMNODES = 0                # number of random samples
    RADIUS = 0                  # radius to be considered a neighbor
    RADIUS_EXTENSION = 0        # radius to be considered a neighbor when extending the tree

    ROBOTRADIUS = 2             # robots size assuming its a circle

    def __init__(
        self,
        start: [int, int],
        goal: [float, float],
        xlim: [int, int],
        ylim: [int, int],
        obstacles: shapely.STRtree,
        robots: [BaseGeometry],
        robot_radius: float,
        num_nodes,
        epsilon,
        radius,
        solve_radius
    ):
        self.tree = []
        self.path = []          # list of path coordinate points ([x, y])

        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])

        # Initialize tree with goal state
        self.tree.append(
            self.goal
        )
        self.kdtree = spatial.cKDTree([[goal[0], goal[1]]])

        self.obstacles = obstacles
        self.robots = robots
        self.XLIMIT = xlim
        self.YLIMIT = ylim

        self.NUMNODES = num_nodes
        self.NUMEXT = 80
        self.EPSILON = epsilon
        self.EPSILON_EXTENSION = epsilon
        self.RADIUS = radius
        self.RADIUS_EXTENSION = radius
        self.ROBOTRADIUS = robot_radius
        self.SOLVE_RADIUS = solve_radius

    def set_extension_params(self, epsilon, radius):
        self.EPSILON_EXTENSION = epsilon
        self.RADIUS_EXTENSION = radius

    def set_start(self, startstate):
        self.start = Node(startstate[0], startstate[1])
    
    def enlarge_radius(self):
        self.SOLVE_RADIUS * 2

    def get_random_node(self):
        rand_x = random.random() * self.SOLVE_RADIUS + self.start.x
        rand_y = random.random() * self.SOLVE_RADIUS + self.start.y

        rand_x = max(self.XLIMIT[0], rand_x)
        rand_x = min(self.XLIMIT[1], rand_x)

        rand_y = max(self.YLIMIT[0], rand_y)
        rand_y = min(self.YLIMIT[1], rand_y)

        return Node(rand_x, rand_y)
    
    def get_nearest_neighbors_to_start(self, n):
        if n > len(self.tree):
            n = len(self.tree)
            if n == 1:
                return [self.tree[0]]
            
        _, indices = self.kdtree.query([self.start.x, self.start.y], n)

        # Retrieve the nearest neighbors and their costs
        n_nearest_neighbors = [self.tree[i] for i in indices]
        return n_nearest_neighbors

    def take_step(self, n1: Node, n2: Node):
        if distance(n1, n2) < self.EPSILON:
            n2.set_parent(n1)
            return n2
        else:
            theta = math.atan2(n2.y - n1.y, n2.x - n1.x)
            return Node(
                n1.x + self.EPSILON * cos(theta), n1.y + self.EPSILON * sin(theta), n1
            )
        
    def take_extention_step(self, n1: Node, n2: Node):
        if distance(n1, n2) < self.EPSILON_EXTENSION:
            n2.set_parent(n1)
            return n2
        else:
            theta = math.atan2(n2.y - n1.y, n2.x - n1.x)
            return Node(
                n1.x + self.EPSILON_EXTENSION * cos(theta), n1.y + self.EPSILON_EXTENSION * sin(theta), n1
            )

    def collision_check(self, n1: Node, n2: Node) -> bool:
        segment = shapely.LineString([[n1.x, n1.y], [n2.x, n2.y]])
        robot = segment.buffer(self.ROBOTRADIUS)

        nearest_obstacle_index = self.obstacles.nearest(robot)
        if self.obstacles.geometries.take(nearest_obstacle_index).intersects(robot):
            return False
        for robot_idx in range(len(self.robots)):
            other_robot = self.robots[robot_idx]
            if other_robot.intersects(robot):
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
    
    def re_wire(self, newnode):
        indices_within_radius = self.kdtree.query_ball_point([newnode.x, newnode.y], self.RADIUS)

        for index in indices_within_radius:
            p = self.tree[index]
            if (self.collision_check(p, newnode)
                and p != newnode.parent
                and newnode.cost + distance(p, newnode) < p.cost
            ):
                p.x = self.tree[index].x
                p.y = self.tree[index].y
                p.set_parent(newnode)
                p.cost = newnode.cost + distance(p, newnode)
                self.tree[index] = p

    def re_wire_extention(self, newnode):
        indices_within_radius = self.kdtree.query_ball_point([newnode.x, newnode.y], self.RADIUS_EXTENSION)

        for index in indices_within_radius:
            p = self.tree[index]
            if (self.collision_check(p, newnode)
                and p != newnode.parent
                and newnode.cost + distance(p, newnode) < p.cost
            ):
                p.x = self.tree[index].x
                p.y = self.tree[index].y
                p.set_parent(newnode)
                p.cost = newnode.cost + distance(p, newnode)
                self.tree[index] = p

    def check_goal_reachable(self) -> bool:
        found_path = False
        print("[INFO]\tChecking if goal reached: ...")
        if self.start == None:
            print("[ERROR]\t... Start is not defined!\r\n")
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
            self.construct_path()
            return True
        else:
            print("[WARNING]\t... No path to goal found!\r\n")
            return False

    def construct_path(self):
        if self.start == None:
            print("[ERROR]\t... Start is not defined!\r\n")
            return False
        self.path = []
        print("[INFO]\tConstructing path: ...")
        current_node = self.start
        i = 1
        while current_node != self.goal:
            x = current_node.x
            y = current_node.y
            self.path.append((x, y))
            print(f"\tComputed {i} path segments", end="\r")
            i += 1
            current_node = current_node.parent
        self.path.append((current_node.x, current_node.y))
        print(f"\tComputed {i} path segments", end="\r")
        print("                                        ", end="\r")
        print("[INFO]\t... complete!\r\n")
        return True

    def extend_tree(self, iter) -> bool:
        for i in range(iter):
            random_node = self.get_random_node()
            # connecting to nearest neighbor
            _, index = self.kdtree.query([random_node.x, random_node.y], 1)
            nn = self.tree[index]
            newnode = self.take_extention_step(nn, random_node)

            if self.collision_check(nn, newnode):
                newnode, nn = self.choose_parent(nn, newnode)
                
                self.tree.append(newnode)
                self.build_kd_tree()
                self.re_wire_extention(newnode)
            print(f"\t{i} iterations complete", end="\r")
        print(f"[INFO]\t... {iter} additional iterations complete!\r\n")
        return self.check_goal_reachable()
    
    def solve(self, start = None):
        if start != None:
            self.set_start(start)

        if len(self.tree) < 2:
            self.build_tree()

        if self.start != None:
            if self.check_goal_reachable():
                return True
            else:
                print("[INFO]\t... Path not yet found starting additional iterations: ...")
                for j in range(2):
                    if self.extend_tree(self.NUMEXT):
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

    def build_tree(self):
        print("[INFO]\tStart building the RRT* tree: ...")
        for i in range(self.NUMNODES):
            random_node = self.get_random_node()
            # connecting to nearest neighbor
            _, index = self.kdtree.query([random_node.x, random_node.y], 1)
            nn = self.tree[index]
            newnode = self.take_step(nn, random_node)

            # adding connection to tree if collisionfree
            if self.collision_check(nn, newnode):
                newnode, nn = self.choose_parent(nn, newnode)
                
                self.tree.append(newnode)
                self.build_kd_tree()
                # optimize tree with respect to new Node
                self.re_wire(newnode)
            print(f"\t{i} iterations complete", end="\r")
        print(f"[INFO]\t... {self.NUMNODES} iterations complete!\r\n")
