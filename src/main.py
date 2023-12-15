from read_config import read_config_from_yaml

################ choose rrtstar or reverse_rrtstar to start eighter from start or goal node ##############
from reverse_rrtstar import *

import pygame
import shapely

def shift_obstacles(obstacle_list, bounds):
    #print(obstacle_list)
    for obstacle in obstacle_list:
        for point in obstacle:
            point[0] -= bounds[0]
            point[1] -= bounds[1]
    #print(obstacle_list)

def shift_goal(goal, bounds):
    shifted_goal = [0, 0]
    #print(goal)
    shifted_goal[0] = goal[0] - bounds[0]
    shifted_goal[1] = goal[1] - bounds[1]
    #print(shifted_goal)
    return shifted_goal

def shift_start(start, bounds):
    shifted_start = [0, 0]
    #print(start)
    shifted_start[0] = start[0] - bounds[0]
    shifted_start[1] = start[1] - bounds[1]
    #print(shifted_start)
    return shifted_start


def run_planner(screen, start, goal, goal_radius, shapely_obstacles, obstacles, boundary, bounds):
    blue = (0, 0, 255)
    green= (0, 255, 0)
    for obstacle in obstacle_list:
        pygame.draw.polygon(screen, blue, obstacle)
    shifted_goal = shift_goal(goal, bounds)
    pygame.draw.circle(screen, green, shifted_goal, goal_radius, 2)
    shifted_start = shift_start(start, bounds)
    pygame.draw.circle(screen, green, shifted_start, 2, 2)
    pygame.display.update()

    solver = RRT_Solver(shapely.Point(goal).buffer(goal_radius), [bounds[0], bounds[2]], [bounds[1], bounds[3]], obstacles, 10, 500, 20, 100, start)
    if not solver.solve(pygame, screen):
        print("ERROR\tNo valid path found!")
    else:
        print(f"\tFound path:\t{solver.path}")
    return

obstacle_list, shapely_obstacles, boundary, bounds, win_size, agents_dict = read_config_from_yaml("config/config_2.yaml")

shift_obstacles(obstacle_list, bounds)

WINSIZE = win_size

obstacles = shapely.STRtree(shapely_obstacles)

pygame.init()
screen = pygame.display.set_mode(WINSIZE)
pygame.display.set_caption('RRTstar')
white = 255, 255, 255
black = 20, 20, 40
screen.fill(white)

for agent, attribute in agents_dict.items():
    x0 = attribute["state"]
    #print(x0)
    start = [x0['x'], x0['y']]
    goal = attribute["goal"]
    goal_radius = attribute["goal_radius"]
    run_planner(screen, start, goal, goal_radius, shapely_obstacles, obstacles, boundary, bounds)
    running = True
    while running:
       for event in pygame.event.get():
           if event.type == pygame.QUIT:
                 running = False