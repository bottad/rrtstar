import sys
from read_config import read_config_from_yaml
import colors

import pygame
import shapely

import rrtstar
import rrtstar_kd
import reverse_rrtstar
import reverse_rrtstar_kd

from enum import Enum

class RRTOptions(Enum):
    RRTSTAR = "rrtstar"
    REVERSE_RRTSTAR = "reverse_rrtstar"
    RRTSTAR_KD = "rrtstar_kd"
    REVERSE_RRTSTAR_KD = "reverse_rrtstar_kd"

def choose_rrt(argv) -> (RRTOptions, bool):
    reverse = False
    kd = False
    for arg in argv:
        if arg == "r":
            reverse = True
        if arg == "kd":
            kd = True

    if reverse and kd:
        return (RRTOptions.REVERSE_RRTSTAR_KD, True)
    elif reverse and not kd:
        return (RRTOptions.REVERSE_RRTSTAR, True)
    elif not reverse and kd:
        return (RRTOptions.RRTSTAR_KD, False)
    else:
        return (RRTOptions.RRTSTAR, False)

################ choose rrtstar or reverse_rrtstar to start eighter from start or goal node ##############
def RRT_solver(option: RRTOptions, 
            goal,
            xlim: [int, int],
            ylim: [int, int],
            obstacles: shapely.STRtree,
            robot_radius: float,
            num_nodes=2000,
            epsilon=0.1,
            radius=1,
            startstate = None
        ) -> bool:
    if option == RRTOptions.RRTSTAR:
        return rrtstar_kd.RRT_Solver(goal, xlim, ylim, obstacles, robot_radius, num_nodes, epsilon, radius, startstate)
    elif option == RRTOptions.RRTSTAR_KD:
        return rrtstar.RRT_Solver(goal, xlim, ylim, obstacles, robot_radius, num_nodes, epsilon, radius, startstate)
    elif option == RRTOptions.REVERSE_RRTSTAR:
        return reverse_rrtstar.RRT_Solver(goal, xlim, ylim, obstacles, robot_radius, num_nodes, epsilon, radius)
    elif option == RRTOptions.REVERSE_RRTSTAR_KD:
        return reverse_rrtstar_kd.RRT_Solver(goal, xlim, ylim, obstacles, robot_radius, num_nodes, epsilon, radius)
    else:
        print("[ERROR]\tNo valid RRT option chosen!\r\n")

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

def back_shift_start(shifted_start, bounds):
    start = [0, 0]
    #print(shifted_start)
    start[0] = shifted_start[0] + bounds[0]
    start[1] = shifted_start[1] + bounds[1]
    #print(start)
    return start

def draw_obstacles(pygame, screen, obstacle_list):
    for obstacle in obstacle_list:
        pygame.draw.polygon(screen, colors.BLUE, obstacle)
    pygame.display.update()

def draw_goal(pygame, screen, goal, goal_radius):
    pygame.draw.circle(screen, colors.GREEN, goal, goal_radius, 2)
    pygame.display.update()

def draw_start(pygame, screen, start):
    pygame.draw.circle(screen, colors.GREEN, start, 2, 2)
    pygame.display.update()

################ Main Function #####################

def main():
    # import configuartion
    obstacle_list, shapely_obstacles, boundary, bounds, win_size, agents_dict = read_config_from_yaml("config/config_2.yaml")
    shift_obstacles(obstacle_list, bounds)
    WINSIZE = win_size
    obstacles = shapely.STRtree(shapely_obstacles)

    # Initialize screen
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('RRTstar')
    screen.fill(colors.WHITE)

    # read input arguments

    option, reverse = choose_rrt(sys.argv)

    for agent, attribute in agents_dict.items():
        x0 = attribute["state"]
        #print(x0)
        start = [x0['x'], x0['y']]
        goal = attribute["goal"]
        goal_radius = attribute["goal_radius"]
        # initialize map
        draw_obstacles(pygame, screen, obstacle_list)
        shifted_goal = shift_goal(goal, bounds)
        shifted_start = shift_start(start, bounds)
        draw_goal(pygame, screen, shifted_goal, goal_radius)
        if not reverse:
            draw_start(pygame, screen, shifted_start)

        # run planner
        RRT = RRT_solver(option, shapely.Point(goal).buffer(goal_radius), [bounds[0], bounds[2]], [bounds[1], bounds[3]], obstacles, 10, 500, 20, 100, start)
        RRT.solve(pygame, screen)
        # main loop
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.MOUSEBUTTONUP and reverse:
                    start = pygame.mouse.get_pos()
                    RRT.set_start(back_shift_start(start, bounds))
                    RRT.check_goal_reachable(pygame, screen)

if __name__ == "__main__":
    main()