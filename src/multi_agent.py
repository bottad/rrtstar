from read_config import read_config_from_yaml
import colors

import pygame
import shapely

import multi_agent_rrt

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

def shift_point(point, bounds):
    shifted_point = [0, 0]
    #print(start)
    shifted_point[0] = point[0] - bounds[0]
    shifted_point[1] = point[1] - bounds[1]
    #print(shifted_start)
    return shifted_point

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

def draw_path(pygame, screen, path, bounds):
    current_point = shift_point(path[0], bounds)
    for i in range(1, len(path)):
        next_point = shift_point(path[i], bounds)
        pygame.draw.line(screen, colors.PINK, current_point, next_point, 3)
        current_point = next_point
        pygame.display.update()
        


class Agent:
    start = None
    goal = None

    RRT = None

    def __init__(self, goal, goal_radius, xlim, ylim, obstacles, start = None):
        self.goal = goal
        self.goal_radius = goal_radius
        self.start = start
        self.RRT = multi_agent_rrt.RRT_Solver(shapely.Point(goal).buffer(goal_radius), xlim, ylim, obstacles, 10, 500, 20, 100, start)

################ Main Function #####################

def main():
    # import configuartion
    obstacle_list, shapely_obstacles, boundary, bounds, win_size, agents_dict = read_config_from_yaml("config/config_4.yaml")
    shift_obstacles(obstacle_list, bounds)
    WINSIZE = win_size
    obstacles = shapely.STRtree(shapely_obstacles)

    # Initialize screen
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('RRTstar')
    screen.fill(colors.WHITE)

    agents = []

    agent_number = 1

    for agent_nr, attribute in agents_dict.items():
        print(f"Building RRTstar-tree for agent {agent_number}:\r\n")
        x0 = attribute["state"]
        #print(x0)
        start = [x0['x'], x0['y']]
        goal = attribute["goal"]
        goal_radius = attribute["goal_radius"]
        # initialize map
        draw_obstacles(pygame, screen, obstacle_list)
        #shifted_goal = shift_goal(goal, bounds)
        #shifted_start = shift_start(start, bounds)
        #draw_goal(pygame, screen, shifted_goal, goal_radius)

        #draw_start(pygame, screen, shifted_start)

        # create agent class
        agent = Agent(goal, goal_radius, [bounds[0], bounds[2]], [bounds[1], bounds[3]], obstacles, start)
        agent.RRT.build_tree()

        agents.append(agent)
        agent_number += 1

    running = True
    agent_number = 1

    for agent in agents:
        print(f"[INFO]\t######## Solving path for agent {agent_number}: ########\n\r")
        run_agent = True
        draw_goal(pygame, screen, shift_goal(agent.goal, bounds), agent.goal_radius)
        print("Choose goal by clicking on it on the screen...\r\n")
        while run_agent:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    break
                elif event.type == pygame.MOUSEBUTTONUP:
                    start = pygame.mouse.get_pos()
                    draw_start(pygame, screen, start)
                    agent.RRT.set_start(back_shift_start(start, bounds))
                    if agent.RRT.solve():
                        path = agent.RRT.path
                        draw_path(pygame, screen, path, bounds)
                    run_agent= False                                # Comment this out to solve multiple paths per agent
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                        run_agent = False
            if not running:
                break
        if not running:
            break
        agent_number += 1

    print("... No more agents left. Close window to terminate the program!\n\r")

    # main loop
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

if __name__ == "__main__":
    main()