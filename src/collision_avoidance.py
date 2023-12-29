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

def back_shift_point(shifted_point, bounds):
    point = [0, 0]
    #print(shifted_start)
    point[0] = shifted_point[0] + bounds[0]
    point[1] = shifted_point[1] + bounds[1]
    #print(start)
    return point

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

def draw_path(pygame, screen, path, bounds, color = colors.PINK):
    current_point = shift_point(path[0], bounds)
    for i in range(1, len(path)):
        next_point = shift_point(path[i], bounds)
        pygame.draw.line(screen, color, current_point, next_point, 3)
        current_point = next_point
        pygame.display.update()

def draw_robot(pygame, screen, point, radius):
    pygame.draw.circle(screen, colors.RED, point, radius)
    pygame.display.update()

################ Main Function #####################
        
def main():
    # import configuartion
    obstacle_list, shapely_obstacles, boundary, bounds, win_size, agents_dict = read_config_from_yaml("config/config_5.yaml")
    shift_obstacles(obstacle_list, bounds)
    WINSIZE = win_size
    obstacles = shapely.STRtree(shapely_obstacles)

    robot_radius = 10

    # Initialize screen
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('RRTstar')
    screen.fill(colors.WHITE)

    agent_number = 0

    for agent_nr, attribute in agents_dict.items():
        print(f"Building RRTstar-tree for agent {agent_number}:\r\n")
        x0 = attribute["state"]
        #print(x0)
        start = [x0['x'], x0['y']]
        goal = attribute["goal"]
        goal_radius = attribute["goal_radius"]
        # initialize map
        draw_obstacles(pygame, screen, obstacle_list)
        shifted_goal = shift_goal(goal, bounds)
        #shifted_start = shift_start(start, bounds)
        draw_goal(pygame, screen, shifted_goal, goal_radius)

        #draw_start(pygame, screen, shifted_start)

        # create agent class
        RRT = multi_agent_rrt.RRT_Solver(shapely.Point(goal).buffer(goal_radius), [bounds[0], bounds[2]], [bounds[1], bounds[3]], obstacles, 10, 500, 20, 100, 100, start)
        RRT.build_tree()
        agent_number += 1

    if agent_number > 1:
        print("[ERROR]\tThis program is designed for one agent! Please use another configuration!\r\n")

    running = True
    initializing = True

    path = []

    print("Choose start location of agent by clicking on the screen...\r\n")
    while initializing:
        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    break
                elif event.type == pygame.MOUSEBUTTONUP:
                    start = pygame.mouse.get_pos()
                    draw_start(pygame, screen, start)
                    RRT.set_start(back_shift_start(start, bounds))
                    if RRT.solve():
                        path = RRT.path
                        draw_path(pygame, screen, path, bounds)
                    initializing = False
                    break                     
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                        start = [RRT.start.x, RRT.start.y]
                        draw_start(pygame, screen, start)
                        if RRT.solve():
                            path = RRT.path
                            draw_path(pygame, screen, path, bounds)
                        initializing = False
                        break

    print("Choose interfering robot locations by clicking on it on the screen...\r\n")
    
    collisions = []
    robots = []

    while running:
        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    break
                elif event.type == pygame.MOUSEBUTTONUP:
                    robot_pos = pygame.mouse.get_pos()
                    draw_robot(pygame, screen, robot_pos, 10)
                    robots.append(shapely.Point(back_shift_point(robot_pos, bounds)).buffer(robot_radius))
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                        print("[INFO]\tChecking for intersections...")
                        ################ collision checker #################
                        intersection = False
                        cur_point = path[0]
                        for robot_idx in range(len(robots)):
                            for idx in range(1, len(path)):
                                path_segment = shapely.LineString([cur_point, path[idx]]).buffer(robot_radius)
                                if path_segment.intersects(robots[robot_idx]):
                                    intersection = True
                                    collisions.append((idx - 1, robot_idx))
                                    print(f"[INFO]\t{len(collisions)} collisions detected.", end="\r")
                                    break
                            cur_point = path[idx]
                        print()

                        if intersection:
                            draw_path(pygame, screen, path, bounds, colors.ORANGE)
                            for collision in collisions:
                                print(f"[INFO]\tChecking collison starting from {collision[0]} whith robot number {collision[1]}")
                                path_correction = RRT.resolve_path(path[collision[0]], robots)
                                if path_correction == []:
                                    print("[WARNING]\tNo valid path correction found!\r\n")
                                    return
                                draw_path(pygame, screen, path_correction, bounds, colors.GREEN)
                            running = False
                            break
                        else:
                            print("[INFO]\t...no collisions detected!")
                            draw_path(pygame, screen, path, bounds, colors.GREEN)
                            running = False
                            break

    running = True

    print("you are here!")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN or event.key == pygame.K_KP_ENTER:
                        running = False
                        main()

if __name__ == "__main__":
    main()