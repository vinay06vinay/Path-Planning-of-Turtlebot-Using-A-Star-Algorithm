import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
import heapq as hq
import time
import copy
from collections import defaultdict
import rospy
from geometry_msgs.msg import Twist

def obstacle_space(width,height,canvas,robot_radius,clearance):
    obstacle_matrix = {}
    if(clearance > 5 or clearance < 0):
        clearance = 5
    offset  = robot_radius+clearance
    for x in range(width):
        for y in range(height):
#           #Obstacle map and matrix formation with offset consideration
           # #First Rectangle
            r1c = (x+offset>=150 and x-offset<=165) and (y+offset>=0 and y-offset<=125)
            #second Rectangle
            r2c = (x+offset>=250 and x-offset<=265) and (y+offset>= 75 and y-offset<=200)
            #Circle with clearance. The offset will be added to the radius
            c1c = ((x-400)**2 + (y-110)**2 - ((50+offset)**2) <= 0)
            if(r1c or r2c or  c1c):
                obstacle_matrix[(x,y)] = -1
                canvas[y,x] = (0,120,255)
            #Creating Boundary with offset
            elif(((y-offset <=0) or (y-200+offset >=0))) :
                obstacle_matrix[(x,y)] = -1
                canvas[y,x] = (0,120,255)
            elif(((x-offset <=0) or (x-600+offset >=0))) :
                obstacle_matrix[(x,y)] = -1
                canvas[y,x] = (0,120,255)
            else:
                obstacle_matrix[(x,y)] = math.inf
                
                
          #Highlight the original obstacles without clearance with different colour
            # #First Rectangle
            r1 = (x>=150 and x<=165) and (y>=0 and y<=125)
            #second Rectangle
            r2 = (x>=250 and x<=265) and (y>=75 and y<=200)

            #Circle map
            c1 = ((x-400)**2 + (y-110)**2 - (50**2) <= 0)
            if(r1 or r2 or c1 ):
                canvas[y,x] = (0,0,255)
    plt.imshow(canvas)
    return obstacle_matrix,canvas
    
def get_start_goal_inputs(obstacle_matrix):
    
    while True:
        print("Enter the x-coordinate of start node")
        start_x = int(input())
        print("Enter the y-coordinate of start node")
        start_y = int(input())
        if((start_x <= 0 or start_x > 599) ):
            print("The X-coordinate of start node is out of the map. Please enter the coordinates again")
        elif((start_y <= 0 or start_y >199)):
            print("The Y-coordinate of start node is out of the map. Please enter the coordinates again")
        elif(obstacle_matrix[(start_x,start_y)] == -1):
            print("The entered start node falls in obstacle space")
        else:
            break
    while True:
        print("Initial Orientation of start node of robot(Angle in multiple of 30)")
        start_angle = int(input())
        if(start_angle %3 != 0):
            print("Please enter correct angle ")
        else:
            if(start_angle <30 ):
                start_angle = 360+start_angle
            break
    while True:
        print("Enter the x-coordinate of goal node")
        goal_x = int(input())
        print("Enter the y-coordinate of goal node")
        goal_y = int(input())
        if(goal_x <= 0 or goal_x >599):
            print("The X-coordinate of start node is out of the map. Please enter the coordinates again")
        elif(goal_y <= 0 and goal_y > 199):
            print("The Y-coordinate of start node is out of the map. Please enter the coordinates again")
        elif(obstacle_matrix[(goal_x,goal_y)] == -1):
            print("The entered goal node falls in obstacle space")
        else:
            break
    return (start_x,start_y,start_angle,goal_x,goal_y)
    
def get_robot_inputs():
    #robot radius from meter to cm
    robot_radius = 0.105*100
    while True:  
        print("Enter the clearance of the robot (0-50 mm)")
        clearance = int(input())
        if(clearance>=0):
            break
    while True:  
        print("Enter the Left RPM of the motor")
        left_rpm = int(input())
        if(left_rpm>=0):
            break 
    while True:  
        print("Enter the Right RPM of the motor")
        right_rpm = int(input())
        if(right_rpm>=0):
            break  
    clearance = clearance/10
    return clearance, robot_radius , left_rpm ,  right_rpm


def action(obstacle_matrix,current_node,i):
    next_node=copy.deepcopy(current_node)
    next_node = list(next_node)
    #radius of the wheel of turtlebot
    r = 0.093 
    l = 0.160 
    dt = 0.1
    t= 0 
    D=0
    x_init = next_node[0]
    y_init = next_node[1]
    theta_init = next_node[2]
    #List of nodes between the parent and child node
    node_list =[]
    node_list.append((x_init,y_init,theta_init))
    while (t<1):
        t = t+dt
        x = x_init + ((0.5*r*(i[0]+i[1]))*np.cos(np.deg2rad(theta_init))*dt)
        y = y_init + ((0.5*r*(i[0]+i[1]))*np.sin(np.deg2rad(theta_init))*dt)
        theta = theta_init + ((r/l)*(i[1] - i[0]))*dt
        #D is the cost to come
        D=D+ math.sqrt(math.pow((0.5*r * (i[0] + i[1]) * math.cos(np.deg2rad(theta)) * dt),2)+math.pow((0.5*r * (i[0] + i[1]) * math.sin(np.deg2rad(theta)) * dt),2))
        if((round(x) >0 and round(x) < 600) and (round(y) >0 and round(y) < 200) and (obstacle_matrix[(int(round(x)),int(round(y)))] != -1)):
            node_list.append((x,y,theta))
        else:
            return False,tuple(next_node), node_list,D
        #Updating the values of initial values with current ones so that during next node calculations it has the track of current ones
        x_init, y_init, theta_init = x, y, theta

    next_node[0] = int(round(x_init))
    next_node[1] = int(round(y_init))
    next_node[2] = theta
    if((round(next_node[0]) >0 and round(next_node[0]) < 600) and (round(next_node[1]) >0 and round(next_node[1]) < 200) and (obstacle_matrix[(int(round(next_node[0])),int(round(next_node[1])))] != -1)):
        return True,tuple(next_node), node_list,D
    else:
        return False,tuple(next_node), node_list,D
        
       
    
'''
This Function Calculates the cost to goal between current and goal nodes using Euclidean Distance Formula
'''
def cost_2_goal(start_x,start_y,goal_x,goal_y):
    point_1 = np.array((start_x,start_y))
    point_2 = np.array((goal_x,goal_y))
    cost = np.linalg.norm(point_1-point_2)
    return cost
'''
To check whether the node is already visited. If so, returns true and updates the total cost if its less
'''
def check_duplicate(new_node,visited_matrix):
    if(new_node[0] > 599 or new_node[1] > 199):
        return True
    angle = new_node[2]
    if(angle>=360):
        angle%=360
    if(visited_matrix[int(new_node[0]*2)][int(new_node[1]*2)][int(angle/30)] == 0):
        visited_matrix[int(new_node[0]*2)][int(new_node[1]*2)][int(angle/30)] = 1
        return False,visited_matrix
    else:
        return True,visited_matrix
def astar(obstacle_matrix,start_x,start_y,start_theta,goal_x,goal_y,rpm_l,rpm_r):
    visited_matrix = np.zeros((1200,400,12))
    closed_list = {} #List of nodes explored
    open_list = []
    veloc_dict = {}
    theata_dist={}
    veloc_dict[(start_x,start_y)] = [(2 * np.pi * rpm_l) / 60, (2 * np.pi * rpm_r) / 60, start_theta, start_x, start_y]
    explored_dict = defaultdict(list) #Used for node explore visualisation where each key is a parent node with all child nodes appended
    child_parent_dict = {} #Used for backtracking and to get the optimal path
    node_list_dict = {} #this dict contains all the intermediate nodes for the current node
    parent_node = (start_x,start_y,start_theta)
    initial_cost_2_come = 0
    initial_cost_2_goal = cost_2_goal(start_x,start_y,goal_x,goal_y)
    total_cost = initial_cost_2_come+initial_cost_2_goal
    hq.heapify(open_list)
    hq.heappush(open_list,(total_cost,initial_cost_2_come,parent_node,(start_x,start_y,start_theta)))
    count = 0
    goal_reached = False
    explored_dict[parent_node].append((parent_node))
    while(len(open_list)>0):
        cn=hq.heappop(open_list)
        current_node = cn[3]
        parent_node = cn[2]
        current_cost2c = cn[1]
        current_total_cost = cn[0]
        closed_list[current_node] = cn[2]
        explored_dict[(int(parent_node[0]),int(parent_node[1]),int(parent_node[2]))].append((int(current_node[0]),int(current_node[1]),int(current_node[2])))
        child_parent_dict[((current_node[0]),(current_node[1]))] = ((parent_node[0]),(parent_node[1])) 
        goal_check_distance = np.sqrt(((goal_y-current_node[1])**2)+((goal_x-current_node[0])**2))
        if(goal_check_distance<4):
            goal_reached = True
            veloc_dict[(goal_x, goal_y)] = [(2 * np.pi * rpm_l) / 60, (2 * np.pi * rpm_r) / 60, start_theta, goal_x, goal_y]
            print("Goal Reached, Starting Back Track")

            break
        action_set = [[0,rpm_l],[rpm_l,0],[rpm_l,rpm_l],[0,rpm_r],[rpm_r,0],[rpm_r,rpm_r],[rpm_l,rpm_r],[rpm_r,rpm_l]]        
        for i in action_set:
            vel_L = (2 * np.pi * i[0]) / 60
            vel_R = (2 * np.pi * i[1]) / 60
            # traj_action(current_node,vel_L,vel_R, )
            flag,new_node,node_list,cost_new = action(obstacle_matrix,current_node,i)
            # print(flag,new_node,node_list,cost_new)
            if(flag):
                if(new_node not in closed_list):
                    check_duplicate_flag,visited_matrix = check_duplicate(new_node,visited_matrix)
                    if(check_duplicate_flag):
                        for i in open_list:
                            if (new_node == i[3]):
                                cost = current_cost2c+cost_new+cost_2_goal(new_node[0],new_node[1],goal_x,goal_y)
                                if(cost < i[0]):
                                    i = list(i)
                                    i[0] = cost
                                    i[1] = cost_new+current_cost2c
                                    i[2] = cn[2]
                                    i = tuple(i)
                                    node_list_dict[new_node] = node_list

                                    veloc_dict[(new_node[0],new_node[1])] = [vel_L, vel_R, new_node[2],new_node[0],new_node[1]]

                                    hq.heapify(open_list)
                                    # child_parent_dict[(int(new_node[0]),int(new_node[1]))] = (int(current_node[0]),int(current_node[1])) 
                                break
                    else:
                        cost = current_cost2c+cost_new+cost_2_goal(new_node[0],new_node[1],goal_x,goal_y)
                        c2c = current_cost2c+cost_new
                        node_to_push = (cost,c2c,current_node,new_node)
                        node_list_dict[new_node] = node_list
                        veloc_dict[(new_node[0],new_node[1])] = [vel_L, vel_R, new_node[2], new_node[0], new_node[1]]
                        hq.heappush(open_list,node_to_push)
                        # explored_dict[(int(current_node[0]),int(current_node[1]),int(current_node[2]))].append((int(new_node[0]),int(new_node[1]),int(new_node[2])))
                        hq.heapify(open_list)
                   
        count += 1 
    return(child_parent_dict,goal_reached,closed_list,explored_dict,closed_list,node_list_dict,veloc_dict)
# '''
# Backtracking using the child parent dictionary relationship got from the exploration
# '''
def back_track(child_parent_dict,start_x,start_y,start_theta,goal_x,goal_y):
    start = (start_x,start_y)
    optimal_list = []
    optimal_list.append((goal_x,goal_y))
    if((float(goal_x),float(goal_y)) in child_parent_dict.keys()):
        current = (float(goal_x),float(goal_y))
    else:
        last_key = list(child_parent_dict)[-1]
        current = last_key
    while(start!=current):
        if (current in child_parent_dict.keys()):
            optimal_list.append(child_parent_dict[current])
            current = child_parent_dict.pop(current)
        else:
            break

    optimal_list.append(start)
    return optimal_list[::-1]

# def sim_ip(optimal_list, velo_dict):
#     print(velo_dict)
#     print(optimal_list)


def move_robot(pub_vel,vel_change,theta_change):
    r = rospy.Rate(0.085)
    vel_value = Twist()
    # velocity = np.sqrt(dvx * dvx + dvy * dvy) / 100.0
    endTime = rospy.Time.now() + rospy.Duration(1)
    #while rospy.Time.now() < endTime:
    vel_value.linear.x = vel_change
    vel_value.angular.z = theta_change
    pub_vel.publish(vel_value)
    r.sleep()
# '''
# Visualizing and saving the video using OpenCV
# '''
def visualise(canvas,optimal_list,explored_dict,closed_list,child_parent_dict,node_list_dict,start,veloc_dict, obstacle_matrix):
    fourcc = cv2.VideoWriter_fourcc(*'XVID')    
    out = cv2.VideoWriter('A-Star.avi',fourcc,250,(canvas.shape[1],canvas.shape[0]))
    # for key,value in child_parent_dict.items():
    #     cv2.arrowedLine(canvas, (int(value[0]),int(value[1])), (int(key[0]),int(key[1])), (255,255,255), 1,tipLength=0.5) 
    #     out.write(canvas)

    for key,values in explored_dict.items():
        for i in values:
            pt1 = key[:2]
            pt2 = i[:2]
            cv2.arrowedLine(canvas, pt1, pt2, (255,255,255), 1,tipLength=0.5) 
        # cv2.imshow("Exploration",canvas)
        out.write(canvas)
        cv2.waitKey(1)

    # for key,value in closed_list.items():
    #     parent_pt = (int(value[0]),int(value[1]))
    #     child_pt = (int(key[0]),int(key[1]))
    #     cv2.arrowedLine(canvas, parent_pt, child_pt, (255,255,255), 1) 
    #     out.write(canvas)
    '''
    keys = list(closed_list.keys())
    for key in keys:
        if(start != key):
            t = node_list_dict[key]
            for i in range(len(t)-1):
                pt1 = (int(t[i][0]),int(t[i][1]))
                pt2  = (int(t[i+1][0]),int(t[i][1+1]))
                cv2.line(canvas, pt1, pt2, (255,255,255), 1) 
            out.write(canvas)
            cv2.waitKey(1)
        '''
    for i in range(len(optimal_list)-1):
        pt1 = (int(optimal_list[i][0]),int(optimal_list[i][1]))
        pt2 = (int(optimal_list[i+1][0]),int(optimal_list[i+1][1]))
        cv2.line(canvas, pt1, pt2, (0,255,0), 1,cv2.LINE_AA)
        # cv2.imshow("Optimal",canvas)
        # cv2.waitKey(1)
        out.write(canvas)
    for i in range(500):
        out.write(canvas)
    out.release()
    
    cv2.imshow('Output',canvas)
    cv2.waitKey(5000)
    # cv2.destroyAllWindows()
    # plt.imshow(canvas)

    rospy.init_node('a_star')
    r = rospy.Rate(0.085)
    pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    #rospy.sleep(1)
    turtle_model = rospy.get_param("model","burger")
    wheelRadius = 3.8
    frequency = 100
    vel_value = Twist()
    vel_value.linear.x = 0.0
    vel_value.linear.y = 0.0
    vel_value.linear.z = 0.0
    vel_value.angular.x = 0.0
    vel_value.angular.y = 0.0
    vel_value.angular.z = 0.0
    pub_vel.publish(vel_value)
    for i in optimal_list:
        vel_l, vel_r, theta, x, y = veloc_dict[i]
        print(vel_l, vel_r, theta, x, y)
        theta_radians = np.pi * theta/180
        theta_dot = (0.033/0.16)*(vel_r - vel_l)
        theta_change = theta_dot + theta_radians
        x_dot = (0.033/2)*(vel_r+vel_l)*math.cos(theta_change)
        y_dot = (0.033/2)*(vel_r+vel_l)*math.sin(theta_change)
        vel_change = np.sqrt(x_dot**2+y_dot**2)
        print(vel_change,theta_change)
        vel_value.linear.x = vel_change
        vel_value.angular.z = theta_change
        pub_vel.publish(vel_value)
        r.sleep()
        #move_robot(pub_vel,vel_change,theta_change)
        # w = 0.5

        # flag = True

#         dvx = wheelRadius * 0.5 * (vel_l + vel_r) * math.cos(theta)
#         dvy = wheelRadius * 0.5 * (vel_l + vel_r) * math.sin(theta)
#         dx = dvx
#         dy = dvy
#         dw = (0.33/0.16)*(vel_l-vel_r)
#         x = x + dx
#         y = y + dy
#         print(dvx, dvy, dw)

#         # move_robot(pub_vel, dvx, dvy, dw)

#         for index in range(0, 100):
#             dvx = wheelRadius * 0.5 * (vel_l + vel_r) * math.cos(theta)
#             dvy = wheelRadius * 0.5 * (vel_l + vel_r) * math.sin(theta)
#             dx = dvx / frequency
#             dy = dvy / frequency
#             dw = dw
#             x = x + dx
#             y = y + dy
#             print(dvx, dvy, dw)
#             move_robot(pub_vel, dvx, dvy, dw)
#         #
#             if ((round(x) > 0 and round(x) < 600) and (round(y) > 0 and round(y) < 200) and (obstacle_matrix[(int(round(x)), int(round(y)))] != -1)):
#                 flag = True
#             else:
#                 flag = False
#                 break










def main():
    canvas = np.ones((200,600,3),dtype='uint8')
    clearance,robot_radius,left_rpm,right_rpm = get_robot_inputs()
    obstacle_matrix,canvas = obstacle_space(600,200,canvas,robot_radius,clearance) 
    start_x,start_y,start_theta,goal_x,goal_y = get_start_goal_inputs(obstacle_matrix)
    
    # #Changing the y-coordinates in relation to image coordinates
    # start_y = 200-start_y
    # goal_y = 200-goal_y
    print(f"The start node selected : {(start_x,start_y,start_theta)}. The goal node selected : {(goal_x,goal_y)}")
    start_time = time.time()
    child_parent_dict,goal_reached,closed_list,explored_dict,closed_list,node_list_dict, veloc_dict = astar(obstacle_matrix,start_x,start_y,start_theta,goal_x,goal_y,left_rpm,right_rpm)
    end_time = time.time()  
    print(f"Time Taken By the Algorithm : {end_time - start_time} seconds")
    if(goal_reached == False):
        print("No Goal Found")
        return
    
    optimal_list = back_track(child_parent_dict,start_x,start_y,start_theta,goal_x,goal_y)
    # sim_ip(optimal_list, veloc_dict)
    visualise(canvas,optimal_list,explored_dict,closed_list,child_parent_dict,node_list_dict,(start_x,start_y,start_theta),veloc_dict, obstacle_matrix)

if __name__ == '__main__':
    
    main()
    
    

