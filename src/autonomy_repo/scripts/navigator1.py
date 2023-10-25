#!/usr/bin/env python3

import typing as T
import numpy as np
import rclpy                    # ROS2 client library
from rclpy.node import Node     # ROS2 node baseclass

# asl_tb3_lib
from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

from scipy.interpolate import splev, splrep

# Navigator inherits from BaseNavigator
class Navigator(BaseNavigator):
    # initial base class (happen before everything else)
    def __init__(self) -> None:
        super().__init__("navigator") 
        
        self.kp = 2.0 # Proportional control gain for heading control
        
        self.V_PREV_THRES = 0.0001 # velocity threshold for trajectory tracking

        self.kpx = 2
        self.kpy = 2
        self.kdx = 2
        self.kdy = 2

    def reset(self) -> None:
        self.V_prev = 0
        self.omega_prev = 0.
        self.t_prev = 0.

    # 1. implement/override "compute_heading_control"
    def compute_heading_control(self, cur_state: TurtleBotState, goal_state: TurtleBotState) -> TurtleBotControl:
        
        err = wrap_angle(goal_state.theta - cur_state.theta)
        w = self.kp * err

        # new TurtleBotControl
        control = TurtleBotControl()
        # set omega attribute to computed angular velocity
        control.omega = w

        return control
    
    # 2. implement/override "trajectory_control"
    def compute_trajectory_tracking_control(self, state: TurtleBotState, plan: TrajectoryPlan, t: float) -> TurtleBotControl:
        # control = TurtleBotControl()

        # x_d = splev(t, plan.path_x_spline, der=0)
        # y_d = splev(t, plan.path_y_spline, der=0)
        # xd_d = splev(t, plan.path_x_spline, der=1)
        # yd_d = splev(t, plan.path_y_spline, der=1)
        # xdd_d = splev(t, plan.path_x_spline, der=2)
        # ydd_d = splev(t, plan.path_y_spline, der=2)

        # ########## Code starts here ##########
        # J_inv = np.array([[np.cos(state.theta), np.sin(state.theta)],
        #                   [-np.sin(state.theta)/self.V_prev, np.cos(state.theta)/self.V_prev]])

        # u = np.array([[xdd_d + self.kpx*(x_d - state.x) + self.kdx*(xd_d - self.V_prev*np.cos(state.theta))],
        #               [ydd_d + self.kpy*(y_d - state.y) + self.kdy*(yd_d - self.V_prev*np.sin(state.theta))]])
        
        # a_and_w = J_inv @ u
        # om = a_and_w[1, 0]

        # V = self.V_prev + a_and_w[0,0]*(t - self.t_prev)

        # # Preventing singularities
        # if np.abs(V) < self.V_PREV_THRES:
        #     if V > 0:
        #         V = self.V_PREV_THRES
        #     else:
        #         V = -self.V_PREV_THRES

        # # save the commands that were applied and the time
        # self.t_prev = t
        # self.V_prev = V

        # control.v = V
        # control.omega = om
        # return control
        # compute desired state x_d, xd_d, xdd_d, y_d, yd_d, ydd_d sampled from spline
        x_d = splev(t, plan.path_x_spline, der=0)
        xd_d = splev(t, plan.path_x_spline, der=1)
        xdd_d = splev(t, plan.path_x_spline, der=2)
        y_d = splev(t, plan.path_y_spline, der=0)
        yd_d = splev(t, plan.path_y_spline, der=1)
        ydd_d = splev(t, plan.path_y_spline, der=2)

        # minimum V = threshold
        if self.V_prev < self.V_PREV_THRES:
            self.V_prev = self.V_PREV_THRES

        # find current state xd and yd
        xd = self.V_prev * np.cos(state.theta)
        yd = self.V_prev * np.sin(state.theta)

        # find virtual control inputs: u1, u2
        u1 = xdd_d + self.kpx*(x_d-state.x) + self.kdx *(xd_d - xd)
        u2 = ydd_d + self.kpx*(y_d-state.y) + self.kdx *(yd_d - yd)
        z = np.array([u1,u2]) 

        # matrix 
        M = np.array([[np.cos(state.theta), -yd],
                      [np.sin(state.theta),  xd]])
        
        # solve for real control input [a,w] in 
        r = np.linalg.solve(M, z)

        # output in TurtuleBotControl [V, omega]
        dt = t - self.t_prev
        tracking_control = TurtleBotControl()
        tracking_control.v = self.V_prev + r[0] * dt # v = v0 + a*dt
        tracking_control.omega = r[1]

        # update prev parameters
        self.t_prev = t
        self.V_prev = tracking_control.v
        self.omega_prev = tracking_control.omega

        return tracking_control
    
    # 3. A* path planning
    def compute_trajectory_plan(self, state: TurtleBotState, goal: TurtleBotState, occupancy: StochOccupancyGrid2D, resolution: float, horizon: float) -> TrajectoryPlan | None:
        
        
        statespace_lo = [state.x - horizon, state.y - horizon] # turtlebot travel within +- horizon
        statespace_hi = [state.x + horizon, state.y + horizon]
        
        # Initialize A* problem
        astar = AStar(statespace_lo, statespace_hi, [state.x, state.y], [goal.x, goal.y], occupancy, resolution=resolution) 

        # check solvability
        if not astar.solve() or len(astar.path) < 4:
            return None
        
        # Reset tracking controller history
        self.reset()
        
        # Ensure path is a numpy array
        path = np.asarray(astar.path)

        v_desired=0.15 
        spline_alpha=0.05

        # 1. define time stamp array, ts
        ts = []
        t_cur = 0
        for i in range(len(path)):
            # udpate ts by dt
            dt = 1 * v_desired # constant vel between waypoints
            t_cur = t_cur + dt
            ts.append(t_cur)
        # 2. fit cubic splines for x and y coordinates of the path
        path_x_spline = splrep(ts, path[:,0], s=spline_alpha) # fitting path_x
        path_y_spline = splrep(ts, path[:,1], s=spline_alpha) # fitting path_y

        # x = np.array(path[:, 0])
        # y = np.array(path[:, 1])
        # ts = np.zeros(len(x))
        # for i in range(len(x)):
        #     if i == 0:
        #         continue
        #     ts[i] = ts[i-1] + np.sqrt((y[i] - y[i - 1])**2 + (x[i] - x[i - 1])**2)/v_desired
        # path_x_spline = splrep(ts, x, k=3, s=spline_alpha)
        # path_y_spline = splrep(ts, y, k=3, s=spline_alpha)
        
        return TrajectoryPlan(
            path=path,
            path_x_spline=path_x_spline,
            path_y_spline=path_y_spline,
            duration=ts[-1],
        )


class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1):
        self.statespace_lo = statespace_lo         # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.x_offset = x_init                     
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.closed_set = set()    # the set containing the states that have been visited
        self.open_set = set()      # the set containing the states that are condidate for future expension

        self.est_cost_through = {}  # dictionary of the estimated cost from start to goal passing through state (often called f score), key to index is some state
        self.cost_to_arrive = {}    # dictionary of the cost-to-arrive at state from start (often called g score)
        self.came_from = {}         # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)

        self.path = None        # the final path as a list of states

    def is_free(self, x):
        """
        Checks if a give state x is free, meaning it is inside the bounds of the map and
        is not inside any obstacle.
        Inputs:
            x: state tuple
        Output:
            Boolean True/False
        Hint: self.occupancy is a DetOccupancyGrid2D object, take a look at its methods for what might be
              useful here
        """
        ########## Code starts here ##########
        # raise NotImplementedError("is_free not implemented") 
        # check if x is inside the bound (lower to upper) -> state valid
        for i in range(len(x)):
            if x[i] < self.statespace_lo[i] or x[i] > self.statespace_hi[i]: 
                return False
        # check if x is inside any obstacle -> collison free
        return self.occupancy.is_free(np.array(x))
    
        ########## Code ends here ##########

    def distance(self, x1, x2):
        """
        Computes the Euclidean distance between two states.
        Inputs:
            x1: First state tuple
            x2: Second state tuple
        Output:
            Float Euclidean distance

        HINT: This should take one line. Tuples can be converted to numpy arrays using np.array().
        """
        ########## Code starts here ##########
        # raise NotImplementedError("distance not implemented")
        # 2d euclidean distance in this hw    
        euclidean_d = np.sqrt((x2[0] - x1[0])**2 + (x2[1] - x1[1])**2)
        # print(euclidean_d)
        return euclidean_d
    
        ########## Code ends here ##########

    def snap_to_grid(self, x):
        """ Returns the closest point on a discrete state grid
        Input:
            x: tuple state
        Output:
            A tuple that represents the closest point to x on the discrete state grid
        """
        return (
            self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
            self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
        )

    def get_neighbors(self, x):
        """
        Gets the FREE neighbor states of a given state x. Assumes a motion model
        where we can move up, down, left, right, or along the diagonals by an
        amount equal to self.resolution.
        Input:
            x: tuple state
        Ouput:
            List of neighbors that are free, as a list of TUPLES

        HINTS: Use self.is_free to check whether a given state is indeed free.
               Use self.snap_to_grid (see above) to ensure that the neighbors
               you compute are actually on the discrete grid, i.e., if you were
               to compute neighbors by adding/subtracting self.resolution from x,
               numerical errors could creep in over the course of many additions
               and cause grid point equality checks to fail. To remedy this, you
               should make sure that every neighbor is snapped to the grid as it
               is computed.
        """
        neighbors = []
        ########## Code starts here ##########
        # raise NotImplementedError("get_neighbors not implemented")
        neighbors_tmp = []
        movements = [
        (self.resolution, 0),                  # Move right
        (-self.resolution, 0),                 # Move left
        (0, self.resolution),                  # Move up
        (0, -self.resolution),                 # Move down
        (self.resolution, self.resolution),    # Move right-up
        (self.resolution, -self.resolution),   # Move right-down
        (-self.resolution, self.resolution),   # Move left-up
        (-self.resolution, -self.resolution),  # Move left-down
        ]

        # Compute all neighbors and append them to the neighbors list
        for dx in movements:
            neighbor = self.snap_to_grid((np.array(x) + dx)) # dx = change in state
            # only append free neighbor into neighbors list
            if self.is_free(neighbor):
                neighbors.append(neighbor)

        ########## Code ends here ##########
        # print(neighbors)
        return neighbors

    def find_best_est_cost_through(self):
        """
        Gets the state in open_set that has the lowest est_cost_through
        Output: A tuple, the state found in open_set that has the lowest est_cost_through
        """
        return min(self.open_set, key=lambda x: self.est_cost_through[x])

    def reconstruct_path(self):
        """
        Use the came_from map to reconstruct a path from the initial location to
        the goal location
        Output:
            A list of tuples, which is a list of the states that go from start to goal
        """
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))

    # def plot_path(self, fig_num=0, show_init_label=True):
    #     """Plots the path found in self.path and the obstacles"""
    #     if not self.path:
    #         return

    #     self.occupancy.plot(fig_num)

    #     solution_path = np.asarray(self.path)
    #     plt.plot(solution_path[:,0],solution_path[:,1], color="green", linewidth=2, label="A* solution path", zorder=10)
    #     plt.scatter([self.x_init[0], self.x_goal[0]], [self.x_init[1], self.x_goal[1]], color="green", s=30, zorder=10)
    #     if show_init_label:
    #         plt.annotate(r"$x_{init}$", np.array(self.x_init) + np.array([.2, .2]), fontsize=16)
    #     plt.annotate(r"$x_{goal}$", np.array(self.x_goal) + np.array([.2, .2]), fontsize=16)
    #     plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)

    #     plt.axis([0, self.occupancy.width, 0, self.occupancy.height])

    # def plot_tree(self, point_size=15):
    #     plot_line_segments([(x, self.came_from[x]) for x in self.open_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
    #     plot_line_segments([(x, self.came_from[x]) for x in self.closed_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
    #     px = [x[0] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
    #     py = [x[1] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
    #     plt.scatter(px, py, color="blue", s=point_size, zorder=10, alpha=0.2)

    def solve(self):
        """
        Solves the planning problem using the A* search algorithm. It places
        the solution as a list of tuples (each representing a state) that go
        from self.x_init to self.x_goal inside the variable self.path
        Input:
            None
        Output:
            Boolean, True if a solution from x_init to x_goal was found

        HINTS:  We're representing the open and closed sets using python's built-in
                set() class. This allows easily adding and removing items using
                .add(item) and .remove(item) respectively, as well as checking for
                set membership efficiently using the syntax "if item in set".
        """
        ########## Code starts here ##########
        is_found = False
        # initial open(candidate for future expension) and close set(visited)
        self.open_set.add(self.x_init)

        # search all potential next state from candidates
        while len(self.open_set) > 0:
            # select current state to be the candidate state that has the lowest est_cost_through
            x_cur = min(self.open_set, key=lambda state: self.est_cost_through[state])
            # check if explore to the goal state 
            if x_cur == self.x_goal:
                # is_found = True
                # construct the shortest path
                self.path = self.reconstruct_path()
                return self.reconstruct_path()
            # update traversed set
            self.open_set.remove(x_cur) # delete from candidate set after traversed it
            self.closed_set.add(x_cur)

            # neighbors for current state x_cur
            cur_neighbors = self.get_neighbors(x_cur)
            # print(cur_neighbors)
            for x_neighbor in cur_neighbors:
                if x_neighbor in self.closed_set:
                    continue
                tentative_cost_to_arrive = self.cost_to_arrive[x_cur] + self.distance(x_cur,x_neighbor)
                if not x_neighbor in self.open_set:
                    self.open_set.add(x_neighbor)
                elif tentative_cost_to_arrive > self.cost_to_arrive[x_neighbor]:
                    continue
                # keep track that x_neighbor's parent is x_cur
                self.came_from[x_neighbor] = x_cur
                # print(self.came_from)
                # set cost to arrive x_neigbor
                self.cost_to_arrive[x_neighbor] = tentative_cost_to_arrive
                # set the est_cost that pass though one more point (x_neighbor) from init to goal state
                self.est_cost_through[x_neighbor] = tentative_cost_to_arrive + self.distance(x_neighbor,self.x_goal)
             
        return is_found
        ########## Code ends here ##########


if __name__ == "__main__":
    rclpy.init()            # initialize ROS client library
    navigator = Navigator()    # create the node instance
    rclpy.spin(navigator)        # call ROS2 default scheduler
    rclpy.shutdown()        # clean up after node exits