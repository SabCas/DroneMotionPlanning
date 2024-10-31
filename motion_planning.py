import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv
import pandas as pd

from planning_utils import a_star, heuristic, create_grid, prune_path 
from planning_utils import RRTStar
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from udacidrone.frame_utils import local_to_global

import matplotlib.pyplot as plt


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 0

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        lat0_lon0 = pd.read_csv('colliders.csv', nrows = 1, header = None)
        lat0, lon0 = lat0_lon0.iloc[0,0], lat0_lon0.iloc[0,1]
        _, lat0 = lat0.split()
        _, lon0 = lon0.split()
        lat0 = np.float64(lat0)
        lon0 = np.float64(lon0)
        
        # TODO: set home position to (lat0, lon0, 0)
        self.set_home_position(lon0, lat0, 0)
        current_local_pos = global_to_local(self.global_position, self.global_home)
        
        # Me: checking current local position
        print ('current local position {0} and {1}'.format(current_local_pos[0], current_local_pos[1]))
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=3)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
       
        # TODO: convert start position to current position rather than map center
        grid_start = (int(current_local_pos[0] - north_offset), int(current_local_pos[1] - east_offset))
        goal_offset_north = 75
        goal_offset_east = 30
        goal_local = [current_local_pos[0] + goal_offset_north, current_local_pos[1] + goal_offset_east, TARGET_ALTITUDE + 20]
        
        # Convert local goal to grid coordinates
        grid_goal = (int(goal_local[0] - north_offset), int(goal_local[1] - east_offset))

        if grid[grid_goal[0], grid_goal[1]] == 1:
            print("Goal is in a collision area. Finding nearest non-collision point.")
            found = False
            for i in range(-5, 6):
                for j in range(-5, 6):
                    if grid[grid_goal[0] + i, grid_goal[1] + j] == 0:
                        grid_goal = (grid_goal[0] + i, grid_goal[1] + j)
                        found = True
                        print("Adjusted goal position to nearest free space:", grid_goal)
                        break
                if found:
                    break
            if not found:
                print("Failed to find a free space for goal within search range.")
                return


        print('Local Start and Goal:', grid_start, grid_goal)

        print('Local Start and Goal: ', grid_start, grid_goal)
        # path, _ = a_star(grid, heuristic, grid_start, grid_goal)
          # Run RRT to find a path from start to goal
        rrt_star = RRTStar(grid_start, grid_goal, grid)
        path = rrt_star.plan()
        # path = prune_path(path, grid)

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        print('Waypoints: ', waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=10000)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
