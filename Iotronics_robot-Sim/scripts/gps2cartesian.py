#!/usr/bin/env python3

import rospy
import math
from geographiclib.geodesic import Geodesic
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist

def get_origin():
    origin_lat = rospy.get_param('start_latitude')
    origin_long = rospy.get_param('start_longitude')
    return origin_lat, origin_long

def gps_to_cartesian(goal_lat, goal_long):
    '''
    Find the cartesian coordinate of the GPS location with respect to the GPS reference origin,
    which is the start location of the robot.

    Args:
        goal_lat : Goal latitude
        goal_long : Goal longitude

    Returns:
        x : x coordinate in the sensor frame in meters
        y : y coordinate in the sensor frame in meters
    '''
    origin_lat, origin_long = get_origin()
    geod = Geodesic.WGS84  # define the WGS84 ellipsoid
    g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long)  # Compute geodesic calculations between two GPS points
    distance = g['s12']  # access distance
    azimuth = g['azi1']

    # Convert polar (distance and azimuth) to x, y translation in meters (needed for ROS)
    # Convert azimuth to radians
    azimuth = math.radians(azimuth)
    x = math.cos(azimuth) * distance
    y = math.sin(azimuth) * distance

    return x, y

def sensor_lidar():
    # read a message from the laser scanner device on the /scan topic
    scan_data = rospy.wait_for_message('/scan', LaserScan)
    return scan_data

def think(scan_data):
    # extract region of scan we are interested in
    num_points = len(scan_data.ranges)
    fwd_range = scan_data.ranges[190:210]

    # compute an average distance to wall measurement
    distance_to_wall = sum(fwd_range) / len(fwd_range)

    # condition: if distance to wall is above some threshold 6 meters, move forward
    approach_threshold = 6.0  # meters
    move_flag = distance_to_wall > approach_threshold

    print(f'Distance to wall: {distance_to_wall}')

    return move_flag

def act(robot_vel_publisher, move_flag):
    # create a new Twist object
    robot_vel = Twist()

    # set the nominal forward velocity
    fwd_vel = 0.5  # m/s

    # condition: if move_flag is true, move robot
    if move_flag:
        robot_vel.linear.x = fwd_vel
        msg = "Robot Moving!"
    else:
        robot_vel.linear.x = 0.0
        msg = "Robot Stopped!"

    # publish velocity
    robot_vel_publisher.publish(robot_vel)

    return msg

def main():
    # initialize ROS Node
    rospy.init_node('approach_barn_example')

    # set control frequency & time
    hz = 0.5  # hz
    rate = rospy.Rate(hz)
    t_start = time.time()  # seconds

    # create a publisher to send velocity commands to the robot
    robot_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # setup control loop
    while not rospy.is_shutdown():
        # SENSE
        scan_data = sensor_lidar()

        # THINK/PLAN
        move_flag = think(scan_data)

        # ACT
        message = act(robot_vel_publisher, move_flag)

        # Simulate GPS data
        goal_lat = 37.7749
        goal_long = -122.4194

        # Calculate Cartesian coordinates based on GPS
        x, y = gps_to_cartesian(goal_lat, goal_long)
        rospy.loginfo("The translation from the origin (0,0) to the GPS location provided is {:.3f}, {:.3f} m.".format(x, y))

        t_elapsed = time.time() - t_start
        print(f'At time [{t_elapsed:.3f}]: {message}!')

        rate.sleep()

if __name__ == '__main__':
    main()

