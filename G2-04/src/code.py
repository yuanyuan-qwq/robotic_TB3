#!/usr/bin/env python3

import rospy
import math
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.2
STOP_FRONT_DISTANCE = 0.2
STOP_BACK_DISTANCE = 0.3
LIDAR_ERROR = 0.05
SAFE_STOP_FRONT_DISTANCE = STOP_FRONT_DISTANCE + LIDAR_ERROR
SAFE_STOP_BACK_DISTANCE = STOP_BACK_DISTANCE + LIDAR_ERROR

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()

    def get_scan_right(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []

        samples = len(scan.ranges)  # The number of samples is defined in
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples

        if samples_view > samples:
            samples_view = samples

        if samples_view == 1:
            scan_filter.append(scan.ranges[270])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2

            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 100

        return scan_filter

    def get_scan_back(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        if samples_view == 1: 
            scan_filter.append(scan.ranges[180]) # view at the back

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 100
        
        return scan_filter
        
    def get_scan_front(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        if samples_view == 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 100
        
        return scan_filter
    
    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True
        from_left = False
        from_right = False

        while not rospy.is_shutdown():
            #right scan
            lidar_right_distances = self.get_scan_right()
            min_right_distance = min(lidar_right_distances) # min distance = right distance
            #back scan
            lidar_back_distances = self.get_scan_back()
            min_back_distance = min(lidar_back_distances)
            
            # front scan
            lidar_front_distances = self.get_scan_front()
            min_front_distance = min(lidar_front_distances)
            
            
            # too near to the wall - move left to move away from wall
            if min_right_distance > 0 and min_right_distance < 0.2: #range far 0 until 0.2
                print("Move away from wall")
                from_right = True
                linearx = LINEAR_VEL
                angularz = 0.5
                twist.angular.z = angularz
                twist.linear.x = linearx
                self._cmd_pub.publish(twist)
                rospy.loginfo('Distance of the obstacle from right : %f', min_right_distance)
                rospy.loginfo('Robot Direction %f', linearx)
                rospy.loginfo('Robot rotation %f', angularz)
                
            # near the wall but have a distance - move forward to follow wall
            elif min_right_distance >= 0.2 and min_right_distance < 0.3:
                print("Follow wall")
                linearx = 0.1
                angularz = 0
                if from_right:
                  angularz = -2
                  from_right = False
                elif from_left:
                  angularz = 2
                  from_left = False
                else:
                  angularz = 0
                twist.angular.z = angularz
                twist.linear.x = linearx
                self._cmd_pub.publish(twist)
                rospy.loginfo('Distance of the obstacle from right : %f', min_right_distance)
                rospy.loginfo('Robot Direction %f', linearx)
                
            # cannot detect wall - move left
            elif min_right_distance == 100 or min_front_distance == 100 or min_back_distance == 100:
                print("No obstacle")
                linearx = LINEAR_VEL
                angularz = 0.05
                twist.angular.z = angularz
                twist.linear.x = linearx
                self._cmd_pub.publish(twist)
                rospy.loginfo('Distance of the obstacle: 0')
                rospy.loginfo('Robot Direction %f', linearx)
            
            # back got obstacles + front not obstacles - move forward
            elif min_back_distance < SAFE_STOP_BACK_DISTANCE and min_front_distance > SAFE_STOP_FRONT_DISTANCE:
              print("Move forward")
              # Run away
              twist.linear.x = LINEAR_VEL
              twist.angular.z = 0.0
              self._cmd_pub.publish(twist)
              turtlebot_moving = True
              rospy.loginfo('Distance of the back obstacle : %f', min_back_distance)

            # front got obstacles - turn
            elif min_front_distance < SAFE_STOP_FRONT_DISTANCE and min_right_distance != 0:
              print("Turn")
              linearx = 0.0
              angularz = 3
              twist.linear.x = linearx
              twist.angular.z = angularz
              self._cmd_pub.publish(twist)
              turtlebot_moving = True
              rospy.loginfo('Distance of the obstacle from front : %f', min_front_distance)
              rospy.loginfo('Robot Direction %f', linearx)
            
            # else situation - go right to find wall
            else:
              print("Go to wall")
              from_left = True
              linearx = LINEAR_VEL
              angularz = -0.1
              twist.linear.x = linearx
              twist.angular.z = angularz
              self._cmd_pub.publish(twist)
              turtlebot_moving = True
              rospy.loginfo('Distance of the obstacle from right : %f', min_right_distance)
              rospy.loginfo('Robot Direction %f', linearx)
              rospy.loginfo('Robot rotation %f', angularz)


	


def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
