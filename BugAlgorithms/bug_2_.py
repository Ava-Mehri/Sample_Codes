#Ava Mehri, UIC, amehri2@uic.edu, 2021
#Bug2 algorithm
#Gazebo simulation env, Maze world

#!/usr/bin/env python

# ROS stuff
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
# other useful math tools
from tf.transformations import euler_from_quaternion
from math import atan2
from math import sqrt
 
# global constants
angle_eps = 0.2
dis_eps = 0.01

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

pub = None

# Class that will be used to read and parse /odom topic
class odomReader:

    def __init__(self):

        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.x = None
        self.y = None
        self.theta = None

    # Function that will take care of input message from odom topic
    # This function will be called whenever new message is available to read
    # Subsequently odom topic is parsed to get (x,y,theta) coordinates 
    def newOdom(self, msg):
        # get x and y coordinates
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # convert quaternion to Euler angles
        rot_q = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

# Class that is responsible to read and parse raw LaserScan data 
class scanReader:

    def __init__(self):
        # subscribing to "/odom" topic
        # function newOdom() will take care of the new incoming message  
        sub = rospy.Subscriber("/scan", LaserScan, self.newScan)
        # divide laser scan data into 5 regions
        self.region = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
    # Function that will take care of input message from scan topic
    # This function will be called whenever new message is available to read
    # Subsequently scan topic is parsed to get region data: minimum distance to object from every sight 
    def newScan(self, msg):
        self.ranges = msg.ranges
        self.msg = msg
        self.region['left'] = min(self.ranges[60:100])
        self.region['fleft'] = min(self.ranges[20:60])
        self.region['front'] = min(self.ranges[0:20]+self.ranges[-20:])
        self.region['fright'] = min(self.ranges[300:340])
        self.region['right'] = min(self.ranges[260:300])
        
        #print "range[90]: ", msg.ranges[90]


# divide robot motion in 3 scenario
state_dict = {
    0: 'go to goal',
    1: 'circumnavigate obstacle',
    2: 'go back to closest point',
}
# define initial scenario
state = 0
# hit-flag changes to 1 once the robot detects the wall
hit_flag = 0
closest_point_x = 0
closest_point_y = 0


def main():
    global pub
    global hit_flag
    global state
    global closest_point_x
    global closest_point_y


    # initialize ROS node
    rospy.init_node("bug_1")
    # run stop function when this node is killed
    rospy.on_shutdown(stop)
    rospy.sleep(0.5)

    # define the control velocity publisher of topic type Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    # initialize odom and scan objects
    # Use these objects to access robot position in space and scan information
    odom = odomReader()
    scan = scanReader()
    rospy.sleep(0.5)

    # initialize speed as Twist topic 
    speed = Twist()

    # set the loop frequency
    rate = rospy.Rate(5)

    # Set the goal point 
    goal = Point()
    #goal.x = 0.0
    #goal.y = 4.0
    goal.x = -1.0
    goal.y = 2.0

    # arbitrary far away coordinate from goal
    closest_point = Point()
    closest_point.x = 1000
    closest_point.y = 1000

    # arbitrary large number representing inf distance to goal
    closest_dist = 1000 
    # Variable that stores the coordinate of hit point when you 
    # encounter obstacle for the first time
    hit_point = Point()



    while not rospy.is_shutdown():
        # initialize speed as Twist topic
        inc_x = goal.x - odom.x
        inc_y = goal.y - odom.y
        dist_to_goal = sqrt(inc_x * inc_x + inc_y * inc_y)

        if state == 0:
            # go to goal state.
            angle_to_goal = atan2(inc_y, inc_x)
            angle_diff = angle_to_goal - odom.theta
            # find the distance to the goal from current location

            if abs(angle_diff) > 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
            else:
                speed.linear.x = 0.3
                speed.angular.z = 0.0
            if dist_to_goal < 0.1:
                speed.linear.x = 0
                speed.angular.z = 0
                print("goal.x: %.3f" % (odom.x))
                print("goal.y: %.3f" % (odom.y))

            if scan.region['front'] < .2:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                state = 1
                # hit-flag changes to 1 once the robot detects the wall
                hit_flag += 1
                hit_point.x = odom.x
                hit_point.y = odom.y

            print "current state: ", state_dict[state]


        elif state == 1:
            # circumnavigate obstacle.
            # the robot follows the wall until it reaches again to the hit point
            speed.angular.z = 0.3
            if scan.region['front'] > .3 :
                if scan.region['fright'] > .3 :
                    speed.angular.z =- 0.2
                    speed.linear.x = 0.0
                elif scan.region['fright'] < .3:
                    speed.linear.x = 0.2
                    speed.angular.z = 0.0
                    # while robot is in this state, I keep adding to hit_flag
                    # then, when hit_flag is greater than a certain number, it means it has followed the wall
                    hit_flag += 1
            elif scan.region['front'] < .2:
                speed.linear.x = 0.0
                speed.angular.z = 0.2

            if dist_to_goal < closest_dist:
                closest_dist = dist_to_goal
                closest_point.x = odom.x
                closest_point.y = odom.y
                #hit_flag += 1

            # here if hit_flag is greater than a certain number it shows that the robot has actually followed the wall
            # and if abs(odom.x - hit_point.x) < 0.1 and abs(odom.y - hit_point.y) < 0.1, it means it is not the first
            # time that it is hitting the wall and it has reached that point after circumnavigating the wall.
            # choosing the min value for this certain number depends on loop frequency and the robot's speed
            # I chose 10 because loop frequency is 5, meaning that it takes 0.2s for each loop. The distance it travels
            # in a loop is about 0.2*0.2=0.04 and I want it to stop when abs(odom.x - hit_point.x) < 0.1 and
            # abs(odom.y - hit_point.y) < 0.1, thus, it must travel at least 0.1m not to enter the next if loop by mistake
            # 0.1/0.04 = 2.5 it means it would detect the wall at least 3 more times and adding to previous hit_flag value
            # hit_flag = 1+3 =4. some value between 5 and 10 would be fine.
            if hit_flag > 10:
                if abs(odom.x - hit_point.x) < 0.1 and abs(odom.y - hit_point.y) < 0.1:
                    state = 2

            print "current state: ", state_dict[state]


        elif state == 2:
            # go back to cloest point
            closest_point_x = closest_point.x
            closest_point_y = closest_point.y

            # if it reaches the closest point it will change the state to 0 again.
            if abs(odom.x - closest_point_x) < 0.05 and abs(odom.y - closest_point_y) < 0.05:
                state = 0
            else:
                speed.angular.z = 0.3
                if scan.region['front'] > .3:
                    if scan.region['fright'] > .3:
                        speed.angular.z = - 0.2
                        speed.linear.x = 0.0
                    elif scan.region['fright'] < .3:
                        speed.linear.x = 0.2
                        speed.angular.z = 0.0
                        hit_flag += 1
                elif scan.region['front'] < .2:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.2


            print "current state: ", state_dict[state]


        print("front: %.3f, \t right: %.3f, \t fright: %.3f, \t left: %.3f, \t fleft: %.3f" % (scan.region['front'], scan.region['right'], scan.region['fright'], scan.region['left'], scan.region['fleft']))
        print("hit_point.x: %.3f" % (hit_point.x))
        print("hit_point.y: %.3f" % (hit_point.y))
        print("closest_point_x: %.3f" % (closest_point_x))
        print("closest_point_y: %.3f" % (closest_point_y))
        pub.publish(speed)
        rate.sleep()

# call this function when you press CTRL+C to stop the robot
def stop():
    global pub
    speed = Twist()
    speed.linear.x = 0.0
    speed.angular.z = 0.0

    pub.publish(speed)

if __name__ == '__main__':
    main()
