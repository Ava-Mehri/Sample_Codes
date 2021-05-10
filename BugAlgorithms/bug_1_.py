#Ava Mehri, UIC, amehri2@uic.edu, 2021
#Bug1 algorithm
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
        self.region['front'] = min(self.ranges[0:72])
        self.region['fright'] = min(self.ranges[73:144])
        self.region['right'] = min(self.ranges[145:216])
        self.region['left'] = min(self.ranges[217:288])
        self.region['fleft'] = min(self.ranges[288:360])
        #self.region['front'] = min(self.ranges[0:30])
        #self.region['fright'] = min(self.ranges[40:80])
        #self.region['right'] = min(self.ranges[140:180])
        #self.region['left'] = min(self.ranges[220:260])
        #self.region['fleft'] = min(self.ranges[290:330])

        #print "range[90]: ", msg.ranges[90]


# divide robot motion in 3 scenario
state_dict = {
    0: 'go to goal',
    1: 'circumnavigate obstacle',
    2: 'go back to closest point',
    3: 'wall detected',
}
# define initial scenario
#state = 0


def main():
    global pub

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
    goal.x = 0.0
    goal.y = 5.0

    # arbitrary far away coordinate from goal
    closest_point = Point()
    closest_point.x = 1000
    closest_point.y = 1000

    # arbitrary large number representing inf distance to goal
    closest_dist = 1000 
    # Variable that stores the coordinate of hit point when you 
    # encounter obstacle for the first time
    hit_point = Point()
    hit_x = []
    hit_y = []
    n = 0
    hit_flag = 0

    while not rospy.is_shutdown():
        # initialize speed as Twist topic
        d = 0.15
        # When all measurements are greater than some certain value.
        if scan.region['front'] > d and scan.region['fleft'] > d and scan.region['fright'] > d and hit_flag == 0:
            state = 0
        #elif abs(odom.x-closest_point.x) < 0.2 and abs(odom.y-closest_point.y) < 0.2:
            #state = 0
        # When front and front-left are greater than a certain value and only front-right is less than that.
        elif scan.region['front'] > d and scan.region['left'] < d and scan.region['right'] > d:
            state = 1
        #elif scan.region['front'] > 0.5 and scan.region['fleft'] < d and scan.region['fright'] > d:
            #state = 1
        # When the distance of the robot to the wall in these 3 regions is less than a certain value. Front,
        # front-right, front-left
        elif scan.region['front'] < d and scan.region['fleft'] < d and scan.region['fright'] < d:
            state = 3
        # When front and front-right are less than a certain value, but front-left is greater than that.
        # It means the right side of the robot is facing the wall.
        elif scan.region['front'] < d and scan.region['left'] > d and scan.region['right'] < d:
            state = 3
        # When front and front-left are less than a certain value, but front-right is greater than that.
        # It means the left side of the robot is facing the wall.
        elif scan.region['front'] < d and scan.region['fleft'] < d and scan.region['fright'] > d:
            state = 3
        # When front-right and front-left are less than a certain value, but front is greater than that.
        # It means the front side of the robot is facing the wall.
        elif scan.region['front'] < d and scan.region['left'] > d and scan.region['right'] > d:
            state = 3
        elif n > 100 and abs(odom.x-hit_point.x) < 0.1 and abs(odom.y-hit_point.y) < 0.1:
            state = 2

        else:
            state = 3

        # TODO:

        # Decide what to do for the robot in each of these states:


        if state == 0:
            # go to goal state.

            inc_x = goal.x - odom.x
            inc_y = goal.y - odom.y
            angle_to_goal = atan2(inc_y, inc_x)
            angle_diff = angle_to_goal - odom.theta
            # find the distance to the goal from current location
            # dist_diff = sqrt(inc_x * inc_x + inc_y * inc_y)

            # If the robot's orientation is not towards the target, we only rotate the robot until its
            # orientation is towards the target
            # trying different small angular velocities, 0.3 works fine
            if abs(angle_diff) > 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
            # Once the robot is headed towards the target, we just move straight towards the goal until reached
            # I designed the controller such that the closer robot gets to the gol the slower it moves.
            else:
                speed.angular.z = 0.0
                speed.linear.x = 0.3
            '''
            Hint: 
                Here robot should go towards a the goal unless it encounters an obstacle. When it encounters the wall
                it should change the state to "circumnavigate obstacle".

                should be bunch of if-else statements

            '''

            print "current state: ", state_dict[state]

        elif state == 3:
            # wall detected.
            hit_flag = 1
            hit_x += [odom.x]
            hit_y += [odom.y]
            n += 1

            hit_point.x = hit_x[0]
            hit_point.y = hit_y[0]
            speed.linear.x = 0.0
            speed.angular.z = 0.5
            print "current state: ", state_dict[state]



        elif state == 1:
            # circumnavigate obstacle.
            inc_x = goal.x - odom.x
            inc_y = goal.y - odom.y
            #if m ==0 and abs(odom.x-hit_point.x) < 0.05 and abs(odom.y-hit_point.y) < 0.05:
                #speed.linear.x = 0.4
                #speed.angular.z = 0.0
            #elif abs(odom.x-hit_point.x) > 0.05 and abs(odom.y-hit_point.y) > 0.05:
                #m += 1
            speed.linear.x = 0.3
            speed.angular.z = 0.0
            dist_to_goal = sqrt(inc_x * inc_x + inc_y * inc_y)
            if dist_to_goal < closest_dist:
                closest_dist = dist_to_goal
                closest_point.x = odom.x
                closest_point.y = odom.y

            '''
            Hint: 
                Here robot should turn right/left based on your choice. And, circumnavigate the obstacle using wall following
                algorithm from previous project. While in this state record closest point to goal where you can head towards goal.
                This state terminates when
                you reach the same point when you hit the obstacle.

                Finally, do not forget to change the state!

                should be bunch of if-else statements

            '''
            # find current distance to goal
            #dist_to_goal = blah

            #if dist_to_goal<closest_dist:
            '''Update closest distance and record corresponding coordinate to closest_point variable'''

            print "current state: ", state_dict[state]


        elif state == 2:
            # go back to cloest point
            if abs(odom.x-closest_point.x) > 0.05 and abs(odom.y-closest_point.y) > 0.05:
                speed.linear.x = 0.4
                speed.angular.z = 0.0
            '''
            Hint: 
                Here robot should go back to closest point encountered in state 1. Once you reach that point, change the state
                to go to goal.

                should be bunch of if-else statements
            '''

            print "current state: ", state_dict[state]


        #print scan.regions
        print("front: %.3f, \t right: %.3f, \t fright: %.3f, \t left: %.3f, \t fleft: %.3f" % (scan.region['front'], scan.region['right'], scan.region['fright'], scan.region['left'], scan.region['fleft']))
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
