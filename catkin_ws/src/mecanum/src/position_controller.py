#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, TransformStamped
from math import pow, atan2, sqrt
from enum import IntEnum

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

class ActuatorState(IntEnum):
    Unknown = 0
    Up = 1
    Down = 2
    Moving = 3

class Position_Controller:

    def __init__(self):
        # Creates a node with name 'Position_Controller_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('controller')

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.goal_subscriber = rospy.Subscriber('vicon/test_table_leg_1/test_table_leg_1', TransformStamped, self.set_goal)
        # test topic for testing leg positions
        self.pose_subscriber = rospy.Subscriber('/vicon/test/test', TransformStamped, self.update_pose)

        # subscribers for table leg positions
        self.table_leg_1_subscriber = rospy.Subscriber('vicon/test_table_leg_1/test_table_leg_1', TransformStamped,
                                                self.set_table_leg_1_pose)
        self.table_leg_2_subscriber = rospy.Subscriber('vicon/test_table_leg_2/test_table_leg_2', TransformStamped,
                                                   self.set_table_leg_2_pose)

        self.loadcell_neg_x_subscriber = rospy.Subscriber('loadcell_neg_x', Int32, self.update_pose)
        self.loadcell_pos_x_subscriber = rospy.Subscriber('loadcell_pos_x', Int32, self.update_pose)
        self.loadcell_y_subscriber = rospy.Subscriber('loadcell_y', Int32, self.update_pose)

        self.linear_actuator_subcriber = rospy.Subscriber('current_lifter_state', Int32, self.update_pose)
        # position of the robot
        # self.pose_subscriber = rospy.Subscriber('/vicon/swole_1/swole_1', TransformStamped, self.update_pose)

        self.pose = TransformStamped()
        self.goal = TransformStamped()
        self.table_leg_1_pose = TransformStamped()
        self.table_leg_2_pose = TransformStamped()
        # leg 3 adjacent to leg 1
        self.table_leg_3_pose = TransformStamped()
        # leg 4 adjacent to leg 2
        self.table_leg_4_pose = TransformStamped()

        # table width of small table
        # 35 mm
        self.table_width = 0.35
        # for larger table
        # self.table_width = 0.55

        self.startUpCompleted = False

        self.load_cell_neg_x = 0
        self.load_cell_pos_x = 0
        self.load_cell_y = 0

        print("Goal reset", self.goal)
        self.manageState()

    def update_linear_actuator_state(self, data):
        self.actuator_state = data.data

    def update_load_cell_neg_x_state(self, data):
        self.load_cell_neg_x = data.data

    def update_load_cell_pos_x_state(self, data):
        self.load_cell_pos_x = data.data

    def update_load_cell_y_state(self, data):
        self.load_cell_y = data.data

    def set_table_leg_1_pose(self, data):
        # print("Table leg 1 pose", data)
        self.table_leg_1_pose = data

        # #     also set leg 3's pose
        # self.table_leg_3_pose =

    def set_table_leg_2_pose(self, data):
        print("Table leg 2 pose", data)
        self.table_leg_2_pose = data

    def set_goal(self, data):
        self.goal = data

    def update_pose(self, data):
        self.pose = data

    def euclidean_distance(self, pose1, pose2):
        return sqrt(pow((pose1.transform.translation.x - pose2.transform.translation.x), 2) +
                    pow((pose1.transform.translation.y - pose2.transform.translation.y), 2))

    def euclidean_distance_xy(self, x, y):
        return sqrt(pow((x), 2) +
                    pow((y), 2))

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - euler_from_quaternion(self.pose.x, self.pose.y, self.pose.z, self.pose.w).roll_x)

    def manageState(self):
        self.rate = rospy.Rate(10)
        rough_distance_tolerance = 0.7
        precise_distance_tolerance = 0.05
        alignment_error_tolerance = 1
        table_grabbing_force = 1000
        servo_open = 10
        servo_closed = 80
        servo_tolerance = 5
        while not rospy.is_shutdown():
            distance_to_goal = self.euclidean_distance(self.goal, self.pose)
            self.alignment_error = self.pose.transform.rotation.z - required_table_angle
            self.actuator_state =
            self.grabbing_force =
            self.servo_state =
            #Conditions are in reverse order as the higher precision conditions will be
            #checked first and be false kicking the algorithm down to the next condition

            #Special condition for startup
            if(not self.startupCompleted):
                if(self.actuator_state == ActuatorState.Down and
                   abs(self.servo_state - servo_open) < servo_tolerance):
                    self.startUpCompleted = True
            elif(self.grabbing_force > table_grabbing_force and
                    self.actuator_state == ActuatorState.Up):
                self.move_table()
            elif(self.grabbing_force > table_grabbing_force):
                self.lift_table()
            elif (distance_to_goal <= precise_distance_tolerance and
                    self.alignment_error < alignment_error_tolerance):
                self.grab_table()
            elif (self.alignment_error < alignment_error_tolerance):
                self.move_to_grabbing_position()
            elif (distance_to_goal <= distance_tolerance):
                self.align_to_goal()
            elif (distance_to_goal > distance_tolerance):
                self.move_to_goal()
            self.rate.sleep()

    def move_to_goal(self):
        self.goal = self.goal

        p = 0.35
        error_x = self.pose.transform.translation.x - self.goal.transform.translation.x
        error_y = self.pose.transform.translation.y - self.goal.transform.translation.y

        print("Pose error X: " + str(error_x) + " , Y: " + str(error_y) + " ")
        vel_msg = Twist()

        # Porportional controller.
        # https://en.wikipedia.org/wiki/Proportional_control
        vel_msg.linear.x = error_x * p
        vel_msg.linear.y = error_y * p
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        #Minimum_speed
        minimum_speed = 0.15
        if(self.euclidean_distance_xy(vel_msg.linear.x, vel_msg.linear.y) < 0.15):
            speed = self.euclidean_distance_xy(vel_msg.linear.x, vel_msg.linear.y)
            if speed == 0:
                pass
            else:
                vel_msg.linear.x = vel_msg.linear.x / speed * minimum_speed
                vel_msg.linear.y = vel_msg.linear.y / speed * minimum_speed

        maximum_speed = 0.5
        #Maximum speed limit
        if(vel_msg.linear.x > maximum_speed):
            vel_msg.linear.x = maximum_speed
        if (vel_msg.linear.x < -maximum_speed):
            vel_msg.linear.x = -maximum_speed
        if (vel_msg.linear.y > maximum_speed):
            vel_msg.linear.y = maximum_speed
        if (vel_msg.linear.y < -maximum_speed):
            vel_msg.linear.y = -maximum_speed

        self.velocity_publisher.publish(vel_msg)

    def align_to_goal(self):
        table_leg_index = self.get_closest_table_leg()

        required_table_angle = self.get_alignment_to_table_position(table_leg_index)

        p = 0.35
        vel_msg.angular.z = self.alignment_error*p

    # This selects the closest grabbing position
    def get_closest_table_leg(self):
        # Array of 8 grabbing positions (2 per leg).
        table_grab_positions = [Pose * 8]

        min = 0
        index = 0
        for position, i in table_grab_positions:
            distance_to_position = self.euclidean_distance(position, self.pose)
            if min > distance_to_position:
                min = distance_to_position
                index = i

        return index

    def get_alignment_to_table_position(self, leg_index):
        pass

    def move_to_grabbing_position(self):
        pass

    def grab_table(self):
        pass

    def lift_table(self):
        pass

    def move_table(self):
        pass

if __name__ == '__main__':
    try:
        x = Position_Controller()
    except rospy.ROSInterruptException:
        pass
