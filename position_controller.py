#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, TransformStamped
from math import pow, atan2, sqrt
from enum import IntEnum
from timeit import default_timer as timer


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
    Stop = 0
    Up = 1
    Down = 2
    Moving = 3

class LeaderFollowerState(IntEnum):
    Unknown = 0
    Leader = 1
    Follower = 2

class Position_Controller:

    def __init__(self):
        # Creates a node with name 'Position_Controller_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('controller')

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.lifter_publisher = rospy.Publisher('lifter_state', int, queue_size=10)
        self.servo_publisher = rospy.Publisher('servo', int, queue_size=10)

        self.goal_subscriber = rospy.Subscriber('vicon/test_table_leg_1/test_table_leg_1', TransformStamped,
                                                self.set_goal)
        # test topic for testing leg positions
        self.pose_subscriber = rospy.Subscriber('/vicon/test/test', TransformStamped, self.update_pose)

        # subscribers for table leg positions
        self.table_leg_1_subscriber = rospy.Subscriber('vicon/test_table_leg_1/test_table_leg_1', TransformStamped,
                                                       self.set_table_leg_1_pose)
        self.table_leg_2_subscriber = rospy.Subscriber('vicon/test_table_leg_2/test_table_leg_2', TransformStamped,
                                                       self.set_table_leg_2_pose)

        self.loadcell_neg_x_subscriber = rospy.Subscriber('loadcell_neg_x', Int32, self.update_load_cell_neg_x_state())
        self.loadcell_pos_x_subscriber = rospy.Subscriber('loadcell_pos_x', Int32, self.update_load_cell_pos_x_state())
        self.loadcell_y_subscriber = rospy.Subscriber('loadcell_y', Int32, self.update_load_cell_y_state())

        self.linear_actuator_subscriber = rospy.Subscriber('current_lifter_state', Int32,
                                                           self.update_linear_actuator_state())
        self.servo_subscriber = rospy.Subscriber('current_lifter_state', Int32, self.update_pose)

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

        # Constants

        # table width of small table
        # 35 mm
        self.TABLE_WIDTH = 0.35
        # for larger table
        # self.table_width = 0.55
        # assume safe distance from table is around 50 mm
        self.SAFE_GRIPPING_DIST = 0.5

        # servo
        self.SERVO_CLOSED = 80
        self.SERVO_OPEN = 0
        self.TABLE_GRABBING_FORCE = 1000
        self.SERVO_TOLERANCE = 5
        self.GRABBING_TIMEOUT = 5

        # Positioning
        self.ROUGH_DISTANCE_TOLERANCE = 0.7
        self.PRECISE_DISTANCE_TOLERANCE = 0.05
        self.ALIGNMENT_ERROR_TOLERANCE = 1
        self.TRANSLATION_P = 0.35
        self.ALIGNMENT_P = 0.1
        self.FORCE_P = 0.1

        # Array holding all of the positions that the robot should go to for grabbing
        self.table_grab_positions = []
        self.startUpCompleted = False
        self.load_cell_neg_x = 0
        self.load_cell_pos_x = 0
        self.load_cell_y = 0
        self.closest_table_leg = 0
        self.leaderFollowerState = LeaderFollowerState.Unknown
        self.actuator_state = ActuatorState.Unknown
        self.servo_state = 0
        self.grabbing_timer_started = False

        print("Goal reset", self.goal)
        self.manageState()

    def set_grabbing_position(self):
        leg_positions = [self.table_leg_1_pose, self.table_leg_2_pose, self.table_leg_3_pose, self.table_leg_4_pose]

        for position in leg_positions:
            # there are 2 safe grabbing positions for each leg
            # mimus, minus offset and minus, plus offset

            position_1 = position
            position_2 = position
            position_1.transform.translation.x -= self.SAFE_GRIPPING_DIST
            position_1.transform.translation.y -= self.SAFE_GRIPPING_DIST
            position_2.transform.translation.x += self.SAFE_GRIPPING_DIST
            position_2.transform.translation.y -= self.SAFE_GRIPPING_DIST

            self.table_grab_positions.append(position_1)
            self.table_grab_positions.append(position_2)

    def find_and_set_other_leg_coords(self):
        x1 = self.table_leg_1_pose.transform.translation.x
        y1 = self.table_leg_1_pose.transform.translation.y

        x2 = self.table_leg_2_pose.transform.translation.x
        y2 = self.table_leg_2_pose.transform.translation.y
        # calculate the length of the diagonal
        diag_length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        # calculate the angle of the diagonal
        diag_angle = math.atan2(y2 - y1, x2 - x1)
        # calculate the half diagonal length (i.e. from the center to a corner)
        half_diag_length = diag_length / 2.0
        # calculate the coordinates of the center of the square
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        # calculate the coordinates of the other two corners
        corner1_x = center_x + half_diag_length * math.cos(diag_angle + math.pi / 2.0)
        corner1_y = center_y + half_diag_length * math.sin(diag_angle + math.pi / 2.0)
        corner2_x = center_x + half_diag_length * math.cos(diag_angle - math.pi / 2.0)
        corner2_y = center_y + half_diag_length * math.sin(diag_angle - math.pi / 2.0)

        # set positions of other 2 legs
        self.table_leg_3_pose.transform.translation.x = corner1_x
        self.table_leg_3_pose.transform.translation.y = corner1_y

        self.table_leg_4_pose.transform.translation.x = corner2_x
        self.table_leg_4_pose.transform.translation.y = corner2_y

    def update_linear_actuator_state(self, data):
        self.actuator_state = data.data

    def update_servo_state(self, data):
        self.servo_state = data.data

    def update_load_cell_neg_x_state(self, data):
        self.load_cell_neg_x = data.data

    def update_load_cell_pos_x_state(self, data):
        self.load_cell_pos_x = data.data

    def update_load_cell_y_state(self, data):
        self.load_cell_y = data.data

    def set_table_leg_1_pose(self, data):
        # print("Table leg 1 pose", data)
        self.table_leg_1_pose = data

        # update other leg coordinates and set grabbing positions
        self.find_and_set_other_leg_coords()
        self.set_grabbing_position()

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
        return constant * (self.steering_angle(goal_pose) - euler_from_quaternion(self.pose.x, self.pose.y, self.pose.z,
                                                                                  self.pose.w).roll_x)

    # This selects the closest grabbing position
    def get_closest_table_leg(self):
        # Array of 8 grabbing positions (2 per leg).
        min = 0
        index = 0
        for position, i in self.table_grab_positions:
            distance_to_position = self.euclidean_distance(position, self.pose)
            if min > distance_to_position:
                min = distance_to_position
                index = i

        return index

    def get_closest_table_pose(self, leg_index):
        return self.table_grab_positions[leg_index]

    def setMinimumSpeed(self, vel_msg, mimumum_speed):
        if (self.euclidean_distance_xy(vel_msg.linear.x, vel_msg.linear.y) < mimumum_speed):
            speed = self.euclidean_distance_xy(vel_msg.linear.x, vel_msg.linear.y)
            if speed == 0:
                pass
            else:
                vel_msg.linear.x = vel_msg.linear.x / speed * minimum_speed
                vel_msg.linear.y = vel_msg.linear.y / speed * minimum_speed
        return vel_msg

    def setMaximumSpeed(self, vel_msg, maximum_speed):
        # Maximum speed limit
        if (vel_msg.linear.x > maximum_speed):
            vel_msg.linear.x = maximum_speed
        if (vel_msg.linear.x < -maximum_speed):
            vel_msg.linear.x = -maximum_speed
        if (vel_msg.linear.y > maximum_speed):
            vel_msg.linear.y = maximum_speed
        if (vel_msg.linear.y < -maximum_speed):
            vel_msg.linear.y = -maximum_speed
        return vel_msg

    def manageState(self):
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            distance_to_goal = self.euclidean_distance(self.goal, self.pose)

            self.closest_table_leg = self.get_closest_table_leg()
            required_table_angle = self.get_alignment_to_table_position(self.closest_table_leg)

            self.alignment_error = self.pose.transform.rotation.z - required_table_angle
            self.grabbing_force = self.load_cell_pos_x - self.load_cell_neg_x
            # Conditions are in reverse order as the higher precision conditions will be
            # checked first and be false kicking the algorithm down to the next condition

            # Global Reset grabbing timer
            if (self.grabbing_force > self.TABLE_GRABBING_FORCE):
                self.grabbing_timer_started = False

            # Special condition for startup
            if (not self.startupCompleted):
                if (self.actuator_state == ActuatorState.Down and
                        abs(self.servo_state - self.SERVO_OPEN) < self.SERVO_TOLERANCE):
                    self.startUpCompleted = True
                else:
                    self.state_startup()
            elif (self.grabbing_force > self.TABLE_GRABBING_FORCE and
                  self.actuator_state == ActuatorState.Up):
                self.state_move_table()
            elif (self.grabbing_force > self.TABLE_GRABBING_FORCE and
                  timer() - self.grabbing_timer_start < self.GRABBING_TIMEOUT):
                self.state_lift_table()
            elif (distance_to_goal <= self.PRECISE_DISTANCE_TOLERANCE and
                  self.alignment_error < self.ALIGNMENT_ERROR_TOLERANCE):
                self.state_grab_table()
            elif (self.alignment_error < self.ALIGNMENT_ERROR_TOLERANCE):
                self.state_move_to_grabbing_position()
            elif (distance_to_goal <= self.distance_tolerance):
                self.state_align_to_goal()
            elif (distance_to_goal > distance_tolerance):
                self.state_move_to_goal()
            self.rate.sleep()

    def state_startup(self):
        self.servo_publisher.publish(0)
        self.lifter_publisher.publish(ActuatorState.Down)

    def state_move_to_goal(self):
        self.goal = self.goal

        error_x = self.pose.transform.translation.x - self.goal.transform.translation.x
        error_y = self.pose.transform.translation.y - self.goal.transform.translation.y

        print("Pose error X: " + str(error_x) + " , Y: " + str(error_y) + " ")
        vel_msg = Twist()

        # Porportional controller.
        # https://en.wikipedia.org/wiki/Proportional_control
        vel_msg.linear.x = error_x * self.TRANSLATION_P
        vel_msg.linear.y = error_y * self.TRANSLATION_P
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Minimum_speed
        self.setMinimumSpeed(vel_msg, 0.15)

        self.setMaximumSpeed(vel_msg, 0.5)

        self.velocity_publisher.publish(vel_msg)

    def state_align_to_goal(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = self.alignment_error * self.ALIGNMENT_P

        self.velocity_publisher.publish(vel_msg)

    def state_move_to_grabbing_position(self):
        self.goal = self.goal
        table_pose = self.get_closest_table_pose(self.closest_table_leg)

        error_x = self.pose.transform.translation.x - table_pose.transform.x
        error_y = self.pose.transform.translation.y - table_pose.transform.y

        print("Pose error X: " + str(error_x) + " , Y: " + str(error_y) + " ")
        vel_msg = Twist()

        # Porportional controller.
        # https://en.wikipedia.org/wiki/Proportional_control
        vel_msg.linear.x = error_x * self.TRANSLATION_P
        vel_msg.linear.y = error_y * self.TRANSLATION_P
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Minimum_speed = Maximum speed
        self.setMinimumSpeed(vel_msg, 0.10)
        self.setMaximumSpeed(vel_msg, 0.10)

        self.velocity_publisher.publish(vel_msg)

    def state_grab_table(self):
        if self.grabbing_timer_started == False:
            self.grabbing_timer_start = timer()
            self.grabbing_timer_started = True
        self.servo_publisher.publish(self.SERVO_CLOSED)

    def state_lift_table(self):
        self.lifter_publisher(ActuatorState.Up)

    def state_move_table(self):
        if self.leaderFollowerState == LeaderFollowerState.Unknown:
            pass
        if self.leaderFollowerState == LeaderFollowerState.Leader:
            pass
        if self.leaderFollowerState == LeaderFollowerState.Follower:
            x_force = self.load_cell_pos_x - self.load_cell_neg_x
            # Set Linear velocity in the x-axis.
            vel_msg = Twist()

            vel_msg.linear.x = x_force * self.FORCE_P
            vel_msg.linear.y = self.load_cell_y * self.FORCE_P
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.alignment_error * self.ALIGNMENT_P

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        x = Position_Controller()
    except rospy.ROSInterruptException:
        pass
