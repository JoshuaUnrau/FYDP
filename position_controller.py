#!/usr/bin/env python

#TODO:
#1. Get desired angles relative to table for gripping positions
#2.

import rospy
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, TransformStamped
from math import pow, atan2, sqrt
from enum import IntEnum
import math
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
        print("Initialising")
        rospy.init_node('controller')

        # position of the robot
        # self.pose_subscriber = rospy.Subscriber('/vicon/swole_1/swole_1', TransformStamped, self.update_pose)

        self.pose = Twist()
        self.goal = Twist()

        # center point of table
        self.table_pose = Twist()

        # Constants
        # table width of small table
        # 35 mm
        self.TABLE_WIDTH = 0.35
        # for larger table
        # self.TABLE_WIDTH = 0.55
        # assume safe distance from table is around 50 mm
        self.SAFE_GRIPPING_DIST = 0.5
        self.TABLE_POSE_SET = False

        # servo
        self.SERVO_CLOSED = 20
        self.SERVO_OPEN = 65
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
        self.actuator_state = ActuatorState.Stop
        self.servo_state = 25
        self.grabbing_timer_started = False

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.lifter_publisher = rospy.Publisher('desired_lifter_state', Int32, queue_size=10)
        self.servo_publisher = rospy.Publisher('servo', Int32, queue_size=10)

        # test topic for testing leg positions
        self.pose_subscriber = rospy.Subscriber('/vicon/swole_1/swole_1', TransformStamped, self.update_pose)

        # subscribers for table leg positions
        self.table_pose_subscriber = rospy.Subscriber('vicon/small_table/small_table', TransformStamped,
                                                       self.set_table_pose)

        self.loadcell_neg_x_subscriber = rospy.Subscriber('loadcell_neg_x', Int32, self.update_load_cell_neg_x_state)
        self.loadcell_pos_x_subscriber = rospy.Subscriber('loadcell_pos_x', Int32, self.update_load_cell_pos_x_state)
        self.loadcell_y_subscriber = rospy.Subscriber('loadcell_y', Int32, self.update_load_cell_y_state)

        self.linear_actuator_subscriber = rospy.Subscriber('current_lifter_state', Int32,
                                                           self.update_linear_actuator_state)
        self.servo_subscriber = rospy.Subscriber('current_servo_state', Int32, self.update_servo_state)

        print("Waiting for table pose.")
        while(not self.TABLE_POSE_SET):
            pass
        print("Table pose set.")
        self.manageState()

    #     table leg angle testing
    #     self.angle_testing()
    def wrapped_angle(self, current_angle, desired_angle):
        """
        This function calculates the angle that the robot should turn
        to get from the current angle to the desired angle, taking into
        account the wrapping issue.

        Arguments:
        current_angle -- the current angle of the robot, in radians
        desired_angle -- the desired angle of the robot, in radians

        Returns:
        The angle that the robot should turn, in radians.
        """
        # Calculate the angle difference between the desired and current angles
        #Adding the 2 * math.pi removes the posibility of negatives
        # modding by 2 * math.pi restricts to 0, 2pi range
        angle_diff = (desired_angle - current_angle + 2 * math.pi) % (2 * math.pi)

        # Adjust the angle difference to be between -pi and pi
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Return the adjusted angle difference
        return angle_diff

    def angle_testing(self):
        rate = rospy.Rate(10)  # 10hz
        # while not rospy.is_shutdown():
        print("Angle testing")
        print("Robot position", self.pose)
        rate.sleep()

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

    def transform_rotated_point(self, x, y, angle, point_is_x):
        if point_is_x:
            return x * math.cos(angle) - y * math.sin(angle)
        else:
            return x * math.sin(angle) + y * math.cos(angle)

    def set_table_pose(self, data):
        # print("Table leg 1 pose", data)
        # self.table_pose = data

        self.table_pose.linear.x = data.transform.translation.x
        self.table_pose.linear.y = data.transform.translation.y
        _, _, self.table_pose.angular.z, = euler_from_quaternion(data.transform.rotation.x,
                                                                 data.transform.rotation.y,
                                                                 data.transform.rotation.z,
                                                                 data.transform.rotation.w)
        # leg_positions = [self.table_leg_1_pose, self.table_leg_2_pose, self.table_leg_3_pose, self.table_leg_4_pose]

        # for position in leg_positions:
        #     # there are 2 safe grabbing positions for each leg
        #     # mimus, minus offset and minus, plus offset
        angle = self.table_pose.angular.z
        position_1 = Twist()
        position_1.linear.x = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           True) + self.TABLE_WIDTH / 2
        position_1.linear.y = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                         False) + self.SAFE_GRIPPING_DIST

        position_1.angular.z = math.radians(270.0)

        position_2 = Twist()
        position_2.linear.x = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           True) + self.TABLE_WIDTH / 2
        position_2.linear.y = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           False) - self.SAFE_GRIPPING_DIST
        position_2.angular.z = math.radians(90.0)

        position_3 = Twist()
        position_3.linear.x = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           True) - self.TABLE_WIDTH / 2
        position_3.linear.y = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           False) + self.SAFE_GRIPPING_DIST
        position_3.angular.z = math.radians(270.0)

        position_4 = Twist()
        position_4.linear.x = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           True) - self.TABLE_WIDTH / 2
        position_4.linear.y = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           False) - self.SAFE_GRIPPING_DIST
        position_4.angular.z = math.radians(90.0)

        position_5 = Twist()
        position_5.linear.x = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           True) + self.SAFE_GRIPPING_DIST
        position_5.linear.y = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           False) + self.TABLE_WIDTH / 2
        position_5.angular.z = math.radians(180.0)

        position_6 = Twist()
        position_6.linear.x = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           True) + self.SAFE_GRIPPING_DIST
        position_6.linear.y = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           False) - self.TABLE_WIDTH / 2
        position_6.angular.z = math.radians(180.0)

        position_7 = Twist()
        position_7.linear.x = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           True) - self.SAFE_GRIPPING_DIST
        position_7.linear.y = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           False) + self.TABLE_WIDTH / 2
        position_7.angular.z = math.radians(0.0)

        position_8 = Twist()
        position_8.linear.x = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           True) - self.SAFE_GRIPPING_DIST
        position_8.linear.y = self.transform_rotated_point(self.pose.linear.x,
                                                           self.pose.linear.y, angle,
                                                           False) - self.TABLE_WIDTH / 2
        position_8.angular.z = math.radians(0.0)

        #     set all in table positions array
        self.table_grab_positions = [position_1, position_2, position_3,
                                     position_4, position_5, position_6,
                                     position_7, position_8]

        #Modify grab positions by table pose
        i = 0
        for twist in self.table_grab_positions:
            self.table_grab_positions[i].angular.z = twist.angular.z + self.table_pose.angular.z
            i += 1

        self.TABLE_POSE_SET = True

    def update_pose(self, data):
        self.pose.linear.x = data.transform.translation.x
        self.pose.linear.y = data.transform.translation.y
        _, _, self.pose.angular.z = euler_from_quaternion(data.transform.rotation.x,
                                                          data.transform.rotation.y,
                                                          data.transform.rotation.z,
                                                          data.transform.rotation.w)

    def euclidean_distance(self, pose1, pose2):
        return sqrt(pow((pose1.linear.x - pose2.linear.x), 2) +
                        pow((pose1.linear.y - pose2.linear.y), 2))
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
        min = 9999
        index = 0
        i = 0
        #Positions are twists not poses
        for position in self.table_grab_positions:
            distance_to_position = self.euclidean_distance(position, self.pose)
            #print(distance_to_position)
            if min > distance_to_position:
                min = distance_to_position
                index = i
            i += 1
        print(index)
        return index

    def get_table_pose(self, leg_index):
        return self.table_grab_positions[leg_index]

    def setMinimumSpeed(self, vel_msg, minimum_speed):
        if (self.euclidean_distance_xy(vel_msg.linear.x, vel_msg.linear.y) < minimum_speed):
            speed = self.euclidean_distance_xy(vel_msg.linear.x, vel_msg.linear.y)
            if speed == 0:
                pass
            else:
                vel_msg.linear.x = vel_msg.linear.x / speed * minimum_speed
                vel_msg.linear.y = vel_msg.linear.y / speed * minimum_speed
        return vel_msg

    def setMaximumSpeed(self, vel_msg, maximum_speed):
        if (self.euclidean_distance_xy(vel_msg.linear.x, vel_msg.linear.y) > maximum_speed):
            speed = self.euclidean_distance_xy(vel_msg.linear.x, vel_msg.linear.y)
            if speed == 0:
                pass
            else:
                vel_msg.linear.x = vel_msg.linear.x / speed * maximum_speed
                vel_msg.linear.y = vel_msg.linear.y / speed * maximum_speed
        return vel_msg

    def manageState(self):
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.closest_table_leg = self.get_closest_table_leg()
            self.goal = self.get_table_pose(self.closest_table_leg)
            #self.goal = self.table_pose
            #print(self.table_pose)
            #print(self.pose)
            #print(self.goal)
            required_table_angle = self.goal.angular.z
            distance_to_goal = self.euclidean_distance(self.goal, self.pose)
            print("Goal distance: " + str(distance_to_goal))

            self.alignment_error = self.wrapped_angle(self.pose.angular.z, required_table_angle)
            self.grabbing_force = self.load_cell_pos_x - self.load_cell_neg_x

            print("Goal distance: " + str(distance_to_goal))
            print("Closest leg: " + str(self.closest_table_leg))
            print("Alignment error: " + str(self.alignment_error))
            print("Pose z: " + str(self.pose.angular.z))
            print("Goal z: " + str(self.goal.angular.z))
            print("Table z: " + str(self.table_pose.angular.z))


            # Conditions are in reverse order as the higher precision conditions will be
            # checked first and be false kicking the algorithm down to the next condition

            # Global Reset grabbing timer
            if (self.grabbing_force > self.TABLE_GRABBING_FORCE):
                self.grabbing_timer_started = False

            # Special condition for startup
            # if (not self.startUpCompleted):
            #     print("State: Startup")
            #     #print(abs(self.servo_state - self.SERVO_OPEN))
            #     print("Servo: " + str(self.servo_state))
            #     print("Actuator: " + str(self.actuator_state))
            #     if (self.actuator_state == ActuatorState.Down and
            #             abs(self.servo_state - self.SERVO_OPEN) < self.SERVO_TOLERANCE):
            #         self.startUpCompleted = True
            #     else:
            #         self.state_startup()
            if (self.grabbing_force > self.TABLE_GRABBING_FORCE and
                  self.actuator_state == ActuatorState.Up):
                print("State: Move table")
                self.state_move_table()
            elif (self.grabbing_force > self.TABLE_GRABBING_FORCE and
                  timer() - self.grabbing_timer_start < self.GRABBING_TIMEOUT):
                print("State: Lift table")
                self.state_lift_table()
            elif (distance_to_goal <= self.PRECISE_DISTANCE_TOLERANCE and
                  self.alignment_error < self.ALIGNMENT_ERROR_TOLERANCE):
                print("State: Grab table")
                self.state_grab_table()
            elif (self.alignment_error < self.ALIGNMENT_ERROR_TOLERANCE):
                print("State: Moving to grabbing position")
                self.state_move_to_grabbing_position()
            elif (distance_to_goal <= self.ROUGH_DISTANCE_TOLERANCE):
                print("State: Aligning to goal")
                self.state_align_to_goal()
            elif (distance_to_goal > self.ROUGH_DISTANCE_TOLERANCE):
                print("State: Moving to goal")
                self.state_move_to_goal()
            self.rate.sleep()

    def state_startup(self):
        self.servo_publisher.publish(self.SERVO_OPEN)
        self.lifter_publisher.publish(ActuatorState.Down)

    def state_move_to_goal(self):
        self.goal = self.goal

        error_x = self.pose.linear.x - self.goal.linear.x
        error_y = self.pose.linear.y - self.goal.linear.y

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
        vel_msg.angular.z = self.alignment_error * self.ALIGNMENT_P

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
        table_pose = self.get_table_pose(self.closest_table_leg)

        error_x = self.pose.linear.x - table_pose.linear.x
        error_y = self.pose.linear.y - table_pose.linear.y

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
