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
import time
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

class RobotState(IntEnum):
    Occuluded = 8
    Init = 7
    MoveTable = 6
    LiftTable = 5
    GrabTable = 4
    TableGrabPosition = 3
    PreciseMove = 2
    AlignGrab = 1
    RoughMove = 0

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
        self.gripper_position = Twist()

        # center point of table
        self.table_pose = Twist()

        # Constants
        # table width of small table
        # 35 mm
        self.TABLE_WIDTH = 0.35
        # for larger table
        # self.TABLE_WIDTH = 0.55
        # assume safe distance from table is around 50 mm
        self.SAFE_GRIPPING_DIST = 0.4
        self.TABLE_POSE_SET = False

        # servo
        self.SERVO_CLOSED = 20
        self.SERVO_OPEN = 65
        self.TABLE_GRABBING_FORCE = 1000
        self.SERVO_TOLERANCE = 5
        self.GRABBING_TIMEOUT = 5

        # Positioning
        self.ROUGH_DISTANCE_TOLERANCE = 0.25
        self.PRECISE_DISTANCE_TOLERANCE = 0.05
        self.ALIGNMENT_ERROR_TOLERANCE = 0.1
        self.TRANSLATION_P = 2 #0.35
        self.ALIGNMENT_P = 4
        self.FORCE_P = 0.1
        self.error_x = 999
        self.error_y = 999
        self.TABLE_OFFSET_X = 0
        self.TABLE_OFFSET_Y = 0
        self.ROBOT_OFFSET_X = -0.06
        self.ROBOT_OFFSET_Y = 0.06
        self.GRIPPER_OFFSET_X = 0.02
        self.GRIPPER_OFFSET_Y = -0.226
        self.ROBOT_OFFSET_AZ = 0 #-2.42049522

        #P_CONTROLLER SPEED CONSTAINTS
        self.MINIMUM_SPEED = 0.4
        self.MAXIMUM_SPEED = 1 #1.5
        self.MINIMUM_ANGULAR_VEL = 0.2
        self.MAXIMUM_ANGULAR_VEL = 1

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
        self.grabbing_timer_start = 0
        self.grabbing_timer_started = False
        self.robotState = RobotState.Init
        self.in_grabbing_position = False

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

    def transform_rotated_point(self,point, x, y, angle):
        new_point = Twist()
        new_point.linear.x = point.linear.x + x * math.cos(angle) - y * math.sin(angle)
        new_point.linear.y = point.linear.y + x * math.sin(angle) + y * math.cos(angle)
        return new_point

    def set_table_pose(self, data):
        # print("Table leg 1 pose", data)
        # self.table_pose = data

        self.table_pose.linear.x = data.transform.translation.x
        self.table_pose.linear.y = data.transform.translation.y
        _, _, self.table_pose.angular.z, = euler_from_quaternion(data.transform.rotation.x,
                                                                 data.transform.rotation.y,
                                                                 data.transform.rotation.z,
                                                                 data.transform.rotation.w)
        self.table_pose.angular.z = (self.table_pose.angular.z + 2*math.pi)%(2*math.pi)
        # leg_positions = [self.table_leg_1_pose, self.table_leg_2_pose, self.table_leg_3_pose, self.table_leg_4_pose]

        # for position in leg_positions:
        #     # there are 2 safe grabbing positions for each leg
        #     # mimus, minus offset and minus, plus offset
        angle = self.table_pose.angular.z
        x = [1, 1, -1, -1, 1, 1, -1, -1]
        y = [1, ]
        y_axis = -1
        position_0 = self.transform_rotated_point(self.table_pose,
                                                  self.TABLE_WIDTH / 2,
                                                  y_axis*(self.TABLE_WIDTH / 2 + self.SAFE_GRIPPING_DIST),
                                                  angle)
        position_0.angular.z = math.radians(180)

        position_1 = self.transform_rotated_point(self.table_pose,
                                                  self.TABLE_WIDTH / 2 + self.SAFE_GRIPPING_DIST,
                                                  y_axis*(self.TABLE_WIDTH / 2),
                                                  angle)
        position_1.angular.z = math.radians(90.0)

        position_2 = self.transform_rotated_point(self.table_pose,
                                                  self.TABLE_WIDTH / 2 + self.SAFE_GRIPPING_DIST,
                                                  y_axis*(-self.TABLE_WIDTH / 2),
                                                  angle)
        position_2.angular.z = math.radians(90)

        position_3 = self.transform_rotated_point(self.table_pose,
                                                  self.TABLE_WIDTH / 2,
                                                  y_axis*(-self.TABLE_WIDTH / 2 - self.SAFE_GRIPPING_DIST),
                                                  angle)
        position_3.angular.z = math.radians(0)

        position_4 = self.transform_rotated_point(self.table_pose,
                                                  -self.TABLE_WIDTH / 2,
                                                  y_axis*(-self.TABLE_WIDTH / 2 - self.SAFE_GRIPPING_DIST),
                                                  angle)
        position_4.angular.z = math.radians(0)

        position_5 = self.transform_rotated_point(self.table_pose,
                                                  -self.TABLE_WIDTH / 2 - self.SAFE_GRIPPING_DIST,
                                                  y_axis*(-self.TABLE_WIDTH / 2),
                                                  angle)
        position_5.angular.z = math.radians(270.0)

        position_6 = self.transform_rotated_point(self.table_pose,
                                                  -self.TABLE_WIDTH / 2 - self.SAFE_GRIPPING_DIST,
                                                  y_axis*(self.TABLE_WIDTH / 2),
                                                  angle)
        position_6.angular.z = math.radians(270.0)

        position_7 = self.transform_rotated_point(self.table_pose,
                                                  -self.TABLE_WIDTH / 2,
                                                  y_axis*(self.TABLE_WIDTH / 2 + self.SAFE_GRIPPING_DIST),
                                                  angle)
        position_7.angular.z = math.radians(0.0)

        #     set all in table positions array
        self.table_grab_positions = [position_0, position_1, position_2, position_3,
                                     position_4, position_5, position_6,
                                     position_7]

        # issues = False
        # for i in range(0, 8):
        #     if (i == 7):
        #         if(not abs(self.euclidean_distance(self.table_grab_positions[i], self.table_grab_positions[0]) - self.TABLE_WIDTH) < 0.01):
        #             print(self.euclidean_distance(self.table_grab_positions[i],
        #                                             self.table_grab_positions[0]))
        #             print(i)
        #             issues = True
        #     else:
        #         #0.0707 is 0.05X --> 0.05Y distance in 2d
        #         if( not abs(self.euclidean_distance(self.table_grab_positions[i], self.table_grab_positions[i+1]) - self.TABLE_WIDTH) < 0.01 and
        #             not abs(self.euclidean_distance(self.table_grab_positions[i], self.table_grab_positions[i+1]) - 0.0707) < 0.01):
        #             print("Index : " + str(i) + " Index: " + str(i+1))
        #             print(self.euclidean_distance(self.table_grab_positions[i],
        #                                           self.table_grab_positions[i+1]))
        #             issues = True

        # if( not issues):
        #     print("No distance issues")

        #print(self.table_grab_positions)

        #Modify grab positions by table pose
        i = 0
        for twist in self.table_grab_positions:
            self.table_grab_positions[i].angular.z = (twist.angular.z + self.table_pose.angular.z) % (2*math.pi)
            i += 1

        self.TABLE_POSE_SET = True

    def update_pose(self, data):
        self.pose.linear.x = data.transform.translation.x
        self.pose.linear.y = data.transform.translation.y
        _, _, angle = euler_from_quaternion(data.transform.rotation.x,
                                                          data.transform.rotation.y,
                                                          data.transform.rotation.z,
                                                          data.transform.rotation.w)
        #This function (transform_rotated_point strips off rotation info)
        self.pose = self.transform_rotated_point(self.pose,
                                                 self.ROBOT_OFFSET_X,
                                                 self.ROBOT_OFFSET_Y,
                                                 angle)
        self.pose.angular.z = angle
        self.pose.angular.z += self.ROBOT_OFFSET_AZ
        self.pose.angular.z = (self.pose.angular.z + 2 * math.pi) % (2*math.pi)


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
    # This selects the closest grabbing position (Use pose or gripper)
    def get_closest_table_leg(self, grabbing_position):
        # Array of 8 grabbing positions (2 per leg).
        min = 9999
        index = 0
        i = 0
        dist_sum = 0
        #Positions are twists not poses
        for position in self.table_grab_positions:
            distance_to_position = self.euclidean_distance(position, grabbing_position)
            #print(distance_to_position)
            dist_sum += distance_to_position
            if min > distance_to_position:
                min = distance_to_position
                index = i
            i += 1
        print(index)
        #print(dist_sum)
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

    def getGripperTwist(self):
        gripper = self.transform_rotated_point(self.pose,
                                               self.GRIPPER_OFFSET_X,
                                               self.GRIPPER_OFFSET_Y,
                                               self.pose.angular.z)
        return gripper

    def debug(self, All):
        print("Goal distance: " + str(distance_to_goal))
        print("Closest leg: " + str(self.closest_table_leg))
        print("Pose error X: " + str(self.error_x) + " , Y: " + str(self.error_y)
              + ", AZ: " + str(self.alignment_error))
        if(All):
            print("Pose X: " + str(self.pose.linear.x) + " , Y: " + str(self.pose.linear.y)
                  + ", AZ: " + str(self.pose.angular.z))
            print("Goal X: " + str(self.goal.linear.x) + " , Y: " + str(self.goal.linear.y)
                  + ", AZ: " + str(self.goal.angular.z))
            print("Table X: " + str(self.table_pose.linear.x) + " , Y: " + str(self.table_pose.linear.y)
                  + ", AZ: " + str(self.table_pose.angular.z))
            print("Pos (4) X: " + str(self.table_grab_positions[4].linear.x) + " , Y: " + str(
                self.table_grab_positions[4].linear.y)
                  + ", AZ: " + str(self.table_grab_positions[4].angular.z))
            print("Pos (5) X: " + str(self.table_grab_positions[5].linear.x) + " , Y: " + str(
                self.table_grab_positions[5].linear.y)
                  + ", AZ: " + str(self.table_grab_positions[5].angular.z))
            print("ERROR Pos (4) X: " + str(self.table_grab_positions[4].linear.x - self.gripper_position.linear.x)
                  + " , Y: " + str(self.table_grab_positions[4].linear.y - self.gripper_position.linear.y)
                  + ", AZ: " + str(self.table_grab_positions[4].angular.z))
            print("ERRPR Pos (5) X: " + str(self.table_grab_positions[5].linear.x - self.gripper_position.linear.x)
                  + " , Y: " + str(self.table_grab_positions[5].linear.y - self.gripper_position.linear.y)
                  + ", AZ: " + str(self.table_grab_positions[5].angular.z))
            #time.sleep(1)

    def manageState(self):
        self.rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.gripper_position = self.getGripperTwist()
            self.closest_table_leg = self.get_closest_table_leg(self.pose)
            self.goal = self.get_table_pose(self.closest_table_leg)

            #Actually move to table leg
            if(self.robotState == RobotState.TableGrabPosition):
                self.error_x = (self.goal.linear.x - self.gripper_position.linear.x)
                self.error_y = (self.goal.linear.y - self.gripper_position.linear.y)
                self.in_grabbing_position = self.euclidean_distance_xy(self.error_x, self.error_y) < self.PRECISE_DISTANCE_TOLERANCE
            else: #Move to offset position from table leg
                self.error_x = (self.goal.linear.x - self.pose.linear.x)
                self.error_y = (self.goal.linear.y - self.pose.linear.y)

            #Rotated to local robot frame error to table
            forward_error = Twist()
            forward_error.linear.x = -self.error_x
            forward_error.linear.y = self.error_y
            forward_error = self.transform_rotated_point(forward_error,0,0,self.pose.angular.z)

            required_table_angle = self.goal.angular.z
            distance_to_goal = self.euclidean_distance(self.goal, self.pose)

            self.alignment_error = self.wrapped_angle(self.pose.angular.z, required_table_angle)
            self.grabbing_force = self.load_cell_pos_x - self.load_cell_neg_x

            print("Goal distance: " + str(distance_to_goal))
            print("Closest leg: " + str(self.closest_table_leg))
            print("Pose error X: " + str(self.error_x) + " , Y: " + str(self.error_y)
                  + ", AZ: " + str(self.alignment_error))
            print("Pose X: " + str(self.pose.linear.x) + " , Y: " + str(self.pose.linear.y)
                  + ", AZ: " + str(self.pose.angular.z))
            print("Table X: " + str(self.table_pose.linear.x) + " , Y: " + str(self.table_pose.linear.y)
                  + ", AZ: " + str(self.table_pose.angular.z))
            print("Goal X: " + str(self.goal.linear.x) + " , Y: " + str(self.goal.linear.y)
                  + ", AZ: " + str(self.goal.angular.z))
            print("Forward error: " + str(forward_error.linear.x) + " , Sideways Error: " + str(forward_error.linear.y))
            #self.debug(False)
            # Conditions are in reverse order as the higher precision conditions will be
            # checked first and be false kicking the algorithm down to the next condition

            # Global Reset grabbing timer
            if (self.grabbing_force > self.TABLE_GRABBING_FORCE):
                self.grabbing_timer_started = False

            #Robot occuluded
            if(self.pose.linear.x == 0 and self.pose.linear.y == 0):
                print("Occluded")

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
            #         self.robotState = RobotState.Init
            # elif (self.grabbing_force > self.TABLE_GRABBING_FORCE and
            #       self.actuator_state == ActuatorState.Up):
            #     print("State: Move table")
            #     self.robotState = RobotState.MoveTable
            #     self.state_move_table()
            # elif (self.grabbing_force > self.TABLE_GRABBING_FORCE):
            #     print("State: Lift table")
            #     self.state_lift_table()
            #     self.dont_move()
            elif (self.in_grabbing_position and
                  abs(self.alignment_error) < self.ALIGNMENT_ERROR_TOLERANCE and
                  (timer() - self.grabbing_timer_start < self.GRABBING_TIMEOUT
                   or not self.grabbing_timer_started)):
                print("State: Grab table")
                self.robotState = RobotState.GrabTable
                self.state_grab_table()
                self.dont_move()
            #Previous state was preciseMove or you are in process of moving to grabbing position.
            elif self.robotState == RobotState.PreciseMove or \
                    self.robotState == RobotState.TableGrabPosition and\
                    abs(self.alignment_error) < self.ALIGNMENT_ERROR_TOLERANCE:
                print("State: Move to actual table grabbing position")
                self.robotState = RobotState.TableGrabPosition
                self.state_move_to_grabbing_position()
            elif (abs(self.alignment_error) < self.ALIGNMENT_ERROR_TOLERANCE
                and distance_to_goal <= self.ROUGH_DISTANCE_TOLERANCE):
                print("State: Precise move to pre-grabbing position")
                self.robotState = RobotState.PreciseMove
                self.state_move_to_grabbing_position()
            elif (distance_to_goal <= self.ROUGH_DISTANCE_TOLERANCE):
                print("State: Aligning to grabbing position")
                self.state_align_to_goal()
                self.robotState = RobotState.AlignGrab
            elif (distance_to_goal > self.ROUGH_DISTANCE_TOLERANCE):
                print("State: Rough move to pre-grabbing position")
                self.state_move_to_goal()
                self.robotState = RobotState.RoughMove
            self.rate.sleep()

    def state_startup(self):
        self.servo_publisher.publish(self.SERVO_OPEN)
        self.lifter_publisher.publish(ActuatorState.Down)

    def state_move_to_goal(self):
        self.goal = self.goal

        vel_msg = Twist()

        # Porportional controller.
        # https://en.wikipedia.org/wiki/Proportional_control
        vel_msg.linear.x = self.error_x * self.TRANSLATION_P
        vel_msg.linear.y = self.error_y * self.TRANSLATION_P
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = self.alignment_error * self.ALIGNMENT_P

        # Minimum_speed
        self.setMinimumSpeed(vel_msg, self.MINIMUM_SPEED)

        self.setMaximumSpeed(vel_msg, self.MAXIMUM_SPEED)
        #print(vel_msg)

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
        #Get the sign of the angle
        sign_ang = vel_msg.angular.z/abs(vel_msg.angular.z)
        if abs(vel_msg.angular.z) > self.MAXIMUM_ANGULAR_VEL:
            vel_msg.angular.z = self.MAXIMUM_ANGULAR_VEL*sign_ang
        if abs(vel_msg.angular.z) < self.MINIMUM_ANGULAR_VEL:
            vel_msg.angular.z = self.MINIMUM_ANGULAR_VEL*sign_ang

        self.velocity_publisher.publish(vel_msg)

    def dont_move(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def state_move_to_grabbing_position(self):
        self.goal = self.goal
        table_pose = self.get_table_pose(self.closest_table_leg)

        vel_msg = Twist()

        # Porportional controller.
        # https://en.wikipedia.org/wiki/Proportional_control
        vel_msg.linear.x = self.error_x * self.TRANSLATION_P
        vel_msg.linear.y = self.error_y * self.TRANSLATION_P
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Minimum_speed = Maximum speed
        self.setMinimumSpeed(vel_msg, self.MINIMUM_SPEED)
        self.setMaximumSpeed(vel_msg, self.MINIMUM_SPEED)

        self.velocity_publisher.publish(vel_msg)

    def state_grab_table(self):
        if self.grabbing_timer_started == False:
            self.grabbing_timer_start = timer()
            self.grabbing_timer_started = True
        self.servo_publisher.publish(self.SERVO_CLOSED)

    def state_lift_table(self):
        self.lifter_publisher.publish(ActuatorState.Up)

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
