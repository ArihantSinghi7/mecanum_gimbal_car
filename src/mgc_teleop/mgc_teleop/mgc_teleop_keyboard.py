import rclpy
from rclpy.node import Node 
import sys
import tty
import termios
import threading
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from mgc_teleop.clamp import Clamp
from math import pi

# NOTE:
# We need 'TwistStamped' because every 1 sec(as specified in the yaml file) the velocity command expires.
# So to know when the velocity expires we need a time stamp of the command.
# We need a 'frame_id' to let ros2_control know the reference frame in which the velocity is being expressed.

# ------------------------------ USER VARIABLES ------------------------------

GIMBAL_JOINT_NAMES = ["gimbal_base_TO_gimbal_yaw_link", "gimbal_yaw_link_TO_gimbal_pitch_link"]
GIMBAL_JOINT_LIMITS = {
                        GIMBAL_JOINT_NAMES[0]: Clamp(-pi, pi), 
                        GIMBAL_JOINT_NAMES[1]: Clamp(-pi/2.0, pi/2.0)}
LINEAR_VELOCITY = 1.0
ANG_VELOCITY = 1.0
KEY_BINDINGS_FOR_MOBILE_BASE = {
    'S': (0.0, 0.0, 0.0), # STOP
    'W': (1.0, 0.0, 0.0),   # FORWARD
    'A': (0.0, 1.0, 0.0),   # STRAFE LEFT
    'X': (-1.0, 0.0, 0.0),  # BACKWARD  
    'D': (0.0, -1.0, 0.0),  # STRAFE RIGHT
    'E': (1.0/1.4142, -1.0/1.4142, 0.0),  # NORTH-EAST
    'Q': (1.0/1.4142, 1.0/1.4142, 0.0),   # NORTH-WEST
    'Z': (-1.0/1.4142, 1.0/1.4142, 0.0),  # SOUTH-WEST  
    'C': (-1.0/1.4142, -1.0/1.4142, 0.0), # SOUTH-EAST
    'T': (0.0, 0.0, 1.0),  # COUNTER-CLOCKWISE ROTATION  
    'Y': (0.0, 0.0, -1.0) # CLOCKWISE ROTATION
}
KEY_BINDINGS_FOR_GIMBAL = {
    'I': (1, +1),   # PITCH DOWN
    'K': (1, -1),   # PITCH UP
    'J': (0, +1),   # YAW COUNTER-CLOCKWISE
    'L': (0, -1)    # YAW CLOCKWISE
}


# Key bindings to show on the terminal to the user
BANNER = r"""
Mecanum Base Teleop
------------------
Q   W   E           S   :Stop
  \ | /             W/X : Forward / Backward
A - S - D           A/D : Strafe Left / Right
  / | \             E/Z : Strafe NE / SW
Z   X   C           Q/C : Strafe NW / SE 

T/Y : Rotate Counter-Clockwise / Clockwise

Gimbal Teleop
------------------
I: Pitch Down
K: Pitch Up
J: Yaw Counter-Clockwise
L: Yaw Clockwise
R: Gimbal reset to home position

Ctrl+C: Quit
------------------
"""

# ----------------------------------------------------------------------------



class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")

        # Declaring parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('gimbal_pitch_step_rad', 0.05)
        self.declare_parameter('gimbal_yaw_step_rad', 0.05)
        self.declare_parameter('trajectory_duration_ms', 200)
        self.declare_parameter('vel_publishing_freq', 20)

        # Getting value of the parameters
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.gimbal_pitch_step_rad = self.get_parameter("gimbal_pitch_step_rad").value
        self.gimbal_yaw_step_rad = self.get_parameter("gimbal_yaw_step_rad").value
        self.traj_duration_ms = self.get_parameter('trajectory_duration_ms').value
        self.vel_publishing_freq = self.get_parameter("vel_publishing_freq").value

        # Declaring some variables
        self.gimbal_current_position = [0.0]*len(GIMBAL_JOINT_NAMES)
        self.v_x = 0.0                
        self.v_y = 0.0                 
        self.w_z = 0.0

        # Creating a publisher to publish 'stamped_velocity' for mobile base
        self.stamped_velocity_publisher = self.create_publisher(TwistStamped,
                                                                "/mecanum_drive_controller/reference",
                                                                10)
        
        # Creating a publisher to publish 'joint_ttrajectory' for gimbal 
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory,
                                                                "/gimbal_controller/joint_trajectory",
                                                                10)

        # Creating a timer to get keyboard input
        self.get_keyboard_input_loop = self.create_timer((1.0/self.vel_publishing_freq), self.publish_stamped_velocity)


        # NOTE:
        # Since we are running two threads and both either uses or updates  these variables: self.v_x, self.v_y, self.w_z
        # we need to create a thread lock such that only one thread can access these variables at a time.
        self.lock = threading.Lock()

        # NOTE:
        # We need a separate thread to read the input from keyboard. 
        # We don't want to stop publishing velocity while we wait for a keyboard input.

        # Keyboard thread
        self.kb_thread_running = True
        self.kb_thread = threading.Thread(target=self.get_keyboard_input, daemon=True)
					 # target=self.keyboard_loop: The function to run on a separate thread 
                     # daemon=True: Allows python to quit when the main thread finishes and not wait for this thread to finish
        self.kb_thread.start()			# Starts the new thread which runs parallel to the main thread


        print(BANNER)



    # NOTE:
    # Normally when we use terminal, we write the entire command and press enter for the command to get registered.
    # But, for teleop operation we want to register the command as soon as it is entered. For this we use 'tty' to switch the terminal to raw mode.
    # After exiting the node we need to switch the terminal back to its original setting, for this we use 'termios' to save the old terminal setting and then revert back to it. 

    # Method to get the pressed key
    def get_key(self):
        fd = sys.stdin.fileno()                             # Gets the file descriptor for keyboard input.
        old = termios.tcgetattr(fd)                         # Gets the current terminal setting before we switch the terminal to raw mode.
        try:
            tty.setraw(fd)                                  # Sets the terminal to raw mode.
            key = sys.stdin.read(1).upper()                  # Reads exactly one character from the keyboard.
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)   # Resets the terminal back to its original state. 
                                                            # termios.TCSADRAIN: wait until all pending output has been written, then apply the terminal setting.
        return key


    # Method to get keyboard input
    def get_keyboard_input(self):
        while self.kb_thread_running:
            key = self.get_key()            # This statement will blocks this thread until a key is pressed.

            # Thread lock once the input is received
            with self.lock:

                
                if key in KEY_BINDINGS_FOR_MOBILE_BASE:     # Move Mobile Base
                    dx, dy, dw = KEY_BINDINGS_FOR_MOBILE_BASE[key]
                    self.v_x = dx*self.linear_speed
                    self.v_y = dy*self.linear_speed
                    self.w_z = dw*self.angular_speed

                elif key in KEY_BINDINGS_FOR_GIMBAL:        # Move Gimbal
                    joint_no, sign = KEY_BINDINGS_FOR_GIMBAL[key]
                    if joint_no == 0:            # Yaw Joint
                        self.gimbal_current_position[joint_no] += self.gimbal_yaw_step_rad*sign

                    elif joint_no == 1:           # Pitch Joint
                        self.gimbal_current_position[joint_no] += self.gimbal_pitch_step_rad*sign
                
                    else:
                        self.get_logger().error("Gimbal Joint Not Found!")

                    # Clamp and Publish
                    self.gimbal_current_position[joint_no] = GIMBAL_JOINT_LIMITS[GIMBAL_JOINT_NAMES[joint_no]].clamp(self.gimbal_current_position[joint_no])
                    self.publish_gimbal_angles()

                elif key == 'R':            # Reset Gimbal
                    self.gimbal_current_position = [0.0] * len(GIMBAL_JOINT_NAMES)
                    self.publish_gimbal_angles()
                
                elif key == '\x03':         # Ctrl+C
                    self.kb_thread_running = False
                    rclpy.try_shutdown()

                else: 
                    pass


    # Method to publish stamped velocity
    def publish_stamped_velocity(self):
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()      # Gets time and converts into valid format  

        with self.lock:
            msg.twist.linear.x = self.v_x
            msg.twist.linear.y = self.v_y
            msg.twist.angular.z = self.w_z

        self.stamped_velocity_publisher.publish(msg)


    # Method to publish gimbal joint angles
    def publish_gimbal_angles(self):
        msg = JointTrajectory()
        msg.joint_names = GIMBAL_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = self.gimbal_current_position[:]
        pt.time_from_start = pt.time_from_start = Duration(sec = 0,
                                                   nanosec = self.traj_duration_ms*1000000)
        msg.points = [pt]
        self.joint_trajectory_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.kb_thread_running = False		# Safety net to ensure that the keyboard thread is always stopped.
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()