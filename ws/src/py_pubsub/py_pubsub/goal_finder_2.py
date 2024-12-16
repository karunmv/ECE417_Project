import rclpy
from rclpy.node import Node
# Detection msg format from ARUCO markers
from aruco_opencv_msgs.msg import ArucoDetection
# Twist msg format to jetbot motors
from geometry_msgs.msg import Twist
import math

# what is minimum distance from the ARUCO marker at which
# the jetbot should have VMAX
MAXVELDIST = 0.20
VMAX = 0.10
VMIN = 0.07 # if he velocity is less than this, the motors are unable to move

# what is minimum distance from the ARUCO marker at which
# the jetbot should have VMIN
MINDIST = 0.10

# https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
Kp_linear = 1.

# what is maximum angle to the ARUCO marker at which
# the jetbot should have OMAX
OMAXANGLE = 25 * math.pi/180.
OMAX = 1.00


# what is minimum angle to the ARUCO marker at which
# the jetbot should have OMIN
OMINANGLE = 5 * math.pi/180.
# if angular velocity is smaller than this, then motors are unable to move the jetbot
OMIN = 0.50

Kp_angular = 1.

def new_twist(linear_vel, ang_vel):
    """Generate a new command to be sent to the motors_waveshare

    linear_vel: linear velocity
    ang_vel : angular velocity
    """
    twist = Twist()
    twist.linear.x = linear_vel
    twist.linear.y = 0.
    twist.linear.z = 0.
    twist.angular.x = 0.
    twist.angular.y = 0.
    twist.angular.z = ang_vel
    return twist



class AvoidARUCO(Node):
    def __init__(self):
        super().__init__('goal_finder')
        self.sub = self.create_subscription(ArucoDetection,
        '/aruco_detections',
        self.on_aruco_detection, 10)
        self.pub = self.create_publisher(Twist, '/jetbot/cmd_vel', 10)
        self.timer = None
        # We are going to chase this marker
        self.chosen_marker_to_chase = 7 # This is the goal marker

    def on_aruco_detection(self, msg):
        """ This function will be called whenever one-single message of ARUCO
        detection message is received. (Basically all the times repeatedly, as
        long as markers are being detected)"""

        self.get_logger().info('Received: "%s"' % msg.markers)
        #If no marker is detected, go straight
        if not len(msg.markers):
            self.pub.publish(new_twist(-VMAX, 0.))
        else:
            # We know the chosen one, look for it in all detections
            goal_marker = None
            avoid_marker = None
            for m in msg.markers:
                if m.marker_id == self.chosen_marker_to_chase:
                    goal_marker = m
                else:
                    avoid_marker = m
            # if goal_marker is not None:
            #     # Found it
            #     # goal_marker = 7
            #     pass
            # else:
            #     self.get_logger().warning("Obstacle avoidance active, searching for marker_id %d" %
            #                               self.chosen_marker_to_chase)
            #     return

            if msg.markers[0].marker_id == 7:
                # Extract the minimum (2D) amount of information, x and z coordinate of
                # the marker
                self.get_logger().info('Position of goal marker: "%s"' %
                                       goal_marker.pose.position)
                z = goal_marker.pose.position.z
                x = goal_marker.pose.position.x
                # Get the angle between the robot facing
                # direction and the line connecting the robot to the marker
                angle = math.atan2(x, z)

                # Do linear interpolation for linear velocity
                linear_vel = (VMIN
                              + Kp_linear * (VMAX - VMIN) * (z - MINDIST)
                              / (MAXVELDIST - MINDIST))
                # Clip the linear velocity between VMIN and VMAX within safe
                # limits
                linear_vel_clipped = (VMAX if  abs(VMAX) < abs(linear_vel) else
                                      0. if abs(linear_vel) < abs(VMIN) else
                                      linear_vel)
                self.get_logger().info('Linear: "%f";' % linear_vel_clipped)

                # Do linear interpolation for angular velocity
                ang_vel = (OMIN + Kp_angular * (OMAX - OMIN) * (angle - OMINANGLE) /
                           (OMAXANGLE - OMINANGLE))
                # Clip the angular velocity between OMAX and OMIN within safe
                # limits
                ang_vel_clipped = (OMAX if abs(OMAX) < abs(ang_vel) else
                                   0. if abs(ang_vel) < abs(OMIN) else
                                   ang_vel)
                self.get_logger().info('Angular: "%f";' % ang_vel_clipped)
                self.pub.publish(new_twist(-linear_vel_clipped, ang_vel_clipped))
            else:
                self.get_logger().info('Position of obstacle marker: "%s"' %
                                       avoid_marker.pose.position)
                z = avoid_marker.pose.position.z
                x = avoid_marker.pose.position.x

                angle = math.atan2(x, z)

                if z < 0.75:
                    linear_vel = (VMIN
                                + Kp_linear * (VMAX - VMIN) * (z - MINDIST)
                                / (MAXVELDIST - MINDIST))

                    linear_vel_clipped = (VMAX if abs(VMAX) < abs(linear_vel) else
                                        0. if abs(linear_vel) < abs(VMIN) else
                                        linear_vel)
                    self.get_logger().info('Linear: "%f";' % linear_vel_clipped)

                    ang_vel = (OMIN + Kp_angular * (OMAX - OMIN) * (OMAXANGLE - angle) /
                            (OMAXANGLE - OMINANGLE))

                    ang_vel_clipped = (OMAX if abs(OMAX) < abs(ang_vel) else
                                    0. if abs(ang_vel) < abs(OMIN) else
                                    ang_vel)
                    self.get_logger().info('Angular: "%f";' % ang_vel_clipped)

                    if x<0:
                        self.pub.publish(new_twist(-linear_vel_clipped, ang_vel_clipped))
                    else:
                        self.pub.publish(new_twist(-linear_vel_clipped, -ang_vel_clipped))
                else:
                    linear_vel = (VMIN
                                + Kp_linear * (VMAX - VMIN) * (z - MINDIST)
                                / (MAXVELDIST - MINDIST))

                    linear_vel_clipped = (VMAX if abs(VMAX) < abs(linear_vel) else
                                        0. if abs(linear_vel) < abs(VMIN) else
                                        linear_vel)
                    self.get_logger().info('Linear: "%f";' % linear_vel_clipped)

                    ang_vel = (OMIN + Kp_angular * (OMAX - OMIN) * (angle - OMINANGLE) /
                            (OMAXANGLE - OMINANGLE))

                    ang_vel_clipped = (OMAX if abs(OMAX) < abs(ang_vel) else
                                    0. if abs(ang_vel) < abs(OMIN) else
                                    ang_vel)
                    self.get_logger().info('Angular: "%f";' % ang_vel_clipped)

def main(args=None):
    rclpy.init(args=args)

    move2a = AvoidARUCO()

    rclpy.spin(move2a)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move2a.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
