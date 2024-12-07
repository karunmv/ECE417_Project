import rclpy
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Twist
import math

# Parameters for velocity control
MAXVELDIST = 0.20
VMAX = 0.10
VMIN = 0.07
MINDIST = 0.10
Kp_linear = 1.0

OMAXANGLE = 25 * math.pi / 180.0
OMAX = 1.00
OMINANGLE = 5 * math.pi / 180.0
OMIN = 0.50
Kp_angular = 1.0

REPEL_SCALE = 1.0  # Scale for repelling effect of obstacles


def new_twist(linear_vel, ang_vel):
    """Generate a Twist message."""
    twist = Twist()
    twist.linear.x = linear_vel
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = ang_vel
    return twist


class AvoidARUCO(Node):
    def __init__(self):
        super().__init__('avoid_aruco')
        self.sub = self.create_subscription(ArucoDetection,
                                             '/aruco_detections',
                                             self.on_aruco_detection,
                                             10)
        self.pub = self.create_publisher(Twist, '/jetbot/cmd_vel', 10)
        self.timer = None

    def on_aruco_detection(self, msg):
        """Callback for ARUCO detections."""
        self.get_logger().info('Received markers: "%s"' % msg.markers)
        
        if not msg.markers:
            # No markers detected, stop the robot
            self.pub.publish(new_twist(0.0, 0.0))
            return

        goal_marker = None
        repulsive_force_x = 0.0
        repulsive_force_z = 0.0

        for marker in msg.markers:
            z = marker.pose.position.z
            x = marker.pose.position.x
            angle = math.atan2(x, z)

            if marker.marker_id == 0:
                # Goal marker
                goal_marker = marker
            else:
                # Repulsive force from obstacles
                repulsion_strength = REPEL_SCALE / (z ** 2 + 1e-6)  # Avoid divide-by-zero
                repulsive_force_x += -repulsion_strength * math.sin(angle)
                repulsive_force_z += -repulsion_strength * math.cos(angle)

        if goal_marker is None:
            # No goal marker detected, avoid obstacles
            linear_vel = max(VMIN, min(VMAX, math.sqrt(repulsive_force_x ** 2 + repulsive_force_z ** 2)))
            ang_vel = math.atan2(repulsive_force_x, repulsive_force_z)
            self.pub.publish(new_twist(-linear_vel, ang_vel))
            return

        # Move toward the goal marker, avoiding others
        z = goal_marker.pose.position.z
        x = goal_marker.pose.position.x
        angle_to_goal = math.atan2(x, z)

        # Combine goal-seeking and obstacle-avoiding velocities
        linear_vel_goal = VMIN + Kp_linear * (VMAX - VMIN) * (z - MINDIST) / (MAXVELDIST - MINDIST)
        linear_vel_goal = max(VMIN, min(VMAX, linear_vel_goal))

        ang_vel_goal = Kp_angular * angle_to_goal

        total_force_x = math.sin(angle_to_goal) * linear_vel_goal + repulsive_force_x
        total_force_z = math.cos(angle_to_goal) * linear_vel_goal + repulsive_force_z

        combined_angle = math.atan2(total_force_x, total_force_z)
        combined_velocity = math.sqrt(total_force_x ** 2 + total_force_z ** 2)

        linear_vel_clipped = max(VMIN, min(VMAX, combined_velocity))
        angular_vel_clipped = max(OMIN, min(OMAX, combined_angle))

        self.get_logger().info(f'Linear: {linear_vel_clipped}; Angular: {angular_vel_clipped}')
        self.pub.publish(new_twist(-linear_vel_clipped, angular_vel_clipped))


def main(args=None):
    rclpy.init(args=args)
    node = AvoidARUCO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
