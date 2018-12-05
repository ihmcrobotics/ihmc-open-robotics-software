import rclpy

from controller_msgs.msg import RobotConfigurationData


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('rcd_sub')

    subscription = node.create_subscription(
        RobotConfigurationData, 'topic', lambda msg: node.get_logger().info('I heard: ' + str(msg.timestamp)))
    subscription  # prevent unused variable warning

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
