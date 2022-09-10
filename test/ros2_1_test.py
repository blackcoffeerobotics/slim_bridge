import yaml
import os

import rclpy
from std_msgs.msg import String

def main(args=None):

    rclpy.init(args=args)

    node = rclpy.create_node('ros2_1_test')
    qos = rclpy.qos.QoSProfile(durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, depth=10)
    robot_name = os.environ['ROBOT_NAME']
    publisher = node.create_publisher(String, robot_name+'/ros2_to_1', qos_profile=qos)

    msg = String()

    dictionary = {
        'topic_name': '/test_string_2_1',
        'latch' : True,
        'reliability': 'reliable',
        'durability': 'transient_local',
        'history': 'keep_last',
        'depth': 1,
        'queue_size': 1,
        'type': 'std_msgs/String'
    }

    msg.data = str(yaml.dump(dictionary, Dumper=yaml.dumper.SafeDumper))
    publisher.publish(msg)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
