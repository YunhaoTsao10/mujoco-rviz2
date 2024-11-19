import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class JointStateToCommand(Node):
    def __init__(self):
        super().__init__('joint_state_to_command')

        # 订阅 /joint_states 获取来自 joint_state_publisher_gui 的关节状态
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.callback, 10)

        # 动态创建每个关节的控制命令发布器
        self.joint_commands_pub = {}

    def callback(self, msg):
        """回调函数，处理接收到的 JointState 消息并发布控制命令"""
        # 对于每个关节，发布其对应的目标位置
        for i, joint_name in enumerate(msg.name):
            target_position = msg.position[i]
            
            # 动态创建对应关节的发布器（如果尚未创建的话）
            if joint_name not in self.joint_commands_pub:
                self.joint_commands_pub[joint_name] = self.create_publisher(
                    Float64, f'/h1_mujoco_control/{joint_name}/command', 10)

            # 创建控制命令消息
            command_msg = Float64()
            command_msg.data = target_position

            # 发布控制命令到对应的关节话题
            self.joint_commands_pub[joint_name].publish(command_msg)
            self.get_logger().info(f"Publishing command for {joint_name}: {target_position}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateToCommand()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

