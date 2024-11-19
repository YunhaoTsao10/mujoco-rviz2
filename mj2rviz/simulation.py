import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import mujoco
import mujoco.viewer as mj_viewer
import threading
import time
import math
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_msgs.msg import TFMessage
import os
import yaml


class H1MujocoController(Node):
    def __init__(self):
        super().__init__('h1_mujoco_controller')

        # Load configuration
        config_file = os.getenv('CONFIG_FILE_PATH')
    
        if os.path.exists(config_file):
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
                model_path = config.get('paths', {}).get('scene_file', '<if xml_file is not found, type your absolute path here!>')
        else:
            self.get_logger().warn(f"Configuration file {config_file} not found")

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Launch the viewer in passive mode to allow external control
        self.viewer = mj_viewer.launch_passive(self.model, self.data)

        # Configure camera parameters
        self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.viewer.cam.lookat = [0, 0, 0]
        self.viewer.cam.distance = 10.0
        self.viewer.cam.azimuth = 90
        self.viewer.cam.elevation = -45

        # 创建发布器，发布到自定义的 mujoco_tf 话题
        self.mujoco_tf_publisher = self.create_publisher(TFMessage, 'mujoco_tf', 10)

        # 用于存储原始 /tf 数据的变量
        self.original_tf = []

        # 订阅 /tf 话题
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        # Create JointState publisher and joint command subscribers
        self.joint_state_pub = self.create_publisher(JointState, '/mujoco_joint_states', 10)
        self.joint_commands_subs = {}

        # Retrieve joint names and create subscriptions for each joint
        joint_names = []
        for joint_id in range(1, self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if joint_name is not None:
                joint_names.append(joint_name)

        for joint_name in joint_names:
            self.joint_commands_subs[joint_name] = self.create_subscription(
                Float64, f'/h1_mujoco_control/{joint_name}_joint/command',
                lambda msg, j=joint_name: self.set_joint_target(j, msg.data), 10)

        # Set up a ROS timer for updating simulation
        self.timer = self.create_timer(0.1, self.update)

    def set_joint_target(self, joint_name, target_position):
        """Set target position for a specific joint."""
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id is not None and joint_id <= self.model.nu:

            # print(f"Number of controllable joints (nu): {self.model.nu}")
            # print(f"Joint {joint_name} has ID {joint_id}")

            self.data.ctrl[joint_id - 1] = target_position * (180.0 / math.pi)
        else:
            self.get_logger().warn(f"Joint {joint_name} ID not found or exceeds control limit.")
    
    def tf_callback(self, msg):
        # 更新原始 TF 数据
        self.original_tf = msg.transforms

    def publish_combined_tf(self):
        # 创建一个 TFMessage，用于整合原始和自定义的 TF 数据
        combined_tf_message = TFMessage()

        # 添加原始的 TF 数据
        combined_tf_message.transforms.extend(self.original_tf)

        # 添加自定义的 world->pelvis 的 TF
        combined_tf_message.transforms.append(self.get_pelvis_to_world_tf())

        # 发布到 mujoco_tf 话题
        self.mujoco_tf_publisher.publish(combined_tf_message)

    def get_pelvis_to_world_tf(self):
        """从 MuJoCo 数据中精确获取 pelvis -> world 的变换."""
        transform = TransformStamped()

        # 时间戳和坐标系设置
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "pelvis"

        # 获取 pelvis 的 body_id
        pelvis_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")

        # 获取 pelvis 在世界坐标系下的位置和旋转
        transform.transform.translation.x = self.data.xpos[pelvis_id][0]
        transform.transform.translation.y = self.data.xpos[pelvis_id][1]
        transform.transform.translation.z = self.data.xpos[pelvis_id][2]

        # MuJoCo 使用 [qw, qx, qy, qz]，ROS 使用 [qx, qy, qz, qw]
        transform.transform.rotation.w = self.data.xquat[pelvis_id][0]
        transform.transform.rotation.x = self.data.xquat[pelvis_id][1]
        transform.transform.rotation.y = self.data.xquat[pelvis_id][2]
        transform.transform.rotation.z = self.data.xquat[pelvis_id][3]

        return transform

    def update(self):
        """Update simulation environment and publish joint states."""
        mujoco.mj_step(self.model, self.data)
        # self.get_logger().info(f"Updated qpos: {self.data.qpos[:self.model.nq].tolist()}")
        self.publish_joint_states()
        self.publish_combined_tf()

    def publish_joint_states(self):
        """Publish joint state data."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        joint_names = []
        joint_positions = []
        joint_velocities = []

        for joint_id in range(1, self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            if joint_name is not None:
                joint_names.append(joint_name)

                # 使用正确的索引获取 qpos 和 qvel 数据
                pos_index = self.model.jnt_qposadr[joint_id]
                vel_index = self.model.jnt_dofadr[joint_id]

                joint_type = self.model.jnt_type[joint_id]
                if joint_type == mujoco.mjtJoint.mjJNT_HINGE or joint_type == mujoco.mjtJoint.mjJNT_SLIDE:
                    joint_positions.append(self.data.qpos[pos_index])
                    joint_velocities.append(self.data.qvel[vel_index])
                elif joint_type == mujoco.mjtJoint.mjJNT_BALL:
                    quat = self.data.qpos[pos_index:pos_index + 4]
                    euler = R.from_quat(quat).as_euler('xyz', degrees=False)
                    joint_positions.append(euler[2])
                    joint_velocities.append(self.data.qvel[vel_index:vel_index + 3].tolist())
            else:
                self.get_logger().warn(f"Joint name for ID {joint_id} not found.")

        # 调整 joint_names 顺序并对齐 position 和 velocity 数据
        joint_names = [name + "_joint" for name in joint_names]
        joint_positions, joint_velocities = self.realign_joint_data(joint_names, joint_positions, joint_velocities)

        joint_state_msg.name = joint_names
        joint_state_msg.position = joint_positions
        joint_state_msg.velocity = joint_velocities

        self.joint_state_pub.publish(joint_state_msg)

    def realign_joint_data(self, joint_names, positions, velocities):
        """调整关节数据顺序以匹配 joint_names."""
        correct_positions = [0.0] * len(joint_names)
        correct_velocities = [0.0] * len(joint_names)

        for i, name in enumerate(joint_names):
            # 在 positions 和 velocities 中找到对应位置
            correct_positions[i] = positions[i] if i < len(positions) else 0.0
            correct_velocities[i] = velocities[i] if i < len(velocities) else 0.0

        return correct_positions, correct_velocities


    def render_loop(self):
        # start = time.time()
        while rclpy.ok() and self.viewer.is_running():
            # step_start = time.time()
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            # time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
            # if time_until_next_step > 0:
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    h1_controller = H1MujocoController()

    # Start MuJoCo rendering in a separate thread
    render_thread = threading.Thread(target=h1_controller.render_loop)
    render_thread.start()

    # Spin the ROS 2 node in the main thread
    rclpy.spin(h1_controller)
    h1_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()