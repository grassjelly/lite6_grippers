import asyncio

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

from parallel_gripper.gripper_driver import ParallelGripperOpenRB150


class GripperDriverNode(Node):
    def __init__(self):
        super().__init__('parallel_gripper_driver')

        self.declare_parameter('robot_ip', '192.168.1.151')
        self.declare_parameter('joint_state_publish_rate', 10.0)

        robot_ip = self.get_parameter('robot_ip').value
        if not robot_ip:
            raise RuntimeError('robot_ip parameter is required')

        from xarm.wrapper import XArmAPI
        arm = XArmAPI(robot_ip, is_radian=False)
        self._gripper = ParallelGripperOpenRB150(arm)

        self._js_pub = self.create_publisher(JointState, 'joint_states', 10)

        rate = self.get_parameter('joint_state_publish_rate').value
        self._timer = self.create_timer(1.0 / rate, self._publish_js)

        # Action server at the topic MoveIt's SimpleControllerManager expects
        self._action_server = ActionServer(
            self,
            GripperCommand,
            'parallel_gripper_controller/gripper_action',
            execute_callback=self._execute_cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info('Parallel gripper driver ready')

    def _publish_js(self):
        try:
            pos = self._gripper.get_pos()
        except Exception as e:
            self.get_logger().warn(f'Failed to read gripper position: {e}', throttle_duration_sec=5.0)
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # drive_joint (+Y) and right_finger_joint (-Y axis) both publish same position value;
        # robot_state_publisher applies the joint axis direction automatically.
        msg.name = ['drive_joint', 'right_finger_joint']
        msg.position = [pos, pos]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        self._js_pub.publish(msg)

    async def _execute_cb(self, goal_handle):
        target = max(0.0, min(ParallelGripperOpenRB150.MAX_METERS,
                               goal_handle.request.command.position))

        self._gripper.move(target)
        # Fixed wait proportional to full-stroke time (~1 s)
        await asyncio.sleep(1.0)

        try:
            actual = self._gripper.get_pos()
        except Exception:
            actual = target

        result = GripperCommand.Result()
        result.position = actual
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = abs(actual - target) < 0.002  # 2 mm tolerance
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GripperDriverNode()
    rclpy.spin(node)
    rclpy.shutdown()
