import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from xarm_msgs.srv import GetSetModbusData, SetDigitalIO, SetInt32, SetModbusTimeout

from parallel_gripper.gripper_driver import ParallelGripperOpenRB150


class GripperDriverNode(Node):
    def __init__(self):
        super().__init__('parallel_gripper_driver')

        self.declare_parameter('hw_ns', 'ufactory')
        self.declare_parameter('joint_state_publish_rate', 10.0)

        hw_ns = self.get_parameter('hw_ns').value
        svc = f'/{hw_ns}'

        # ReentrantCallbackGroup allows the async execute callback to yield at
        # each `await` without holding the group lock, so service responses and
        # the timer can be processed by other executor threads in the meantime.
        self._action_cb_group = ReentrantCallbackGroup()
        # Separate group for service clients so their response futures are
        # eligible to be processed while the action callback is suspended.
        self._service_cb_group = MutuallyExclusiveCallbackGroup()
        # Separate group for the timer so it never contends with the action
        # server callbacks.
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()

        self._modbus_cli = self.create_client(
            GetSetModbusData, f'{svc}/getset_tgpio_modbus_data',
            callback_group=self._service_cb_group)
        self._baud_cli = self.create_client(
            SetInt32, f'{svc}/set_tgpio_modbus_baudrate',
            callback_group=self._service_cb_group)
        self._timeout_cli = self.create_client(
            SetModbusTimeout, f'{svc}/set_tgpio_modbus_timeout',
            callback_group=self._service_cb_group)
        self._digital_cli = self.create_client(
            SetDigitalIO, f'{svc}/set_tgpio_digital',
            callback_group=self._service_cb_group)

        self._gripper = ParallelGripperOpenRB150(self._modbus_cli)
        self._last_pos: float = 0.0
        self._pos_future = None
        self._initialized = False
        self._init_pending = False

        self._js_pub = self.create_publisher(JointState, f'{hw_ns}/joint_states', 10)
        rate = self.get_parameter('joint_state_publish_rate').value
        self._timer = self.create_timer(
            1.0 / rate, self._publish_js,
            callback_group=self._timer_cb_group)

        self._action_server = ActionServer(
            self,
            GripperCommand,
            'parallel_gripper_controller/gripper_action',
            execute_callback=self._execute_cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._action_cb_group,
        )
        self.get_logger().info('Parallel gripper driver ready')

    async def _sleep(self, seconds: float) -> None:
        # asyncio.sleep requires a running asyncio event loop, which rclpy's
        # MultiThreadedExecutor does not provide (it drives coroutines manually
        # via .send()). Instead, resolve an rclpy Future via a one-shot timer.
        f: Future = Future()
        def _on_timer():
            if not f.done():
                f.set_result(None)
        timer = self.create_timer(seconds, _on_timer)
        await f
        self.destroy_timer(timer)

    async def _ensure_initialized(self):
        if self._initialized:
            return

        for cli in (self._baud_cli, self._timeout_cli, self._digital_cli, self._modbus_cli):
            while not cli.service_is_ready():
                self.get_logger().info(f'Waiting for {cli.srv_name}...')
                await self._sleep(1.0)
            self.get_logger().debug(f'Service ready: {cli.srv_name}')

        self.get_logger().debug('Setting modbus baudrate to 115200')
        req = SetInt32.Request()
        req.data = 115200
        resp = await self._baud_cli.call_async(req)
        self.get_logger().debug(f'set_tgpio_modbus_baudrate -> ret={resp.ret}')
        if resp.ret != 0:
            raise RuntimeError(f'set_tgpio_modbus_baudrate failed ret={resp.ret}')

        self.get_logger().debug('Setting modbus timeout to 50 ms')
        req = SetModbusTimeout.Request()
        req.timeout = 50
        resp = await self._timeout_cli.call_async(req)
        self.get_logger().debug(f'set_tgpio_modbus_timeout -> ret={resp.ret}')
        if resp.ret != 0:
            raise RuntimeError(f'set_tgpio_modbus_timeout failed ret={resp.ret}')

        for ionum in (0, 1):
            self.get_logger().debug(f'Setting digital IO {ionum} = 1')
            req = SetDigitalIO.Request()
            req.ionum = ionum
            req.value = 1
            resp = await self._digital_cli.call_async(req)
            self.get_logger().debug(f'set_tgpio_digital({ionum}) -> ret={resp.ret}')
            if resp.ret != 0:
                raise RuntimeError(f'set_tgpio_digital({ionum}) failed ret={resp.ret}')

        self._initialized = True
        self.get_logger().info('Gripper hardware initialized')

    def _publish_js(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # drive_joint (+Y) and right_finger_joint (-Y axis) both publish same position value;
        # robot_state_publisher applies the joint axis direction automatically.
        msg.name = ['drive_joint', 'right_finger_joint']
        msg.position = [-self._last_pos, self._last_pos]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        self._js_pub.publish(msg)

        if not self._initialized:
            # Auto-initialize on the first timer tick that has an executor attached.
            if not self._init_pending and self.executor is not None:
                self._init_pending = True
                self.executor.create_task(self._auto_init())
            return
        if self._pos_future is None or self._pos_future.done():
            self._pos_future = self._gripper.get_pos_async()
            self._pos_future.add_done_callback(self._on_pos_response)

    async def _auto_init(self):
        try:
            await self._ensure_initialized()
        except Exception as e:
            self.get_logger().warn(
                f'Auto-init failed, will retry: {e}', throttle_duration_sec=5.0)
        finally:
            self._init_pending = False

    def _on_pos_response(self, future):
        try:
            self._last_pos = self._gripper.parse_pos(future.result())
            self.get_logger().debug(f'Gripper position update: {self._last_pos:.4f} m')
        except Exception as e:
            self.get_logger().warn(
                f'Failed to read gripper position: {e}', throttle_duration_sec=5.0)

    async def _execute_cb(self, goal_handle):
        self.get_logger().info(
            f'Goal received: position={goal_handle.request.command.position:.4f} m '
            f'max_effort={goal_handle.request.command.max_effort:.2f}')

        await self._ensure_initialized()

        target = max(0.0, min(ParallelGripperOpenRB150.MAX_METERS,
                               goal_handle.request.command.position))
        self.get_logger().debug(f'Clamped target: {target:.4f} m')

        resp = await self._gripper.move_async(target)
        self.get_logger().debug(f'move_async -> ret={resp.ret}')
        if resp.ret != 0:
            self.get_logger().warn(f'Gripper move failed, ret={resp.ret}')

        self.get_logger().debug('Waiting 1 s for gripper to settle')
        await self._sleep(1.0)

        try:
            resp = await self._gripper.get_pos_async()
            actual = self._gripper.parse_pos(resp)
            self._last_pos = actual
            self.get_logger().debug(f'Post-move position read: {actual:.4f} m')
        except Exception:
            actual = target
            self.get_logger().warn('Position read after move failed, using target as actual')

        reached = abs(actual - target) < 0.002
        self.get_logger().info(
            f'Goal done: actual={actual:.4f} m  target={target:.4f} m  '
            f'reached_goal={reached}')

        result = GripperCommand.Result()
        result.position = actual
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = reached  # 2 mm tolerance
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GripperDriverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
