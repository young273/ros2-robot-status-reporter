#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32

from tf2_ros import Buffer, TransformListener, TransformException

from robot_status_reporter.msg import DeliveryStatus


class StatusAggregator(Node):
    def __init__(self) -> None:
        super().__init__('status_aggregator')

        # 声明并获取参数
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('robot_frame_id', 'base_link')
        self.declare_parameter('update_rate', 5.0)

        self.global_frame = self.get_parameter('global_frame_id').value
        self.robot_frame = self.get_parameter('robot_frame_id').value
        self.update_rate = self.get_parameter('update_rate').value

        # 状态初始化
        self.battery_level = 0.0
        self.medicine_count = 0
        self.current_pose = Pose()
        self.current_pose.orientation.w = 1.0
        self.pose_initialized = False  # 标志是否已获得有效位姿

        # 【接入口 1：电量】
        # 这里通过 create_subscription 订阅 /battery_state。
        # 你不需要手动调用 battery_state_callback()，rclpy 会在收到
        # sensor_msgs/BatteryState 消息时自动回调这个函数。
        #
        # 这个接入口的用途：
        # - 由电池管理系统（BMS）、电源板、上位机电量驱动节点发布电量数据
        # - 本节点收到后，只负责把“最新电量百分比”保存到 self.battery_level
        #
        # 这个值会在定时器 publish_integrated_status() 中被打包进自定义消息。
        self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data,
        )

        # 【接入口 2：药量】
        # 这里通过 create_subscription 订阅 /medicine_inventory。
        # 同样不需要你手工调用 medicine_inventory_callback()，
        # 只要外部节点持续发布 std_msgs/Int32，本函数就会被自动触发。
        #
        # 这个接入口的用途：
        # - 药盒传感器、计数器、PLC、MCU 或库存管理节点负责发布当前药量
        # - 本节点收到后，把数值保存到 self.medicine_count
        #
        # 最终会在发布周期中写入 DeliveryStatus.medicine_count。
        self.create_subscription(
            Int32,
            '/medicine_inventory',
            self.medicine_inventory_callback,
            qos_profile_sensor_data,
        )

        self.status_pub = self.create_publisher(
            DeliveryStatus,
            '/robot_integrated_status',
            10,
        )

        # 【接入口 3：位置 / 姿态】
        # 这个接入口不是订阅一个普通 topic，而是通过 TF2 监听坐标变换树。
        #
        # 使用方式：
        # - 外部必须有节点持续发布 TF
        # - 只要 TF 树中这条变换存在，lookup_transform 就能拿到实时位姿
        #
        # 这里的 Buffer 是缓存器，TransformListener 是监听器：
        # - Buffer：保存当前可查询到的全部 TF 变换
        # - TransformListener：自动订阅 /tf 和 /tf_static，并写入 Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 根据传入参数设定上报频率
        timer_period = 1.0 / self.update_rate if self.update_rate > 0 else 0.2
        self.timer = self.create_timer(timer_period, self.publish_integrated_status)

        self.get_logger().info(f'StatusAggregator started, publishing at {self.update_rate}Hz. '
                               f'Frames: {self.global_frame} -> {self.robot_frame}')

    def battery_state_callback(self, msg: BatteryState) -> None:
        # 这个函数由 rclpy 自动调用，触发条件是：/battery_state 收到一条 BatteryState 消息。
        # 一般外部电池驱动节点会按照固定频率发布该话题。
        #
        # 你需要“接入硬件电量”时，只要让硬件驱动节点把 BMS/电池板数据转换成
        # sensor_msgs/BatteryState，然后发布到 /battery_state 即可。
        # 本函数会把消息里的 percentage/capacity/charge 归一化成 0~100 的百分比。
        # 优先使用标准字段 percentage（0.0~1.0），并转成百分比
        percentage = msg.percentage

        if math.isnan(percentage) or percentage < 0.0:
            # 某些电池驱动可能不填 percentage，尝试用 charge/capacity 兜底计算
            if msg.capacity > 0.0 and not math.isnan(msg.charge) and not math.isnan(msg.capacity):
                percentage = msg.charge / msg.capacity
            else:
                percentage = 0.0

        self.battery_level = max(0.0, min(100.0, float(percentage * 100.0)))

    def medicine_inventory_callback(self, msg: Int32) -> None:
        # 这个函数由 rclpy 自动调用，触发条件是：/medicine_inventory 收到一条 Int32 消息。
        #
        # 接入方式通常是：
        # - MCU/PLC/库存节点检测到药量变化后，发布 std_msgs/Int32
        # - data 字段直接表示“当前可用药量”
        #
        # 你不需要在这里主动调用这个函数；外部只要持续发布消息，这里就会被更新。
        # 药量不允许负值，做最小值保护
        self.medicine_count = max(0, int(msg.data))

    def _update_pose_from_tf(self) -> None:
        try:
            # 这里是“位置接入口”的核心查询点。
            # lookup_transform() 会去 TF2 缓存里查找指定的变换 (例如 map -> base_link)。
            transform_stamped = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )

            # 将 TF 平移与旋转直接映射到 Pose。
            self.current_pose.position.x = transform_stamped.transform.translation.x
            self.current_pose.position.y = transform_stamped.transform.translation.y
            self.current_pose.position.z = transform_stamped.transform.translation.z
            self.current_pose.orientation = transform_stamped.transform.rotation

            self.pose_initialized = True  # 成功获取过一次位姿

        except TransformException as ex:
            # 如果当前时刻 TF 还不可用，就不强行清零，避免上位机看到跳变。
            # 这里使用 INFO_THROTTLE 防止刷屏。
            self.get_logger().info(
                f'Could not get TF {self.global_frame} -> {self.robot_frame}: {ex}',
                throttle_duration_sec=5.0
            )

    def publish_integrated_status(self) -> None:
        self._update_pose_from_tf()

        # 避免 TF 还未建好时发布错误的 (0,0,0) 位置导致其他节点误判
        if not self.pose_initialized:
            self.get_logger().info('Waiting for valid robot pose before publishing integrated status...', throttle_duration_sec=5.0)
            return

        status_msg = DeliveryStatus()
        status_msg.battery_level = float(self.battery_level)
        status_msg.medicine_count = int(self.medicine_count)
        status_msg.current_pose = self.current_pose

        self.status_pub.publish(status_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
