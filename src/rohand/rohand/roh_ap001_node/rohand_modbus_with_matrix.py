#!/usr/bin/env python3

'''
cd /media/asus/3C2A2653CEF25C8F/rohand_ros2_pkg-main
colcon build
source install/setup.bash
ros2 run rohand rohand_modbus_with_matrix_ap001

发送位置话题：~/target_joint_states
关节状态发布：~/current_joint_states
力觉数据发布：~/force

## 闭合
ros2 topic pub --once /rohand_node/target_joint_states sensor_msgs/msg/JointState \
"{header: {frame_id: 'rohand_2'}, position: [36.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

## 张开
ros2 topic pub --once /rohand_node/target_joint_states sensor_msgs/msg/JointState \
"{header: {frame_id: 'rohand_2'}, position: [36.0, 174.0, 174.0, 174.0, 178.0, 0.0]}"
最后一个数字是侧摆，0是张开

self.joint_timer_ = self.create_timer(1.0 / 30.0, self._publish_joint_states)
self.force_timer_ = self.create_timer(1.0 / 30.0, self._publish_force)
'''

import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

#from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient
from pymodbus import ModbusException

from common.roh_registers_v2 import *
from force_on_rohand import force_chart
from common.heat_map_dot import HeatMapDot


FRAME_ID_PREFIX = 'rohand_'
# ROH-AP001 hardware type
ROH_HARDWARE_TYPE = 0x2001
FORCE_MATRIX_GROUPS = 6
FORCE_MATRIX_WIDTH = 60


class ROHandNode(Node):

    def __init__(self):
        super().__init__('rohand_node')
        self.get_logger().info("node %s init.." % self.get_name())

        self.declare_parameters(
            namespace='',
            parameters=[
                ('port_name', Parameter.Type.STRING),
                ('baudrate', Parameter.Type.INTEGER),
                ('hand_ids', Parameter.Type.INTEGER_ARRAY)
            ]
        )

        self.port_name_ = self.get_parameter_or('port_name', Parameter('port_name', Parameter.Type.STRING, "/dev/ttyUSB0")).value
        self.baudrate_ = self.get_parameter_or('baudrate', Parameter('baudrate', Parameter.Type.INTEGER, 115200)).value
        self.hand_ids_ = self.get_parameter_or('hand_ids', Parameter('hand_ids', Parameter.Type.INTEGER_ARRAY, [2])).value
        self.get_logger().info("port: %s, baudrate: %d, hand_ids: %s" % (self.port_name_, self.baudrate_, str(self.hand_ids_)))

        # 创建并初始化发布者成员属性pub_joint_states_
        self.joint_states_subscriber_ = self.create_subscription(msg_type=JointState, topic="~/target_joint_states", callback=self._joint_states_callback, qos_profile=10)

        # 创建并初始化发布者成员属性pub_joint_states_
        self.joint_states_publisher_ = self.create_publisher(msg_type=JointState, topic="~/current_joint_states", qos_profile=10)
        self.force_publisher_ = self.create_publisher(Float32MultiArray, "~/force", 10)
        self.force_matrix_publisher_ = self.create_publisher(
            Float32MultiArray, "~/force_matrix", 10
        )

        # Initialize modbus
        # A short timeout prevents an unavailable force register from blocking
        # joint-state control indefinitely.
        self.modbus_client_ = ModbusSerialClient(
            port=self.port_name_, baudrate=self.baudrate_, timeout=0.2
        )
        if not self.modbus_client_.connect():
            raise RuntimeError(f"Failed to connect to Modbus device {self.port_name_}")

        # AP001 is fixed to the right-hand dot-matrix force sensor.  The
        # force visualizer reuses this node's client; it never opens the port.
        self.heatmap_dot_ = HeatMapDot(force_chart.TACS_DOT_MATRIX)
        self.heatmap_dot_.init_dot_info()
        force_chart.img_init(1, self.heatmap_dot_)
        force_chart._height, force_chart._width = force_chart._force_img.shape[:2]

        for i in range(10):
            matched_cnt = 0

            for hand_id in self.hand_ids_:
                try:
                    rr = self.modbus_client_.read_holding_registers(ROH_PROTOCOL_VERSION, count=1, slave=hand_id)
                except ModbusException as exc:
                    self.get_logger().error(f"ERROR: exception in pymodbus, {exc}")
                    continue

                if rr.isError():
                    self.get_logger().error(f"ERROR: pymodbus read request, {rr}")
                    continue

                if (rr.registers[0] >> 8) == MODBUS_PROTOCOL_VERSION_MAJOR:
                    matched_cnt += 1
                else:
                    self.get_logger().error("ERROR: major protocol version of rohand {0} is {1}, expected {2}".format(hand_id, rr.registers[0] >> 8, MODBUS_PROTOCOL_VERSION_MAJOR))
                    raise Exception("Protocol version NOT matched")

                try:
                    rr = self.modbus_client_.read_holding_registers(ROH_HW_VERSION, count=1, slave=hand_id)
                except ModbusException as exc:
                    self.get_logger().error(f"ERROR: exception in pymodbus, {exc}")

                if rr.isError():
                    self.get_logger().error(f"ERROR: pymodbus read request, {rr}")
                
                if (rr.registers[0]) == ROH_HARDWARE_TYPE:
                    self.get_logger().info("Force sensor supported.")
                    # Reset force
                    try:
                        wr = self.modbus_client_.write_registers(address=ROH_RESET_FORCE, values=[1], slave=hand_id)
                    except Exception as exc:
                        self.get_logger().error(f"ERROR: exception in pymodbus, {exc}")

            if matched_cnt == len(self.hand_ids_):
                break

            time.sleep(1.0)
            
        if matched_cnt != len(self.hand_ids_):
            raise Exception("Get protocol version failed")

        # All Modbus transactions run in the default single-threaded ROS
        # executor. This keeps one client and one serial port transaction at a
        # time, without sharing /dev/ttyUSB0 between processes or threads.
        self.joint_timer_ = self.create_timer(1.0 / 100.0, self._publish_joint_states)
        self.force_timer_ = self.create_timer(1.0 / 100.0, self._publish_force)


    def _joint_states_callback(self, msg):
        self.get_logger().info("I heard: %s" % msg)

        try:
            hand_id = int(msg.header.frame_id.replace(FRAME_ID_PREFIX, ''))
        except ValueError as e:
            hand_id = 0

        self.get_logger().info("hand_id: %d" % hand_id)

        try:
            index = self.hand_ids_.index(hand_id)
        except ValueError:
            index = -1

        if index >= 0:
            # 设置目标位置（角度值 * 100 → 寄存器值）
            values = []

            for i in range(len(msg.position)):
                value = int(msg.position[i] * 100)  # scale
                if value < 0:
                    value += 65536
                values.append(value)

            try:
                wr = self.modbus_client_.write_registers(address=ROH_FINGER_ANGLE_TARGET0, values=values, slave=hand_id)
            except Exception as exc:
                self.get_logger().error(f"ERROR: exception in pymodbus, {exc}")
                return

            if wr.isError():
                self.get_logger().error(f"ERROR: pymodbus write_register returned an error: ({wr})")
                # raise ModbusException(txt)
                return

    def _publish_joint_states(self):
        for hand_id in self.hand_ids_:
            joint_states = JointState()

            joint_states.header.stamp = self.get_clock().now().to_msg()
            joint_states.header.frame_id = FRAME_ID_PREFIX + str(hand_id)
            joint_states.name = ['thumb', 'index', 'middle', 'ring', 'little', 'thumb_rotation']

            # 读取当前位置
            try:
                rr = self.modbus_client_.read_holding_registers(ROH_FINGER_ANGLE0, count=6, slave=hand_id)
            except Exception as exc:
                self.get_logger().error(f"ERROR: position read failed: {exc}")
                continue

            if rr.isError():
                self.get_logger().error(f"ERROR: position read returned an error: ({rr})")
                continue
            for value in rr.registers:
                if value > 32767:
                    value -= 65536
                joint_states.position.append(value / 100)


            joint_states.velocity = []

            # 读取当前电流
            try:
                rr = self.modbus_client_.read_holding_registers(ROH_FINGER_CURRENT0, count=6, slave=hand_id)
            except Exception as exc:
                self.get_logger().error(f"ERROR: current read failed: {exc}")
                continue

            if rr.isError():
                self.get_logger().error(f"ERROR: current read returned an error: ({rr})")
                continue
            joint_states.effort.extend(float(value) for value in rr.registers)

            joint_states.header.stamp = self.get_clock().now().to_msg()
            self.joint_states_publisher_.publish(joint_states)

    def _publish_force(self):
        """Read the fixed AP001 right-hand dot-matrix force sensor."""
        finger_force = []
        finger_force_sum = []
        try:
            for finger_id, count in enumerate(self.heatmap_dot_.FORCE_VALUE_LENGTH):
                rr = self.modbus_client_.read_holding_registers(
                    ROH_FINGER_FORCE_EX0 + finger_id * FORCE_GROUP_SIZE,
                    count=count,
                    slave=self.hand_ids_[0],
                )
                if rr.isError() or len(rr.registers) != count:
                    self.get_logger().warning(f"Force register group {finger_id} read failed: {rr}")
                    return
                values = []
                for register in rr.registers:
                    values.extend([(register >> 8) & 0xFF, register & 0xFF])
                finger_force.append(values)
                finger_force_sum.append(float(sum(values)))
        except Exception as exc:
            self.get_logger().error(f"Force sensor read failed: {exc}")
            return

        force_msg = Float32MultiArray()
        force_msg.data = finger_force_sum
        self.force_publisher_.publish(force_msg)

        # 发布固定 6×60 的点阵力矩阵，不足部分使用 0 填充。
        force_matrix_msg = Float32MultiArray()
        force_matrix_msg.layout.dim = [
            MultiArrayDimension(
                label="sensor_group",
                size=FORCE_MATRIX_GROUPS,
                stride=FORCE_MATRIX_GROUPS * FORCE_MATRIX_WIDTH,
            ),
            MultiArrayDimension(
                label="dot_index",
                size=FORCE_MATRIX_WIDTH,
                stride=FORCE_MATRIX_WIDTH,
            ),
        ]
        force_matrix_msg.layout.data_offset = 0
        force_matrix_msg.data = []
        for values in finger_force:
            padded_values = values[:FORCE_MATRIX_WIDTH]
            padded_values.extend([0.0] * (FORCE_MATRIX_WIDTH - len(padded_values)))
            force_matrix_msg.data.extend(float(value) for value in padded_values)
        self.force_matrix_publisher_.publish(force_matrix_msg)

        try:
            force_chart.update_heatmap(finger_force, self.heatmap_dot_)
            cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().error(f"Heatmap display update failed: {exc}")

    def destroy_node(self):
        """关闭共享串口连接和热力图窗口。"""
        if hasattr(self, "joint_timer_"):
            self.joint_timer_.cancel()
        if hasattr(self, "force_timer_"):
            self.force_timer_.cancel()
        if hasattr(self, "modbus_client_"):
            self.modbus_client_.close()
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)  # 初始化rclpy

    node = ROHandNode()  # 新建一个节点

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
