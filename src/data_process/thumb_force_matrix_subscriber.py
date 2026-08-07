"""
订阅 AP001 点阵力话题，并输出拇指 CNN 矩阵。

运行命令：
cd /media/asus/3C2A2653CEF25C8F/rohand_ros2_pkg-main
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/data_process/thumb_force_matrix_subscriber.py
"""

from pprint import pformat
from sys import stdout
from time import monotonic
from typing import Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


FORCE_MATRIX_TOPIC = "/rohand_node/force_matrix"
THUMB_DATA_LENGTH = 60
PRINT_INTERVAL_SECONDS = 0.2

# 数字 n 表示 data[n - 1]，0 表示没有传感器位置。
RIGHT_THUMB_INDEX_MATRIX: tuple[tuple[int, ...], ...] = (
    (1, 2, 0, 3, 0, 4, 5),
    (6, 7, 0, 8, 0, 9, 10),
    (11, 12, 0, 13, 0, 14, 15),
    (16, 17, 0, 18, 0, 19, 20),
    (0, 27, 0, 28, 0, 29, 0),
    (21, 22, 0, 23, 0, 24, 25),
    (0, 0, 32, 0, 34, 0, 0),
)


def build_thumb_force_matrix(values: Sequence[float]) -> list[list[float]]:
    """按照拇指传感器位置，将一维力值转换为 7×7 矩阵。"""
    matrix: list[list[float]] = []
    for row in RIGHT_THUMB_INDEX_MATRIX:
        matrix_row: list[float] = []
        for data_index in row:
            if data_index == 0:
                matrix_row.append(0.0)
                continue
            value_index = data_index - 1
            if value_index >= len(values):
                matrix_row.append(0.0)
                continue
            matrix_row.append(float(values[value_index]))
        matrix.append(matrix_row)
    return matrix


class ThumbForceMatrixSubscriber(Node):
    """接收点阵力话题并输出拇指矩阵。"""

    def __init__(self) -> None:
        super().__init__("thumb_force_matrix_subscriber")
        self.last_print_time_ = 0.0
        self.subscription_ = self.create_subscription(
            Float32MultiArray,
            FORCE_MATRIX_TOPIC,
            self._force_matrix_callback,
            10,
        )
        self.get_logger().info(f"等待话题: {FORCE_MATRIX_TOPIC}")

    def _force_matrix_callback(self, msg: Float32MultiArray) -> None:
        """读取第一组拇指数据，并按点位矩阵输出。"""
        if len(msg.data) < THUMB_DATA_LENGTH:
            self.get_logger().warning(
                f"点阵力数据长度不足: {len(msg.data)}，期望至少 {THUMB_DATA_LENGTH}"
            )
            return

        current_time = monotonic()
        if current_time - self.last_print_time_ < PRINT_INTERVAL_SECONDS:
            return
        self.last_print_time_ = current_time

        thumb_values = msg.data[:THUMB_DATA_LENGTH]
        thumb_matrix = build_thumb_force_matrix(thumb_values)
        # 使用 ANSI 控制符刷新当前位置，避免矩阵持续向下滚动。
        output = "\033[2J\033[H"
        output += "拇指 CNN 力矩阵:\n"
        output += pformat(thumb_matrix, width=100, sort_dicts=False)
        output += "\n"
        stdout.write(output)
        stdout.flush()


def main(args: list[str] | None = None) -> None:
    """启动拇指点阵力订阅节点。"""
    rclpy.init(args=args)
    node = ThumbForceMatrixSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
