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
FINGER_DATA_LENGTH = 60
REQUIRED_DATA_LENGTH = FINGER_DATA_LENGTH * 3
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

# 按实际拇指点阵排序：上下翻转、去除多余点位，形成 6×5 矩阵。
# 数字 n 仍表示原始 data[n - 1]，0 表示没有传感器位置。
RIGHT_THUMB_CNN_INDEX_MATRIX: tuple[tuple[int, ...], ...] = (
    (0, 32, 34, 0, 0),
    (21, 22, 23, 24, 25),
    (16, 17, 18, 19, 20),
    (11, 12, 13, 14, 15),
    (6, 7, 8, 9, 10),
    (1, 2, 3, 4, 5),
)

# 食指原始点位索引矩阵，来源于 RIGHT_FORCE_POINT。
RIGHT_INDEX_INDEX_MATRIX: tuple[tuple[int, ...], ...] = (
    (0, 2, 0, 3, 0, 4, 0),
    (0, 7, 0, 8, 0, 9, 0),
    (0, 12, 0, 13, 0, 14, 0),
    (0, 17, 0, 18, 0, 19, 0),
    (0, 22, 0, 23, 0, 24, 0),
    (26, 0, 0, 0, 0, 0, 30),
    (0, 27, 0, 28, 0, 29, 0),
    (31, 0, 0, 0, 0, 0, 35),
    (0, 32, 0, 33, 0, 34, 0),
    (36, 0, 0, 0, 0, 0, 40),
    (0, 37, 0, 38, 0, 39, 0),
    (41, 0, 0, 0, 0, 0, 45),
    (0, 42, 0, 43, 0, 44, 0),
    (46, 0, 0, 0, 0, 0, 50),
    (0, 47, 0, 48, 0, 49, 0),
    (0, 52, 0, 53, 0, 54, 0),
    (0, 0, 58, 0, 59, 0, 0),
)

# 中指原始点位索引矩阵，来源于 RIGHT_FORCE_POINT。
RIGHT_MIDDLE_INDEX_MATRIX: tuple[tuple[int, ...], ...] = (
    (0, 2, 0, 3, 0, 4, 0),
    (0, 7, 0, 8, 0, 9, 0),
    (0, 12, 0, 13, 0, 14, 0),
    (0, 17, 0, 18, 0, 19, 0),
    (26, 0, 0, 0, 0, 0, 30),
    (0, 22, 0, 23, 0, 24, 0),
    (31, 0, 0, 0, 0, 0, 35),
    (0, 27, 0, 28, 0, 29, 0),
    (36, 0, 0, 0, 0, 0, 40),
    (0, 32, 0, 33, 0, 34, 0),
    (41, 0, 0, 0, 0, 0, 45),
    (0, 37, 0, 38, 0, 39, 0),
    (46, 0, 0, 0, 0, 0, 50),
    (0, 42, 0, 43, 0, 44, 0),
    (0, 47, 0, 48, 0, 49, 0),
    (0, 52, 0, 53, 0, 54, 0),
    (0, 0, 58, 0, 59, 0, 0),
)

# 按 README 中的 CNN 点阵形状上下翻转，并保留 45 个有效点。
RIGHT_INDEX_CNN_INDEX_MATRIX: tuple[tuple[int, ...], ...] = (
    (0, 0, 58, 59, 0),
    (0, 52, 53, 54, 0),
    (0, 47, 48, 49, 0),
    (46, 50, 42, 43, 44),
    (41, 45, 37, 38, 39),
    (36, 40, 32, 33, 34),
    (31, 35, 27, 28, 29),
    (26, 30, 22, 23, 24),
    (0, 17, 18, 19, 0),
    (0, 12, 13, 14, 0),
    (0, 7, 8, 9, 0),
    (0, 2, 3, 4, 0),
)

RIGHT_MIDDLE_CNN_INDEX_MATRIX: tuple[tuple[int, ...], ...] = (
    (0, 0, 58, 59, 0),
    (0, 52, 53, 54, 0),
    (0, 47, 48, 49, 0),
    (42, 43, 44, 46, 50),
    (37, 38, 39, 41, 45),
    (32, 33, 34, 36, 40),
    (27, 28, 29, 31, 35),
    (22, 23, 24, 26, 30),
    (0, 17, 18, 19, 0),
    (0, 12, 13, 14, 0),
    (0, 7, 8, 9, 0),
    (0, 2, 3, 4, 0),
)

def build_force_matrix(
    values: Sequence[float],
    index_matrix: Sequence[Sequence[int]],
) -> list[list[float]]:
    """按照传感器索引矩阵，将一维力值转换为 CNN 矩阵。"""
    matrix: list[list[float]] = []
    for row in index_matrix:
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


def build_thumb_force_matrix(values: Sequence[float]) -> list[list[float]]:
    """按照实际拇指传感器位置，将一维力值转换为 6×5 矩阵。"""
    return build_force_matrix(values, RIGHT_THUMB_CNN_INDEX_MATRIX)


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
        if len(msg.data) < REQUIRED_DATA_LENGTH:
            self.get_logger().warning(
                f"点阵力数据长度不足: {len(msg.data)}，"
                f"期望至少 {REQUIRED_DATA_LENGTH}"
            )
            return

        current_time = monotonic()
        if current_time - self.last_print_time_ < PRINT_INTERVAL_SECONDS:
            return
        self.last_print_time_ = current_time

        thumb_values = msg.data[0:FINGER_DATA_LENGTH]
        index_values = msg.data[FINGER_DATA_LENGTH:FINGER_DATA_LENGTH * 2]
        middle_values = msg.data[FINGER_DATA_LENGTH * 2:REQUIRED_DATA_LENGTH]
        thumb_matrix = build_thumb_force_matrix(thumb_values)
        index_matrix = build_force_matrix(
            index_values, RIGHT_INDEX_CNN_INDEX_MATRIX
        )
        middle_matrix = build_force_matrix(
            middle_values, RIGHT_MIDDLE_CNN_INDEX_MATRIX
        )
        # 使用 ANSI 控制符刷新当前位置，避免矩阵持续向下滚动。
        output = "\033[2J\033[H"
        output += "拇指 CNN 力矩阵 (6×5):\n"
        output += pformat(thumb_matrix, width=100, sort_dicts=False) + "\n\n"
        output += "食指 CNN 力矩阵 (12×5):\n"
        output += pformat(index_matrix, width=100, sort_dicts=False) + "\n\n"
        output += "中指 CNN 力矩阵 (12×5):\n"
        output += pformat(middle_matrix, width=100, sort_dicts=False) + "\n"
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
