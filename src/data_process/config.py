"""
AP001 右手矩阵力传感器的位置配置。

运行检查命令：
cd /media/asus/3C2A2653CEF25C8F/rohand_ros2_pkg-main
PYTHONPATH=src/rohand python3 src/data_process/config.py
"""

from typing import Sequence

import numpy as np

from common.heat_map_dot import HeatMapDot


# 传感器坐标使用 heat_map_dot.py 中的原始坐标，矩阵索引为 [y, x]。
RIGHT_MATRIX_HEIGHT = 340
RIGHT_MATRIX_WIDTH = 470
RIGHT_MATRIX_SHAPE = (RIGHT_MATRIX_HEIGHT, RIGHT_MATRIX_WIDTH)

# 原始示意图尺寸，仅用于记录，不能直接替代传感器坐标矩阵尺寸。
RIGHT_MATRIX_IMAGE_HEIGHT = 953
RIGHT_MATRIX_IMAGE_WIDTH = 937


def _points_to_matrix(
    points: Sequence[tuple[int, int]],
    height: int,
    width: int,
) -> np.ndarray:
    """将传感器坐标转换为 0/1 位置矩阵。"""
    matrix = np.zeros((height, width), dtype=np.uint8)
    for x, y in points:
        if x < 0 or y < 0:
            continue
        if x >= width or y >= height:
            raise ValueError(f"传感器坐标越界: ({x}, {y})")
        matrix[y, x] = 1
    return matrix


_heatmap_dot = HeatMapDot(0)
_heatmap_dot.init_dot_info()
_right_force_points = _heatmap_dot.RIGHT_FORCE_POINT

# 六个区域的二值位置矩阵：0 表示无传感器，1 表示有传感器。
RIGHT_THUMB_MATRIX = _points_to_matrix(
    _right_force_points[0], RIGHT_MATRIX_HEIGHT, RIGHT_MATRIX_WIDTH
)
RIGHT_INDEX_MATRIX = _points_to_matrix(
    _right_force_points[1], RIGHT_MATRIX_HEIGHT, RIGHT_MATRIX_WIDTH
)
RIGHT_MIDDLE_MATRIX = _points_to_matrix(
    _right_force_points[2], RIGHT_MATRIX_HEIGHT, RIGHT_MATRIX_WIDTH
)
RIGHT_RING_MATRIX = _points_to_matrix(
    _right_force_points[3], RIGHT_MATRIX_HEIGHT, RIGHT_MATRIX_WIDTH
)
RIGHT_LITTLE_MATRIX = _points_to_matrix(
    _right_force_points[4], RIGHT_MATRIX_HEIGHT, RIGHT_MATRIX_WIDTH
)
RIGHT_PALM_MATRIX = _points_to_matrix(
    _right_force_points[5], RIGHT_MATRIX_HEIGHT, RIGHT_MATRIX_WIDTH
)

# 第一维顺序：拇指、食指、中指、无名指、小指、手掌。
RIGHT_SENSOR_POSITION_MAP = np.stack(
    [
        RIGHT_THUMB_MATRIX,
        RIGHT_INDEX_MATRIX,
        RIGHT_MIDDLE_MATRIX,
        RIGHT_RING_MATRIX,
        RIGHT_LITTLE_MATRIX,
        RIGHT_PALM_MATRIX,
    ],
    axis=0,
)

RIGHT_SENSOR_NAMES = (
    "thumb",
    "index",
    "middle",
    "ring",
    "little",
    "palm",
)


def _validate_config() -> None:
    """检查矩阵形状、取值范围和六个区域的有效点数量。"""
    if RIGHT_SENSOR_POSITION_MAP.shape != (6, *RIGHT_MATRIX_SHAPE):
        raise ValueError("RIGHT_SENSOR_POSITION_MAP 的形状不正确")
    if not np.isin(RIGHT_SENSOR_POSITION_MAP, (0, 1)).all():
        raise ValueError("位置矩阵只能包含 0 和 1")
    print("矩阵形状:", RIGHT_SENSOR_POSITION_MAP.shape)
    print("六个区域有效点数量:", RIGHT_SENSOR_POSITION_MAP.sum(axis=(1, 2)).tolist())


if __name__ == "__main__":
    _validate_config()
