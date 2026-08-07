# 仓库贡献指南

## 项目结构与模块组织

本仓库是一个包含 `rohand` ROS 2 `ament_python` 功能包的工作空间。

- `src/rohand/scripts/roh_ap001_node/` 包含 AP001 灵巧手的 Modbus、串口和 CAN 可执行节点。
- `src/rohand/common/` 包含共享寄存器映射，以及不同版本的串口/CAN 协议实现。
- `src/rohand/test/` 包含版权、Flake8 和 PEP 257 的 ROS 软件包代码检查测试。
- `src/rohand/resource/`、`package.xml`、`setup.py` 和 `setup.cfg` 定义软件包元数据及安装行为。
- `README.md`、`AGENTS.md` 和 `requirements.txt` 位于工作空间根目录。
- `build/`、`install/` 和 `log/` 由 colcon 生成，不应编辑或提交。

## 构建、测试与开发命令

在仓库根目录执行以下命令，加载 ROS 2 环境并构建软件包：

```bash
source /opt/ros/<distro>/setup.bash
colcon build
source install/setup.bash
```

使用 `ros2 run rohand rohand_modbus_ap001` 运行 AP001 Modbus 节点。硬件测试可能需要
执行 `sudo chmod 666 /dev/ttyUSB0`；请使用实际设备，并避免提交特定机器的配置。


使用 `colcon test --packages-select rohand` 运行软件包测试，然后使用
`colcon test-result --verbose` 查看结果。若只运行本地测试，请在包含 ROS 2 测试依赖的
环境中执行 `pytest src/rohand/test/`。

## 编码风格与命名约定

使用 Python 编写代码，采用 4 个空格缩进，单行最多 100 个字符，并为每个函数添加类型
注解。模块、函数和变量优先使用 `snake_case`；类使用 `PascalCase`；常量使用大写名称。
在文件名和模块中明确区分不同协议版本。提交修改前运行仓库的 Flake8 和 PEP 257 检查。

## 测试规范

在 `test/` 下添加测试，文件名使用 `test_*.py`，测试函数使用 `test_*`。新增协议或寄存器
映射功能时，应尽可能添加针对性测试。所有未跳过的代码检查测试必须通过；新增源文件时，
请保留软件包要求的版权声明头。


