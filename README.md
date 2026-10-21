# C20D ROS 2 串口驱动

先飞（Xianfei）**C-20D 卧式两轴云台**的 C++17 驱动，面向 **Ubuntu 24.04 / ROS 2 Jazzy**，MIT 许可证。仓库名沿用 `C20C_Ros2Control`，本实现与实机验证对象是 **C20D 卧式**，不代表已验证 C20C 或立式型号。

依据厂家《云台私有协议-XF(A5)V1.0.4》实现 40 字节控制帧和 26 字节反馈帧。运行时的串口、协议和 ROS 节点全部使用 C++；Python 仅用于无硬件集成测试。

## 0.2.0 单位变更

按使用需求，自定义话题统一使用**度 / 度每秒**，保留原字段名：`GimbalCommand.roll/pitch`、`GimbalState` 非 raw 数组、`CarrierData.attitude_rpy`、`CameraData` 偏角，以及 `max_rate` / `roll_limit` / `pitch_limit` 参数。标准 `sensor_msgs/JointState` 按 ROS 约定仍使用弧度；原始帧和 raw 定点字段不变。默认速度上限换算后保持原来约 28.65°/s。

**这是相对 0.1.0 的单位不兼容更新。** 升级后重启驱动及客户端，重新 source 环境；旧脚本里的弧度数值要换成度（乘 180/π），不要直接沿用旧参数文件。按用户要求，这些自定义角度接口明确偏离 ROS 默认 SI 弧度约定。

查看角度（依次为横滚、俯仰、航向，单位度）：

```bash
ros2 topic echo /c20d_driver/state --field relative_angle_rpy --qos-reliability best_effort
```

## 已实现

- 发布完整反馈：相机姿态角、相对角、相机角速度、安装方向、温度就绪、硬件错误、云台状态、指令执行状态、固件原始字节，以及完整原始帧。
- ROS 控制横滚/俯仰：LOCK 姿态角、FPV 相对角、角速度、回中；支持厂家比例速度、载体惯导、相机视场/倍率与点动/目标跟踪协议字段。
- CRC 校验、串流拆包/重同步、独占串口、超时重连、速度指令超时归零、断线不重放旧指令。
- `JointState`、诊断信息、独立串口探测工具、官方报文单元测试和 ROS/伪终端集成测试。

本机已在 `/dev/ttyTHS1` 读到真实反馈并完成小角度运动测试。协议覆盖与实机验证范围不同，见 [验证记录](docs/validation.md)。

## 接线与编译

串口采用 **3.3 V TTL、115200、8N1、无流控**，TX/RX 交叉、共地。供电、引脚位置遵照[官方卧式手册](https://download.s21i.co99.net/22859939/0/4/ABUIABA9GAAgs5e11AYomNuWuwE.pdf)，不要将 RS-232 电平直接接入 TTL 接口。串口路径按自己的设备指定，USB 转串口优先使用 `/dev/serial/by-id/...`。

已安装 ROS 2 Jazzy 和 rosdep、colcon 的环境中：

```bash
git clone https://github.com/wee733/C20C_Ros2Control.git c20d_ros2
cd c20d_ros2
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --packages-up-to c20d_driver --symlink-install
source install/setup.bash
```

若串口没有读写权限，将当前用户加入该设备所属串口组（通常为 `dialout`），重新登录后生效。不要同时运行 probe 和驱动节点，也不要探测已被其他程序占用的串口。本机 `/dev/ttyTHS3` 被其他设备使用，不是本云台端口。

## 启动与读取

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch c20d_driver c20d.launch.xml port:=/dev/ttyTHS1
```

另一个终端同样 source 环境：

```bash
ros2 topic echo /c20d_driver/state --qos-reliability best_effort
ros2 topic echo /c20d_driver/joint_states --qos-reliability best_effort
ros2 topic echo /c20d_driver/diagnostics
```

也可以在节点未运行时检查指定端口：

```bash
ros2 run c20d_driver c20d_probe /dev/ttyTHS1
```

probe 使用指令码 0 轮询约 2 秒并打印有效帧和 CRC 计数。启动节点同样先使用指令码 0，不自动下发电机启用或手动模式指令。通信本身仍会发送控制协议帧，不是被动串口监听。

`control_enabled:=false` 可禁止接收运动话题和设备指令服务，保留轮询和反馈：

```bash
ros2 launch c20d_driver c20d.launch.xml port:=/dev/ttyTHS1 control_enabled:=false
```

## 控制示例

以下角度单位为 **度（°）**，角速度单位为 **度/秒（°/s）**，例如 `pitch: 2.0` 就是 2°。所有订阅话题使用 reliable、volatile、深度 1；状态和关节反馈采用 SensorDataQoS。开始前确认状态新鲜、无硬件错误、机构有活动空间。两轴云台没有独立的航向电机，ROS 运动接口只开放横滚和俯仰。

FPV：横滚 0°、俯仰相对角 +2°（双轴必须同时设为 FPV）：

```bash
ros2 topic pub --once /c20d_driver/command c20d_driver/msg/GimbalCommand \
  '{roll_mode: 2, pitch_mode: 2, roll_type: 0, pitch_type: 0, roll: 0.0, pitch: 2.0}'
```

LOCK：横滚姿态 0°、俯仰姿态 +2°。它与 FPV 的机械相对角目标不同：

```bash
ros2 topic pub --once /c20d_driver/command c20d_driver/msg/GimbalCommand \
  '{roll_mode: 1, pitch_mode: 1, roll: 0.0, pitch: 2.0}'
```

LOCK 俯仰角速度 +3°/s；按 Ctrl+C 停止发布，默认 0.5 秒内驱动将该速度置零：

```bash
ros2 topic pub -r 20 /c20d_driver/command c20d_driver/msg/GimbalCommand \
  '{roll_mode: 1, pitch_mode: 1, roll_type: 2, pitch_type: 2, roll: 0.0, pitch: 3.0}'
```

立即发零速度保持：

```bash
ros2 topic pub --once /c20d_driver/command c20d_driver/msg/GimbalCommand \
  '{roll_type: 2, pitch_type: 2, roll: 0.0, pitch: 0.0}'
```

回到机械相对零位：

```bash
ros2 topic pub --once /c20d_driver/command c20d_driver/msg/GimbalCommand \
  '{roll_mode: 2, pitch_mode: 2, roll: 0.0, pitch: 0.0}'
```

`center_roll` / `center_pitch` 每收到一次 true 消息翻转一次厂家回中触发位，使用 `--once`，不要持续发布 true。

设备服务：

```bash
ros2 service call /c20d_driver/device_command c20d_driver/srv/DeviceCommand '{command: 4}'
```

指令码：1 陀螺校准、2 启用电机、3 关闭电机、4 手动零速度保持、5 点动、6 目标跟踪。**3 会关闭电机，不是速度停止**。`accepted` 仅表示驱动接受排队，执行情况要看 `state.command_code` 和 `state.command_status`（0 执行中、1 成功、2 失败）。校准必须满足手册的静置要求，节点另检查温度就绪。点动/跟踪必须先持续提供新鲜 `CameraData`；它们不是驱动自带的视觉检测算法。

## 数据、方向和限制

| 接口 | 类型 | 内容 |
| --- | --- | --- |
| `/c20d_driver/state` | `c20d_driver/msg/GimbalState` | 全量反馈，角度为度、角速度为度/秒，数组统一 roll/pitch/yaw；raw 数组保留线序 |
| `/c20d_driver/joint_states` | `sensor_msgs/msg/JointState` | 横滚、俯仰相对角，按 ROS 标准保持弧度；不把相机角速度冒充关节速度 |
| `/c20d_driver/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 在线状态、有效帧、CRC/丢弃字节、拒绝指令计数 |
| `/c20d_driver/command` | `c20d_driver/msg/GimbalCommand` | 手动两轴目标 |
| `/c20d_driver/carrier` | `c20d_driver/msg/CarrierData` | 厂家定义载体姿态和 NEU 加速度，不能直接塞入普通 IMU 数据 |
| `/c20d_driver/camera` | `c20d_driver/msg/CameraData` | 垂直视场、变倍、目标水平/垂直偏角 |
| `/c20d_driver/device_command` | `c20d_driver/srv/DeviceCommand` | 设备指令及是否入队 |

反馈保留**厂家坐标与正方向**，单位换算不等于 REP-103 ENU 坐标变换。默认 `frame_id=c20d_vendor`；不发布未经标定的 TF、四元数或 `sensor_msgs/Imu`。固件版本保留原始数值，不猜测显示格式。

在本次立装状态（反馈 `upright=true`）、固件原始值 36 的卧式实机上，FPV 俯仰指令与相对反馈符号相反。因此 ROS 层默认 `fpv_pitch_command_sign=-1.0`，使 FPV 目标符号与相对角反馈一致；**仅影响 FPV 俯仰目标**，LOCK 和速度沿用厂家指令符号，原始反馈不修改。其他固件/安装方向需先验证小角度，可在参数文件设为 `+1.0` 使用原始协议符号。

配置见 [c20d.yaml](src/c20d_driver/config/c20d.yaml)，参数启动后只读：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `update_rate` | 100 Hz | 允许 50–150 Hz；厂家要求至少 50 Hz |
| `feedback_timeout` | 0.5 s | 反馈/指令回应超时后重连，清空旧指令 |
| `command_timeout` | 0.5 s | 速度停止刷新则归零；角度目标继续保持 |
| `auxiliary_timeout` | 0.5 s | 载体失效；比例速度归零；目标失效转手动保持 |
| `max_rate` | 28.64788975654116 °/s | 单轴速度目标上限，可降低 |
| `roll_limit` | 60° | 横滚目标绝对值不超过 60° |
| `pitch_limit` | 120° | 俯仰目标绝对值不超过 120° |
| `control_enabled` | true | 接受运动话题/设备指令 |
| `fpv_pitch_command_sign` | -1.0 | 仅允许 -1.0 / +1.0 |

非有限数、超限、混合 FPV、FPV 速度模式和过期时间戳会被拒绝。横滚/俯仰 FOLLOW 模式按手册禁止。角度软件限制检查的是目标值，不是机械极限保护；接近限位时不要持续发速度指令。断线后的硬件行为取决于云台固件，驱动无法通过已经断开的线发送停止；退出时仅尽力发送零速度保持。

## 测试与参考

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to c20d_driver --symlink-install
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 colcon test --packages-select c20d_driver --event-handlers console_direct+
colcon test-result --verbose
```

测试使用伪终端，不自动操作真实云台。报文字段、官方资料版本、参考仓库和文档勘误见 [协议说明](docs/protocol.md)；实机范围见 [验证记录](docs/validation.md)。

[MIT](LICENSE)。厂商手册及参考项目的权利归各自权利人，本仓库不重新分发厂商 PDF。
