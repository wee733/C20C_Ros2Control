# C20D ROS 2 串口驱动

先飞（Xianfei）**C-20D 卧式两轴云台**的 C++17 驱动，面向 **Ubuntu 24.04 / ROS 2 Jazzy**，MIT 许可证。仓库名沿用 `C20C_Ros2Control`，本实现与实机验证对象是 **C20D 卧式**，不代表已验证 C20C 或立式型号。

依据厂家《云台私有协议-XF(A5)V1.0.4》实现 40 字节控制帧和 26 字节反馈帧。串口、协议和核心 ROS 驱动使用 C++；Foxglove 方向键转发节点按接入需求使用 Python，另外提供 Python 无硬件集成测试。

## 0.3.0 接口与方向更新

控制按用途拆分，横滚和俯仰仍在同一条命令中原子更新：相对角（FPV）、姿态角（LOCK）、真实角速度（LOCK）、比例角速度（LOCK）。话题确定模式，不再填写模式/类型数字。所有面向云台的非 raw 俯仰目标与反馈统一为 **抬头正、低头负**。

自定义角度使用**度**，角速度使用**度/秒**，新消息用 `_deg` / `_deg_s` 标明单位。标准 `JointState` 仍为弧度。用户要求的单位和俯仰方向不等同于 ROS REP-103 欧拉角约定，不应直接作为 ROS 姿态/IMU 向量。

**这是相对 0.2.0 的不兼容更新：**

- 删除 `/c20d_driver/command` 订阅及 `GimbalCommand`，改用下表中的四个控制接口。不同轴使用不同控制类型的旧组合不再提供。
- 删除 `fpv_pitch_command_sign` 参数。所有新控制话题直接按已验证实机的抬头正方向编码；无需用户补偿符号。
- 为保持相同的物理目标，旧 FPV 命令的 `pitch` 取反后填入新 `pitch_deg`；旧 LOCK 姿态/速度目标只需迁移字段，不取反。此规则针对旧默认符号参数 `-1.0`。
- `state.relative_angle_rpy[1]`、`state.camera_rate_rpy[1]` 和 `joint_states.position[1]` 相对 0.2.0 取反；姿态角反馈、横滚/航向及所有 raw 字段保持原符号。
- 编译后停止旧节点、重启驱动和客户端，并重新 source `install/setup.bash`。不要混用旧消息定义与新节点；清理旧配置中的符号参数。

查看相对俯仰（度，正值抬头）：

```bash
ros2 topic echo /c20d_driver/state/relative_angle --qos-reliability best_effort
```

## 已实现

- 发布完整反馈：相机姿态角、相对角、相机角速度、安装方向、温度就绪、硬件错误、云台状态、指令执行状态、固件原始字节，以及完整原始帧。
- 独立 ROS 接口控制横滚/俯仰：LOCK 姿态角、FPV 相对角、角速度、回中；支持厂家比例速度、载体惯导、相机视场/倍率与点动/目标跟踪协议字段。
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

## Foxglove Publish 面板

驱动启动就创建下列回显发布者，供 bridge 发现控制消息类型；不发送启动占位消息，不因提供类型而触发运动。无需运行额外的 `command_schema` 发布进程。

| Publish 面板目标话题（控制） | 消息类型 | 回显话题（观察/提供类型） |
| --- | --- | --- |
| `/c20d_driver/command/relative_angle` | `c20d_driver/msg/RelativeAngleCommand` | `/c20d_driver/command_echo/relative_angle` |
| `/c20d_driver/command/attitude` | `c20d_driver/msg/AttitudeCommand` | `/c20d_driver/command_echo/attitude` |
| `/c20d_driver/command/angular_rate` | `c20d_driver/msg/AngularRateCommand` | `/c20d_driver/command_echo/angular_rate` |
| `/c20d_driver/command/proportional_rate` | `c20d_driver/msg/ProportionalRateCommand` | `/c20d_driver/command_echo/proportional_rate` |

例如选择相对角控制话题和 `RelativeAngleCommand`，消息填 `{"roll_deg": 0.0, "pitch_deg": -2.0}`，表示相对中位低头 2°。Publish 面板应发往 **`command/...`**，不要发往 `command_echo/...`。速度命令需持续刷新，否则按看门狗归零；角度命令发布一次即可。

回显只包含通过驱动校验、已接受的原始输入，保留输入时间戳。每种回显 reliable + transient_local、深度 1，稍后连接的兼容订阅者可读取该类型最后一条输入。它不是设备执行成功确认，也不是当前生效目标：切换控制类型、速度超时或断线后，历史回显可能仍存在。执行结果看 `state` 和设备实际反馈。拒绝的输入不回显。

bridge 进程必须在加载本工作区 `install/setup.bash` 的环境中启动，话题白名单需包含 `command_echo/...`，客户端发布白名单需允许 `command/...`。升级后重启驱动，Foxglove 重新连接以刷新类型。旧 `/command` 和 `GimbalCommand` 已在 0.3.0 移除，不应再配置到 Publish 面板中。

## Foxglove 方向键转发

`ros2 launch c20d_driver c20d.launch.xml` 默认同时启动 `gimbal_teleop_relay`。驱动已运行时，也可以单独启动一次（不要重复运行多个转发节点）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run c20d_driver gimbal_teleop_relay.py
```

启动驱动但不启用转发：`ros2 launch c20d_driver c20d.launch.xml teleop_enabled:=false`。

- 输入 `/gimbal_teleop/cmd`，类型 `geometry_msgs/msg/Twist`。
- `angular.x` 原样转为 `AngularRateCommand.roll_deg_s`；`angular.y` 原样转为 `pitch_deg_s`。
- 普通方向输入输出到 `/c20d_driver/command/angular_rate`，加上当前 ROS 时间戳；回中输入输出到 `/c20d_driver/command/relative_angle`。
- **该专用 Twist 话题的 angular.x/y 使用度/秒**，是面板约定，不是标准 Twist 的 rad/s。不要接到其他底盘速度话题。`linear.z > 0.5` 是回中信号；忽略 linear.x/y 和 angular.z。
- 面板按住时以 20 Hz 发布 ±10°/s，松手/停止按钮发布 angular.x/y 均为 0；俯仰正值抬头。横滚按 x 原符号转发，左右按钮的物理方向需按实际安装核对。
- 中间按钮 `linear.z > 0.5` 的按下沿只发送一次 `RelativeAngleCommand`，两轴目标为 0°，`center_roll` / `center_pitch` 都为 true。它在驱动回调中原子替换旧速度目标，不在不同话题连续发送“停止 + 回中”，避免顺序竞态。按住不重复，松开后再按可重新触发。
- 回中后的零速度松手/空闲包不覆盖回中目标，避免中途停住；新的非零方向输入可接管，随后松开仍发送零速度。重启转发节点会重置按键状态，建议松开按钮后重启。
- 不缓存重发速度；停止输入后驱动按 `command_timeout`（默认 0.5 s）归零。它依赖驱动运行及串口有效，不是硬件急停。
- 输入包含非有限 angular.x/y 或 linear.z 时不转发；深度 1 避免积压旧命令。速度上限由云台驱动的 `max_rate` 校验，不在转发层另做倍率换算。
- `/gimbal_teleop/input_echo` 在启动时提供 Twist 类型定义并回显有限数输入，供 Foxglove 获取 schema。它表示转发节点收到输入，不表示驱动或设备接受了命令；后者看 `/c20d_driver/command_echo/angular_rate`、`/c20d_driver/command_echo/relative_angle` 与实际反馈。

桥接白名单需要允许客户端发布 `/gimbal_teleop/cmd`，并允许广播 `/gimbal_teleop/input_echo`。默认放行全部时无需修改。转发节点不产生周期性保活，避免在网络断流后继续刷新云台速度。

## 控制示例

每条消息同时给出横滚和俯仰，未填写的轴默认目标为零；**省略轴不表示保持该轴原目标**。所有控制话题使用 reliable、volatile、深度 1，状态使用 SensorDataQoS。仅让一个上层控制器发布运动命令：四个入口共用一个执行器，最后被接受的命令整体替换前一个目标，没有跨话题优先级；节点按接收顺序处理。模式切换与双轴目标在同一回调内更新。

FPV 相对角：横滚 0°、相对安装平台抬头 2°：

```bash
ros2 topic pub --once /c20d_driver/command/relative_angle c20d_driver/msg/RelativeAngleCommand \
  '{roll_deg: 0.0, pitch_deg: 2.0}'
```

相对安装平台低头 90°（确保机构活动空间足够后使用）：

```bash
ros2 topic pub --once /c20d_driver/command/relative_angle c20d_driver/msg/RelativeAngleCommand \
  '{roll_deg: 0.0, pitch_deg: -90.0}'
```

相对角零位对应机械中位；安装平台水平、相机中位朝前时，-90° 对应向下看。平台自身倾斜时，它不保证对地垂直向下。FPV 跟随平台并抑制部分晃动；`fpv_sensitivity` 可设置为 -16..15。

LOCK 姿态角：横滚姿态 0°、俯仰姿态 +2°，相对稳定姿态参考，而非机械相对零位：

```bash
ros2 topic pub --once /c20d_driver/command/attitude c20d_driver/msg/AttitudeCommand \
  '{roll_deg: 0.0, pitch_deg: 2.0}'
```

LOCK 真实角速度：以 +3°/s 抬头，横滚速度为零。停止发布后默认 0.5 秒内速度归零：

```bash
ros2 topic pub -r 20 /c20d_driver/command/angular_rate c20d_driver/msg/AngularRateCommand \
  '{roll_deg_s: 0.0, pitch_deg_s: 3.0}'
```

立即发零速度保持当前姿态：

```bash
ros2 topic pub --once /c20d_driver/command/angular_rate c20d_driver/msg/AngularRateCommand \
  '{roll_deg_s: 0.0, pitch_deg_s: 0.0}'
```

回到机械相对零位：

```bash
ros2 topic pub --once /c20d_driver/command/relative_angle c20d_driver/msg/RelativeAngleCommand \
  '{roll_deg: 0.0, pitch_deg: 0.0}'
```

角度消息中的 `center_roll` / `center_pitch` 每收到一次 true 就翻转一次厂家回中位，应使用 `--once`。它们是厂家回中请求；指定机械零位优先使用上面的相对角目标。

LOCK 比例角速度有独立类型 `ProportionalRateCommand`，字段为 `roll_deg_s` / `pitch_deg_s`。该数值是倍率/FOV 换算前的基础速度，实际速度由厂家系数决定，不保证等于输入数值。持续提供真实、新鲜且非零的 `CameraData.vertical_fov_deg` / `zoom` 后，可发布：

```bash
ros2 topic pub -r 20 /c20d_driver/command/proportional_rate c20d_driver/msg/ProportionalRateCommand \
  '{roll_deg_s: 0.0, pitch_deg_s: -3.0}'
```

设备服务：

```bash
ros2 service call /c20d_driver/device_command c20d_driver/srv/DeviceCommand '{command: 4}'
```

指令码：1 陀螺校准、2 启用电机、3 关闭电机、4 手动零速度保持、5 点动、6 目标跟踪。**3 会关闭电机，不是速度停止**。`accepted` 仅表示驱动接受排队，执行情况看 `state.command_code` 和 `state.command_status`（0 执行中、1 成功、2 失败）。校准需静置且温度就绪。点动/跟踪必须持续提供新鲜 `CameraData`，不是驱动自带视觉算法。发布上述任意运动命令会切回手动控制。

## 数据、方向和限制

下表话题均位于 `/c20d_driver/` 下。

| 接口 | 消息类型（c20d_driver/msg，另有说明除外） | 含义 |
| --- | --- | --- |
| `command/relative_angle` | `RelativeAngleCommand` | 双轴 FPV 相对角目标，度 |
| `command/attitude` | `AttitudeCommand` | 双轴 LOCK 稳定姿态目标，度 |
| `command/angular_rate` | `AngularRateCommand` | 双轴 LOCK 真实角速度，度/秒 |
| `command/proportional_rate` | `ProportionalRateCommand` | 双轴 LOCK 比例速度基础值，度/秒 |
| `state/relative_angle` | `AngleState` | 编码器解算相对角：`roll_deg/pitch_deg/yaw_deg` |
| `state/attitude` | `AngleState` | 相机稳定姿态：`roll_deg/pitch_deg/yaw_deg` |
| `state/angular_rate` | `AngularRateState` | 相机 IMU 本体角速度：`roll_deg_s/pitch_deg_s/yaw_deg_s` |
| `state` | `GimbalState` | 同一串口帧的完整状态：归一方向后的数组及原始字段 |
| `joint_states` | `sensor_msgs/msg/JointState` | 相对横滚/俯仰，弧度；俯仰抬头正，不发布伪造的关节速度 |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 在线、帧数、CRC、拒绝指令计数 |
| `carrier` | `CarrierData` | 外部载机姿态及 NEU 加速度，仍遵循厂家载机坐标 |
| `camera` | `CameraData` | FOV、倍率、图像目标偏角，垂直偏角仍为向下正 |
| `device_command` | `c20d_driver/srv/DeviceCommand` | 设备指令服务及是否入队 |

三种拆分反馈与完整 `state` 从同一有效帧产生，使用相同接收时间戳。相对角来自编码器，姿态角来自厂家姿态估计；两者的零位和参考不同，不能因正方向一致就当作相同角度。相机 IMU 本体角速度不是机械关节速度，也不是一般多轴运动下的欧拉角导数。

**俯仰映射（已验证的立装 C20D、固件原始值 36）：**

| 数据 | ROS 数值与原始协议数值的关系（先换算单位） |
| --- | --- |
| FPV / LOCK 角度目标 | ROS pitch = 线协议 pitch |
| 真实 / 比例速度目标 | ROS pitch = 线协议 pitch |
| 相对俯仰反馈 | ROS pitch = −原始相对 pitch |
| 姿态俯仰反馈 | ROS pitch = 原始姿态 pitch |
| 相机俯仰角速度反馈 | ROS pitch rate = −原始 IMU pitch rate |

横滚/航向保留厂家符号；两轴卧式型号不开放航向控制。完整状态中的 `*_raw_*` 数组和 `raw_frame` 始终不改符号，仍按原线序保存厂家数据。`carrier` 的载机姿态与 `camera` 的图像偏角是另外的协议量，不应套用云台俯仰转换。

默认 `frame_id=c20d_head_up` 是本驱动约定的标识，**不是已标定的 REP-103 坐标系**，没有自动生成 TF、四元数或 `sensor_msgs/Imu`。`JointState` 的俯仰关节正轴需在 URDF 中定义为抬头方向；典型 x 前/y 左/z 上的安装对应绕 −y 轴，不能仅因使用弧度就假定绕 +y。其他固件或吊装方向尚未验证，不能直接推广本次映射。

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

非有限数、超限、无效 FPV 灵敏度和过期/未来时间戳会被拒绝。拆分接口不允许混合 FPV 或 FPV 速度模式。横滚/俯仰 FOLLOW 模式按手册禁止。角度软件限制检查的是目标值，不是机械极限保护；接近限位时不要持续发速度指令。断线后的硬件行为取决于云台固件，驱动无法通过已经断开的线发送停止；退出时仅尽力发送零速度保持。

## 测试与参考

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to c20d_driver --symlink-install
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 colcon test --packages-select c20d_driver --event-handlers console_direct+
colcon test-result --verbose
```

测试使用伪终端，不自动操作真实云台。报文字段、官方资料版本、参考仓库和文档勘误见 [协议说明](docs/protocol.md)；实机范围见 [验证记录](docs/validation.md)。

[MIT](LICENSE)。厂商手册及参考项目的权利归各自权利人，本仓库不重新分发厂商 PDF。
