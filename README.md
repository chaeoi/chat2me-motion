# Chat2Me Motion

通过 Unix socket 接收动作请求，生成头部与双臂的连续轨迹，并向 RL 控制器发布统一的 17 关节指令。底层电机命令由 `debug_sim2real_wbc_23dof_yuquan_throw_kpkd_zero_cmd_vel_ros2_head.py` 负责。

控制链路：`chat2me-core → /action → /throw/arm_joint_command → _head.py → 电机驱动`。

## 构建

需要 ROS 2 Humble、C++17、CMake、`libyaml-cpp-dev`。JSON 解析使用仓库内的 picojson，不需要安装额外 JSON 库。

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

没有 ROS 的环境也可运行全部控制逻辑和协议测试：

```bash
cmake -S . -B build-offline -DCHAT2ME_MOTION_WITH_ROS=OFF
cmake --build build-offline -j2
ctest --test-dir build-offline --output-on-failure
```

测试使用内存中的模拟反馈和临时 Unix socket，不创建 ROS 节点、不访问电机。`motion_offline_server` 仅供 `chat2me` 的跨仓库协议测试使用，也不链接 ROS。

## 配置与运行条件

配置示例见 `config/config.example.yml`。启动时必须指定可读配置；缺失配置或旧版 `neck` / `pt_control_topic` 配置会明确报错。旧版配置需要按新示例迁移，不能只修改话题名称。

- 反馈：`/motor/q2w_upper_motor_joint_state`，32 关节 `JointState`，订阅兼容 BestEffort 和 Reliable。
- 输出：`/throw/arm_joint_command`，使用左臂 7、右臂 7、头 yaw/pitch/roll 3 的顺序。`position=[q(17),kp(17)]`，`velocity=[qd(17),kd(17)]`，`effort=tau(17)`；当前前馈力矩为零。
- `_head.py` 需要处于 disturb 策略的 X-MANUAL 模式。此服务的 `202 accepted` 表示轨迹已接收，不表示机器人已经执行。
- 服务启动或仅收到反馈时不发指令。首个有效动作从当前反馈姿态初始化全部 17 个关节；动作结束后持续发布保持目标。头部动作保留双臂已设定的目标和增益。
- 反馈超过 `feedback_timeout_seconds` 未更新时停止发布并取消轨迹；反馈恢复后进入 `waiting_action`，需要新的动作请求，不会续播旧动作。
- `joints.lower/upper` 和 `max_velocity` 均为 17 维，单位为 rad 和 rad/s。示例的手臂 ±π 是数值边界，需要替换为实机机械限位。头部 yaw 始终不能配置到 ±70° 以外。
- 不应另开一个发布器分别发送手臂指令；头臂目标需要经此服务统一维护，避免在 `_head.py` 中互相覆盖。

以下命令会运行正式服务，仅在计划进行实机控制时使用：

```bash
build/chat2me-motion run --config /path/to/config.yml
```

`service install --no-start` 只安装文件与 unit，不启用或启动服务。`service install` 安装并启用、重启服务；`--force-config` 明确覆盖已存在的配置。安装不自动把旧配置转换为新配置。

## HTTP 接口

通过配置中的 Unix socket 使用标准 HTTP。仅支持 `GET /health`、`POST /action`、`POST /stop`。

动作请求：

```json
{"id":"act-123","name":"nod","params":{},"preempt":true,"timeout_ms":10000}
```

成功响应为 `202`：

```json
{"ok":true,"status":"accepted","id":"act-123","action":"nod"}
```

`preempt=true` 以当前插值位置重新规划；`false` 时若有动作正在执行则返回 `409`。不积压过时动作。`timeout_ms` 范围为 1–30000；超过配置速度时轨迹自动延长，若总时长超过请求超时则在执行前返回 `422`。每段使用五次多项式插值，同时输出位置和解析速度。

内置动作：`nod`、`shake`、`look_around`、`think`、`look_at`。`look_at` 从 `params.direction_deg` 读取声源方向，约定 0°=正前、90°=右、270°=左。角度会转换为机器人 yaw 并限幅。兼容旧请求中的顶层 `direction_deg`，两处同时提供会拒绝。

直接设置头臂目标使用 `move_joints`，`params` 可提供 `left_arm`（7 维）、`right_arm`（7 维）、`head`（3 维）以及 `duration_ms`。省略的关节保持原目标。示例仅展示协议：

```json
{"id":"pose-1","name":"move_joints","params":{"left_arm":[0,0,0,0,0,0,0],"right_arm":[0,0,0,0,0,0,0],"head":[0,0,0],"duration_ms":2000},"preempt":true,"timeout_ms":10000}
```

自定义手臂动作在 `actions` 下定义为 waypoint 列表，每步包含 `duration_seconds` 和需要改变的 `left_arm`、`right_arm`、`head`。数值需要来自已标定的动作。配置后以动作名称调用 `/action`；启动检查会拒绝越界、非有限数值、空轨迹或覆盖内置名称。

`/health` 返回反馈年龄、发布状态、当前动作 ID、状态及支持的动作列表。没有新鲜反馈时返回 `503`。动作状态为 `running`、`completed`、`timed_out`、`stopped` 等，反映轨迹生成状态，不代替电机到位反馈。

`POST /stop` 取消当前轨迹并以零目标速度保持当前插值位置。未知动作、错误参数和越界目标返回 `422`；无有效反馈返回 `503`；错误 JSON 或 HTTP 返回 `400`。

## 第三方代码

`third_party/picojson.h` 来自 [picojson v1.3.0](https://github.com/kazuho/picojson/tree/v1.3.0)，保留原始 BSD 许可证；随二进制安装的许可证见 `third_party/NOTICE`。
