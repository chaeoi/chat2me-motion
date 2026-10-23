# Chat2Me Motion 使用指南

Chat2Me Motion 接收语音系统或 HTTP 客户端的动作请求，控制机器人头部与双臂。需要配合支持 17 关节头臂指令的 RL 控制器使用，动作通过 `/throw/arm_joint_command` 下发。

## 下载安装

从 [GitHub Releases](https://github.com/chaeoi/chat2me-motion/releases) 下载预编译程序，按机器人主机的 CPU 架构选择：

| `uname -m` 输出 | 下载文件 |
| --- | --- |
| `aarch64`、`arm64` | `chat2me-motion-linux-arm64` |
| `x86_64`、`amd64` | `chat2me-motion-linux-amd64` |

以下命令自动选择架构，下载最新版本：

```bash
case "$(uname -m)" in
  aarch64|arm64) motion_arch=arm64 ;;
  x86_64|amd64) motion_arch=amd64 ;;
  *) echo "不支持的 CPU 架构"; exit 1 ;;
esac

curl -fL "https://github.com/chaeoi/chat2me-motion/releases/latest/download/chat2me-motion-linux-${motion_arch}" -o chat2me-motion
chmod +x chat2me-motion
./chat2me-motion --version
```

下载后的文件名可以自定，安装器会统一复制为 `/opt/chat2me/motion/chat2me-motion`。例如将下载文件命名为 `motion`，执行的安装命令就是 `sudo ./motion service install --no-start`；systemd 使用的是安装后的正式文件。

`/opt` 下的安装目录通常由 root 管理。直接下载到该目录时需要 `sudo wget` 或 `sudo curl`；上面的下载命令可先在用户有写权限的目录执行，再由安装器写入正式位置。

运行环境为 Ubuntu 22.04、ROS 2 Humble，需要安装 yaml-cpp 运行库；Ubuntu 22.04 的包名为 `libyaml-cpp0.7`。

先安装程序和 systemd 服务文件，暂不启动：

```bash
sudo ./chat2me-motion service install --no-start
```

安装位置：

| 内容 | 路径 |
| --- | --- |
| 程序 | `/opt/chat2me/motion/chat2me-motion` |
| 配置 | `/opt/chat2me/motion/config.yml` |
| systemd 服务 | `chat2me-motion.service` |

首次安装会生成配置文件；已有配置会保留。

## 配置机器人连接

编辑配置：

```bash
sudoedit /opt/chat2me/motion/config.yml
```

主要配置如下，完整参数见 [配置示例](config/config.example.yml)：

```yaml
server:
  socket_path: /opt/chat2me/motion/motion.sock

ros:
  domain_id: 0
  localhost_only: false
  joint_state_topic: /motor/q2w_upper_motor_joint_state
  joint_command_topic: /throw/arm_joint_command
  control_hz: 50
  feedback_timeout_seconds: 0.5

arm_gestures: true
actions: {}
```

`domain_id` 必须与电机驱动、RL 控制器一致。`joint_state_topic` 接收 32 关节反馈，`joint_command_topic` 连接头臂控制器。动作生效需要 RL 控制器处于 **disturb 策略的 X-MANUAL 模式**。

关节参数在 `joints` 下配置，顺序为左臂 7、右臂 7、头部 yaw/pitch/roll 3：

| 参数 | 含义 |
| --- | --- |
| `kp`、`kd` | 每个关节的位置、速度增益，各 17 项 |
| `lower`、`upper` | 每个关节的角度下限、上限，单位 rad，各 17 项 |
| `max_velocity` | 每个关节的最大目标速度，单位 rad/s，共 17 项 |

关节限位按 qj2m 的 `pin_model/model.urdf` 设置；头部 yaw 额外限制在 ±70° 内。实机限位与模型不同时按实际数据调整。`arm_gestures: true` 为点头、侧头启用肩、肘、腕七关节配合，先外展再抬臂，分阶段转腕后收回到动作前的姿态；不控制灵巧手和手指。设为 `false` 可只测试头部。修改配置后重启服务生效。

旧版包含 `neck` 或 `pt_control_topic` 的配置需要按新示例迁移。`--force-config` 会覆盖已有配置，仅在需要重置时使用。

## 启动与检查

启动服务并设置开机启动：

```bash
sudo systemctl enable --now chat2me-motion.service
```

查看服务状态和日志：

```bash
/opt/chat2me/motion/chat2me-motion service status
sudo journalctl -u chat2me-motion.service -n 100 --no-pager
```

检查连接状态，此请求不触发动作：

```bash
sudo curl -sS --unix-socket /opt/chat2me/motion/motion.sock http://localhost/health
```

`ready=true` 表示电机反馈可用；没有收到反馈或反馈过期时返回 HTTP `503`。`state` 表示当前轨迹状态，如 `waiting_action`、`running`、`completed`、`timed_out`、`stopped`。`ready` 不表示 RL 控制器已经切到 MANUAL，需要同时确认其控制模式。

服务在收到首个有效动作请求后才开始发布目标。动作结束后保持目标姿态；反馈中断时取消动作并停止发布，恢复后等待新的动作请求。

## 接入 Chat2Me

在 Chat2Me 使用的 env 中配置：

```env
MOTION_ENABLED=1
MOTION_SOCKET=/opt/chat2me/motion/motion.sock
MOTION_CONNECT_TIMEOUT_SECONDS=0.3
MOTION_READ_TIMEOUT_SECONDS=1.0
MOTION_ACTION_TIMEOUT_MS=10000
```

`MOTION_SOCKET` 必须与 motion 配置的 `server.socket_path` 一致。Chat2Me 固定挂载安装目录 `/opt/chat2me/motion`，首次启动会自动生成缺失的 `runtime.env`，无需为 Compose 单独配置 socket 环境变量。修改 socket 文件名时保持在该挂载目录内，并同步两边配置。

语音指令在 Chat2Me 的 `config/profile.yaml` 中配置。例如带 `action: nod` 的条目可以触发点头；标定好的自定义手臂动作也可用同样方式绑定语音指令。

## 手动调用动作

以下 `/action` 请求会执行动作，使用前需完成机器人连接配置并进入 X-MANUAL 模式。示例 socket 路径应替换为实际配置值。

### 头部动作

| 动作名称 | 效果 |
| --- | --- |
| `nod` | 左臂外展、抬起、转腕，配合两次点头后收回；默认约 9.45 秒 |
| `shake` | 只绕头部自旋轴左右摇头 |
| `look_around` | 只绕头部自旋轴左右环顾 |
| `think` | 右臂抬起、转腕，配合歪头停顿后收回；默认约 9.8 秒 |
| `look_at` | 转向指定声源方向，短暂停留后返回原朝向 |

点头：

```bash
sudo curl -sS --unix-socket /opt/chat2me/motion/motion.sock \
  -H 'Content-Type: application/json' \
  -d '{"id":"nod-1","name":"nod","params":{},"preempt":true,"timeout_ms":10000}' \
  http://localhost/action
```

看向右侧声源：

```bash
sudo curl -sS --unix-socket /opt/chat2me/motion/motion.sock \
  -H 'Content-Type: application/json' \
  -d '{"id":"look-1","name":"look_at","params":{"direction_deg":90},"preempt":true,"timeout_ms":10000}' \
  http://localhost/action
```

`direction_deg` 约定为 0°=正前、90°=右、270°=左，头部目标会按配置限幅。左右转头始终保持动作开始时的俯仰和侧倾，不把其他轴清零。所有内置动作结束后返回动作开始时的姿态；点头和侧头的手臂配合也会返回原位。

### 设置头臂目标

使用 `move_joints`，在 `params` 中提供需要改变的关节组：

| 参数 | 格式 |
| --- | --- |
| `left_arm` | 左臂 7 个关节角，单位 rad |
| `right_arm` | 右臂 7 个关节角，单位 rad |
| `head` | `[yaw, pitch, roll]`，对应 URDF 的 `16_neck_yaw`、`17_neck_pitch`、`18_neck_roll`，单位 rad |
| `duration_ms` | 目标运动时长，默认 1000 ms |

三组目标可同时提供，省略的关节保持原目标。手臂角度需使用已标定数据。

两臂各自的关节顺序均为：肩 pitch、肩 roll、肩 yaw、肘 pitch、腕 yaw、腕 pitch、腕 roll。

头部三个关节轴按 URDF 分别为 Z 轴自旋、接近 Y 轴的俯仰、接近 −X 轴的侧倾；在 32 电机数组中对应索引 17、18、19（CAN ID 18、19、20）。

下面示例仅改变头部目标：

```json
{"id":"pose-1","name":"move_joints","params":{"head":[0.1,0,0],"duration_ms":1000},"preempt":true,"timeout_ms":10000}
```

将该 JSON 作为上面 `/action` 请求的请求体即可。

### 自定义动作

在 `config.yml` 的 `actions` 下添加已标定的轨迹。每个动作是一个步骤列表，每步可设置 `left_arm`、`right_arm`、`head` 和 `duration_seconds`。

例如增加一个头部动作：

```yaml
actions:
  look_left:
    - duration_seconds: 1.0
      head: [0.3, 0.0, 0.0]
    - duration_seconds: 1.0
      head: [0.0, 0.0, 0.0]
```

保存并重启服务后，将 `/action` 请求的 `name` 设为 `look_left` 即可调用。自定义动作不能与内置名称重名。

### 接收结果与停止动作

请求接收成功时返回 HTTP `202`：

```json
{"ok":true,"status":"accepted","id":"nod-1","action":"nod"}
```

这表示轨迹已接收，不代表电机已经到位。可通过 `/health` 查看对应动作 ID 和轨迹状态。

`preempt=true` 会替换当前动作；设为 `false` 时，如果已有动作正在执行则返回 `409`。`timeout_ms` 范围为 1–30000。服务会按最大速度限制延长运动时长，若总时长超过请求超时则返回 `422`。

取消当前轨迹并保持当前插值位置：

```bash
sudo curl -sS -d '' --unix-socket /opt/chat2me/motion/motion.sock http://localhost/stop
```

`/stop` 是轨迹停止接口，实机急停仍使用控制器或硬件提供的急停方式。

## 服务管理与升级

| 操作 | 命令 |
| --- | --- |
| 重启并读取新配置 | `sudo systemctl restart chat2me-motion.service` |
| 停止服务 | `sudo systemctl stop chat2me-motion.service` |
| 取消开机启动 | `sudo systemctl disable chat2me-motion.service` |
| 持续查看日志 | `sudo journalctl -u chat2me-motion.service -f` |

升级时先下载新版本，再执行：

```bash
sudo systemctl stop chat2me-motion.service
sudo ./chat2me-motion service install --no-start
sudo systemctl start chat2me-motion.service
```

升级保留已有配置。直接使用 `service install` 会安装、启用并重启服务；先编辑配置时使用 `--no-start`。

卸载会停止服务，并删除 `/opt/chat2me/motion` 下的程序、配置和 socket：

```bash
sudo /opt/chat2me/motion/chat2me-motion service uninstall
```

## 常见问题

| 现象 | 处理方式 |
| --- | --- |
| socket 不存在 | 查看服务状态和日志，确认配置中的 `socket_path` 与调用路径一致 |
| socket 连接报权限错误 | 宿主机调试使用 `sudo curl`；容器调用时检查挂载目录和访问权限 |
| `/health` 返回 `503` | 检查电机反馈话题、ROS domain 和反馈是否持续更新 |
| `ready=true` 但机器人没有执行 | 确认 RL 控制器处于 disturb 策略的 X-MANUAL 模式，且使用相同的指令话题 |
| 动作请求返回 `422` | 查看响应中的 `error`，检查动作名、关节维数、限位和超时时间 |
| 启动提示缺少 `libyaml-cpp.so.0.7` | Ubuntu 22.04 安装运行库：`sudo apt install libyaml-cpp0.7` |

第三方许可证见 [NOTICE](third_party/NOTICE)。
