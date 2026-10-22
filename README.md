# Chat2Me Motion

Chat2Me Motion 是专为人形/双足机器人设计的头部与动作执行服务，通过 ROS 2 话题实时控制机器人头部/颈部关节，并提供轻量级 Unix Domain Socket (UDS) HTTP 接口与 `chat2me-core` 协同联动。

## 特性

- **严格头部转动约束**：颈部偏航角（Yaw）硬性限制在 **±70°**（±1.2217 rad）以内，防止机械过位或拉扯线缆。
- **高阶 S 曲线平滑轨迹**：内置五次多项式（Minimum Jerk）插值算法，保证运动平稳、启停无冲击。
- **ROS 2 原生集成**：原生订阅 `motor/q2w_upper_motor_joint_state`，下发 `motor/tt_upper_motor_pt_control_data` 控制指令。
- **单二进制与服务管理**：编译为嵌入 RPATH 的单独立二进制文件，无需预先 `source setup.bash`，内置 `service install` 与 `service uninstall`。
- **开箱即用 UDS HTTP 接口**：默认监听 `/opt/chat2me/motion/motion.sock`，支持 `nod`（点头）、`shake`（摇头）、`look_around`（环顾四周）、`think`（侧头思考）、`look_at`（声源定位转向）。

## 快速使用

### 启动测试

```bash
chat2me-motion run --config /opt/chat2me/motion/config.yml
```

### 服务安装

将自身注册为开机自启 systemd 系统服务：

```bash
sudo chat2me-motion service install
```

### 服务卸载

彻底停止服务并清理程序、配置文件与套接字文件：

```bash
sudo chat2me-motion service uninstall
```

### 查看状态

```bash
chat2me-motion service status
```

## 动作接口协议

通过 Unix Domain Socket 发送 HTTP POST 请求：

```json
{
  "id": "act-123456",
  "name": "nod",
  "params": {},
  "preempt": true,
  "timeout_ms": 5000
}
```

声源定位转向（附带角度）：

```json
{
  "id": "act-123457",
  "name": "look_at",
  "direction_deg": 45.0,
  "preempt": true
}
```

响应（202 Accepted）：

```json
{
  "ok": true,
  "id": "act-123456",
  "action": "nod"
}
```
