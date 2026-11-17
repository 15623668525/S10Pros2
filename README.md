# S10ykApp — 山猫 S10 Pro 机器狗远程控制台

基于 Web 的四足机器狗（云深处 山猫 S10 Pro）远程控制系统：手机/PC 打开浏览器即可完成**运动遥控、四路视频监控、语音播报、一键喊话（实时）、双向对讲**。后端刻意做成 **Python 纯标准库 + rclpy**（目标板无外网、不能 pip），前端为**零构建单文件页面**，整体极易部署与维护。

> 当前版本：**v3.0.6**（2026-11-17）· 详细演进见 [`docs/版本记录.md`](docs/版本记录.md)

## 功能总览

| 模块 | 能力 |
|---|---|
| 运动控制 | 站立/趴下/软急停、步态切换（基础/平地/楼梯）、虚拟摇杆连续控速、ARM 使能滑动开关、一键急停（归零+断使能+软急停）、速度限幅与 600ms 断流保护 |
| 视频监控 | 4 路 RTSP 摄像头（前/后/左/右）转 MJPEG 分页显示，视频开关统一滑动开关样式 |
| 语音播报 | 自定义音频（16k/mono WAV/MP3）经板端 sayd 实时推流播放，支持停止与循环 |
| 一键喊话 | 手机麦克风采音 → 后端 → 机器人扬声器**边说边播**（100ms 分片、HTTP keep-alive、丢旧保延迟）；原生采样率采集 + worklet 内重采样 16k 兼顾兼容性与带宽 |
| 双向对讲 | 机器人麦克风（6ch 阵列）→ 浏览器实时收听 + 手机→机器人实时喊话；**MIC 被抢占自动重连与计数**（前端常显） |
| 遥测回显 | 运动状态、步态、速度、机身高度、里程、电量，500ms 轮询 |
| 诊断工具 | 声卡占用体检、麦克风电平表/三阶段自检、链路探活、NTP 时钟核查、UWB 定位话题验证等 |

## 总体架构

```
┌──────────── 手机/PC 浏览器（Chrome，HTTPS）────────────┐
│        frontend/index.html（单文件：摇杆/视频/语音）      │
└──────────────────────┬─────────────────────────────────┘
                       │ HTTP/JSON + Cookie 鉴权 + PCM 流（keep-alive）
┌──────────────────────▼─────────────────────────────────┐
│  云服务器  backend/app.py                                 │
│  · 18090 明文 / 18443 TLS（自签 SAN 证书）单实例双监听     │
│  · S10Bridge：ROS2/zenoh 运动控制与遥测                   │
│  · 摄像头代理：RTSP → MJPEG                               │
│  · SHOUT：上行喊话会话（看门狗/代次保护/互斥）             │
│  · TALK：下行对讲（SSH+arecord 采集、chunked 推流、        │
│    MIC 抢占检测与自动重开）                                │
└──────┬──────────────────────┬───────────────────────────┘
       │ zenoh (7447)         │ ssh / TCP 18081
┌──────▼──────────┐   ┌──────▼────────────────────────┐
│ 机器狗 NOS 主板  │   │ 机器狗业务板  remote/sayd.py     │
│ ROS2 Jazzy      │   │ START/F_PCM/F_STOP 帧协议       │
│ /MOTION_* /GAIT │   │ → aplay -D plughw:0,0 (ES8323) │
└─────────────────┘   └────────────────────────────────┘
```

## 仓库结构

```
S10ykApp/
├── backend/                 # 控制台后端（云服务器）
│   ├── app.py               # 全部后端逻辑（HTTP/TLS、ROS2 桥、喊话、对讲、代理）
│   └── s10yk.service        # systemd 单元
├── frontend/                # 前端（零构建单文件）
│   ├── index.html           # 页面 + 全部 JS
│   └── shout-processor.js   # AudioWorklet：多声道下混 + 线性插值重采样 16k
├── remote/                  # 机器狗板端组件
│   ├── sayd.py              # 音频播放守护（TCP 18081，帧协议，250ms 缓冲可调）
│   ├── s10yk-say.service    # 含 ExecStartPre 自动打开 ES8323 输出开关（D-038）
│   ├── audio_occupancy_check.sh / audio_mic_probe.sh   # 声卡占用/麦克风体检
│   ├── link-probe.sh + *.service/.timer                # 链路探活
│   ├── ntp/                 # 时钟同步调研与核查脚本
│   └── *_probe / *_test.py  # 运动/步态/缓冲/抖动等诊断脚本
├── ros2/
│   ├── diag/                # ROS2 话题诊断（步态、里程、遥控器、转向标定等）
│   └── ubeacon/             # UWB 定位（标签驱动见 docs/UWB 报告）
├── assets/                  # 自定义语音素材 + 清单
└── docs/                    # 全套过程文档（已脱敏）
    ├── 需求文档.md            # REQ 台账（至 REQ-041）
    ├── 技术方案.md            # 总体技术方案（含 §12 语音/喊话/对讲）
    ├── 版本记录.md            # v1.0 → v3.0.6 全部版本
    ├── 缺陷记录.md            # D-001 → D-038（含根因与修复）
    ├── 操作记录.md            # OPS-001 → OPS-072 可复现命令级操作史
    ├── 服务器移植与网络通讯方案.md
    ├── 一键喊话_技术方案调研.md / 一键喊话_单向实施方案.md
    ├── 实现方式审计_话题与硬件.md
    └── evidence/             # 实测证据（日志、摄像头抓帧）
```

## 快速部署（摘要）

1. **业务板（机器人）**：部署 `remote/sayd.py` 至 `/opt/s10yk/`，安装 `remote/s10yk-say.service` 并 `systemctl enable --now s10yk-say`（单元会自动打开 ES8323 的 Speaker/Headphone 开关——厂商栈弱化后无人打开会导致全静音，见 D-038）。
2. **云服务器**：
   - `backend/app.py` → `/opt/s10yk/backend/`，`frontend/` → `/opt/s10yk/frontend/`；
   - 生成自签证书（**必须含 SAN**，Chrome 只认 SAN）：`openssl req -x509 -newkey rsa:2048 -days 825 -nodes -keyout key.pem -out cert.pem -subj "/CN=s10" -addext "subjectAltName=IP:<服务器IP>,DNS:localhost"`；
   - systemd 注入 `S10YK_TOKEN`（鉴权 Cookie 密钥）、`S10YK_TLS_CERT/KEY/PORT` 环境变量；
   - 配置到业务板的 SSH 免密（对讲的下行采集走 `ssh + arecord`）。
3. **机器人侧网络**：zenoh 7447 放行、NTP 时钟同步、摄像头 RTSP 地址按现场配置。
4. 访问 `https://<服务器IP>:18443`（麦克风功能需 HTTPS + Chrome）。

> 完整部署步骤、防火墙、证书、踩坑记录：见 [`docs/服务器移植与网络通讯方案.md`](docs/服务器移植与网络通讯方案.md) 与 [`docs/操作记录.md`](docs/操作记录.md)。

## 安全说明

- 仓库内文档均为**脱敏副本**：真实密码、服务器/机器人公网 IP、SSH 主机密钥已替换为 `<SERVER_PW>`、`<SERVER_IP>`、`<ROBOT_IP>`、`<HOSTKEY>` 等占位符，部署时按现场实际替换即可；原稿不随仓库分发。
- 生产环境建议：改用强随机 `S10YK_TOKEN`、TLS 证书替换自签、7447 限制源地址并启用 zenoh 鉴权。

## 版本快照

- **v3.0.x**：双向对讲（含 MIC 抢占计数）、喊话实时化、上行静音/拥塞/SPK 静音三连修复（D-036/037/038）
- **v2.7.x**：aplay 缓冲 500ms→250ms（端到端延迟降至 ~0.5s）、TLS 双监听
- **v2.x**：服务器移植、四路视频、UWB、语音三页签改版
- **v1.x**：板端直跑版本（运动控制 + 语音播报）

完整历史：[`docs/版本记录.md`](docs/版本记录.md)
