# S10ykServer — S10 Pro 机器狗云控服务器版

应用主体运行在云服务器（204，`/opt/s10yk/`），经 zenoh-bridge/frp 链路远程控制机器狗本体（业务板 103 / NOS 106）。**单机版（应用全盘落盘狗本体）见姊妹仓库 `S10ykOnboard`。**

## 架构

```
手机/浏览器 ──HTTPS 18443──> 204: s10yk-backend（前后端一体）
                                ├── s10yk-persond（人员检测, RTSP 经 wlan 拉 103:8554）
                                ├── s10yk-link-probe（链路探活 timer）
                                └── SSH 免密通道 ──> 103: sayd(s10yk-say, 18081) / mic_front / arecord
204 <──zenoh-bridge-dds(103)──> DDS（/STEER /MOTION_INFO /AUDIO ...）
```

## 目录结构

- `backend/`：app.py（前后端一体 + 限流 + TLS）、person_detect.py（YOLO 人员检测）、s10yk_ctl.sh（一键控制脚本）
- `frontend/`：index.html、shout-processor.js（AudioWorklet 对讲上行）
- `remote/`：部署到板端的组件（sayd.py、mic_front.py、s10yk-say.service）与 204 侧探活（link-probe）、时钟同步（ntp）
- `assets/`：语音资产
- `ros2/`：DDS 诊断工具（diag/）与 UWB 信标驱动（ubeacon/）
- `docs/`：方案、实施记录、版本记录、缺陷记录
- `tools/sanitize.py`：仓库脱敏工具（幂等，同步原件后运行）

## 部署要点（204）

1. `/opt/s10yk/{backend,frontend,certs}`；服务 `s10yk-backend`（18090 明文 + 18443 TLS）+ `s10yk-persond`
2. TLS 证书 SAN 需含服务器公网 IP
3. 一键控制：`/opt/s10yk/s10yk_ctl.sh {start|stop|restart|status|log|sayd ...}`
4. 板端 sayd sudo 控制走 `/opt/s10yk/keys/say.sudo`（root-only，不进仓库）
5. 安全：`S10YK_TOKEN` 鉴权 + v3.5.10 来源 IP 频率限制（120 请求/2s，私网豁免）

## 与单机版（S10ykOnboard）的公共文件

以下文件两仓库**必须保持一致**，任何一侧修复后同步另一侧并各自提交：

- `backend/app.py`、`backend/person_detect.py`
- `frontend/index.html`、`frontend/shout-processor.js`
- `remote/mic_front.py`、`remote/sayd.py`、`remote/s10yk-say.service`
- `remote/ntp/`、`docs/缺陷记录.md`

## 硬件资源占用（实测 2026-11-27）

| 侧 | 组件 | 内存 | CPU |
|---|---|---|---|
| 云侧（204, 48核/128GB） | backend + persond + zenoh-bridge-dds | ≈140 MB（待机） | <1.5 核（检测满载估算） |
| 板侧（103，轻量件） | sayd + mic_front + zenoh-bridge-dds | ≈135 MB | <0.2 核 |

服务器版对板子的负担极轻（≈135 MB / <0.2 核），云侧占比可忽略。详细数据与调优建议（persond 降档等）见 [`docs/硬件资源占用调研_20261127.md`](docs/硬件资源占用调研_20261127.md)。

## 版本

详见 [`docs/版本记录.md`](docs/版本记录.md)。当前基线：v3.5.12（2026-11-27）。
