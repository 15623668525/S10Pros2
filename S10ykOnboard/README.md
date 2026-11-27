# S10ykOnboard — S10 Pro 机器狗单机版（应用全盘落盘本体）

应用主体运行在机器狗本体业务板（103，`/opt/s10yk/`），DDS 直通 NOS 板（106），**运行时不依赖任何外部云服务器**。**云控服务器版（应用主体在 204）见姊妹仓库 `S10ykServer`。**

## 架构

```
手机/浏览器（现场 WiFi 10.21.33.103:18443 或 wlan 58.250.x.x:18443）
        ──HTTPS──> 103: s10yk-backend（前后端一体, S10YK_SAY_HOST=local 全本机化）
                    ├── s10yk-persond（人员检测, RTSP 127.0.0.1:8554, venv aarch64）
                    ├── mic_front.py / arecord（对讲采集, 本机直起）
                    └── 127.0.0.1:18081 sayd（s10yk-say）
        103 <──rclpy/drdds 直连──> 106 NOS（ROS2 Jazzy, domain 0）
```

## 目录结构

- `backend/`：app.py（含 SAY_HOST=local 本机化分支 + 限流 + TLS）、person_detect.py
- `frontend/`：index.html、shout-processor.js
- `remote/`：板上组件（sayd.py、mic_front.py、s10yk-say.service）、板上 systemd 单元（s10yk-backend-onboard.service.template、s10yk-persond-onboard.service）、诊断脚本、ntp
- `assets/`：语音资产（含厂商 audio_node_assets 持久化方案，见部署要点）
- `ros2/`：DDS 诊断工具与 UWB 驱动
- `docs/`：落盘方案与实施记录、版本记录、缺陷记录
- `tools/sanitize.py`：仓库脱敏工具（幂等）

## 部署要点（103）

1. `/opt/s10yk/{backend,frontend,models,certs,assets,venv}`；服务 `s10yk-backend` + `s10yk-persond`（均 enabled 开机自启）
2. TLS 证书 SAN：`IP:10.21.33.103, IP:<WLAN_IP>, DNS:localhost`
3. **厂商目录 /opt/robot 重启会被还原**：audio_node_assets 持久化到 `/opt/s10yk/assets/audio_node_assets/`，unit 用 `S10YK_AUDIO_YAML` 指向
4. drdds（aarch64）从 NOS 106 提取：site-packages/drdds + rosidl typesupport .so + /usr/local/lib/libdrdds.so
5. 安全：鉴权默认 OFF（如需口令，unit 补 `Environment=S10YK_TOKEN=<token>`）+ v3.5.10 来源 IP 频率限制兜底
6. 回滚到服务器版：205 `systemctl disable --now s10yk-backend s10yk-persond` → 204 侧 `enable --now`

## 与服务器版（S10ykServer）的公共文件

以下文件两仓库**必须保持一致**，任何一侧修复后同步另一侧并各自提交：

- `backend/app.py`、`backend/person_detect.py`
- `frontend/index.html`、`frontend/shout-processor.js`
- `remote/mic_front.py`、`remote/sayd.py`、`remote/s10yk-say.service`
- `remote/ntp/`、`docs/缺陷记录.md`

## 硬件资源占用（实测 2026-11-27）

板上全栈（8× Cortex-A55 / 15.7 GB / 无 swap）：

| 组件 | 内存（cgroup） | CPU |
|---|---|---|
| s10yk-persond | 995 MB | ≈1.3 核持续（1280 模型 + 4 线程 + 2fps） |
| s10yk-backend | 54 MB | 9–15%（随推流/对讲波动） |
| s10yk-say | 6 MB | ≈0 |
| **S10YK 合计** | **≈1.05 GB** | **≈1.3–1.5 核** |

板整体内存 available 11 GB（余量充足）；检测开启时段 load 峰值 12.4/8 核，persond 是唯一持续大户。作为对照，服务器版在本板只占 ≈135 MB / <0.2 核。降档选项（模型 640 / 线程 2 / fps 1）与详细数据见 [`docs/硬件资源占用调研_20261127.md`](docs/硬件资源占用调研_20261127.md)。

## 版本

详见 [`docs/版本记录.md`](docs/版本记录.md)。当前基线：v3.5.12（2026-11-27，P5 切换完成）。
