# S10Pros2 — S10 Pro 机器狗控制台（服务器版 + 单机版）

一个仓库，两个子项目，同一份代码基线（v3.5.12，2026-11-27）按部署形态分目录维护：

| 子目录 | 版本 | 应用主体位置 | 外部依赖 |
|---|---|---|---|
| [`S10ykServer/`](S10ykServer/) | 服务器版 | 云服务器（`/opt/s10yk/`，18090/18443） | 经 zenoh-bridge/frp 链路远程控制本体，依赖公网链路 |
| [`S10ykOnboard/`](S10ykOnboard/) | 单机版 | 机器狗本体业务板 103（`/opt/s10yk/`） | **零外部依赖**，DDS rclpy/drdds 直连 NOS 106，开机自启 |

两个子目录各有一份 README（架构、目录结构、部署要点、资源占用摘要）。

## 当前运行形态

单机版已全栈落盘业务板并接管（2026-11-27 P5 切换完成）：103 板 `s10yk-backend` + `s10yk-persond` + `s10yk-say` 开机自启；204 侧组件停用待命（回滚：103 `disable --now` → 204 `enable --now`）。

## 硬件资源占用（实测 2026-11-27，详见各子项目 docs/硬件资源占用调研_20261127.md）

| 形态 | 内存占用 | CPU 占用 |
|---|---|---|
| 服务器版·云侧（48核/128GB） | ≈140 MB | <1.5 核（检测满载估算） |
| 服务器版·板侧（轻量件） | ≈135 MB | <0.2 核 |
| 单机版·板上全栈（8×A55/15.7GB） | ≈1.05 GB | ≈1.3–1.5 核（persond 占 1.3 核持续） |

## 两版公共文件（任一侧修复必须同步另一侧）

- `backend/app.py`、`backend/person_detect.py`
- `frontend/index.html`、`frontend/shout-processor.js`
- `remote/mic_front.py`、`remote/sayd.py`、`remote/s10yk-say.service`
- `remote/ntp/`、`docs/缺陷记录.md`

同步方法：修改一侧后复制到另一侧对应路径，两子目录各自验证后一并提交。

## 文档索引

- 版本记录：`S10ykServer/docs/版本记录.md`（服务器版主线）／`S10ykOnboard/docs/版本记录.md`（落盘主线）
- 缺陷记录：两子目录 `docs/缺陷记录.md`（保持一致）
- 整体落盘方案与实施记录：`S10ykOnboard/docs/`
- 服务器移植与网络通讯方案：`S10ykServer/docs/`
- 硬件资源占用调研：两子目录 `docs/硬件资源占用调研_20261127.md`

## 安全说明

- 本仓库为脱敏仓库：密码、hostkey、公网 IP、UWB ID 等一律以占位符（`<SERVER_PW>`/`<SERVER_IP>`/`<HOSTKEY>` 等）入库；原始档案在本地 `S10ykApp/`（不入库）。
- 每次同步改动后运行 `tools/sanitize.py`（两子目录各内置，幂等）并做敏感串终扫，确认 0 命中再提交。
