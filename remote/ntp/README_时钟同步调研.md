# 机器狗时钟与服务器统一调研（2026-11-16，两块板卡）

> **状态：2026-11-16 18:25 已实施并验证通过（三端统一，偏差 <0.5 s）。**
>
> 结论摘要：真因不是"厂商锁死时钟"，而是 Ubuntu 的 `chronyd-starter.sh` 把厂商加固系统误判为容器，
> 自动给 chronyd 加 `-x`（只测不调）。`/etc/default/chrony` 的 `SYNC_IN_CONTAINER="yes"` 即官方开关，
> 改完重启即 step -51.9 s；106 由厂商 PTP 跟随、无需改动；服务器侧 chrony 早已就绪（`allow <SITE_NET>/24`）。
> 详见下方 §2/§4/§6 与 `操作记录.md` OPS-34、`缺陷记录.md` D-024。

## 1. 问题

机器狗两块板卡（103 业务板 = <ROBOT_IP>，106 主控板 = 10.21.33.106）的时钟比服务器
（<SERVER_IP>）**快 48.8 ~ 52.6 秒且持续漂移**，导致：

- 服务器 zenoh 桥对机器人数据打的时间戳全部"超前"，超出 zenoh 500 ms 容差 → 桥日志
  **75.4 万条/天 ERROR 刷屏**（数据本身"替换时间戳后转发"，不丢，但日志被淹没、排序/去重隐患）；
- 跨机日志时间无法对齐，事后排查（如 D-028）只能靠间接推断；
- 任何"按时间戳判新旧"的逻辑（现在及将来）都不可靠。

## 2. 三端现状（实测）

| 端 | 时钟来源 | 状态 |
|---|---|---|
| 服务器 204 | chronyd → 阿里/腾讯 NTP（stratum 2/3） | ✅ 正常，且 **NTP 服务端已就绪**：`allow <SITE_NET>/24`、`local stratum 8`（上游全断也能授时）、ufw 已放行 `123/udp from <ROBOT_IP>` |
| 103 业务板 | **厂商 MCU 对时机制锁死**；chronyd 已装但带 `-x`（只运行不控钟） | ❌ 快 48.8→52.6 s，`date -s` 数秒内被拉回 |
| 106 主控板 | systemd-timesyncd 默认（`NTP=` 被注释，OPS-24 试验后已回退）；同样被厂商机制锁到 MCU 时间 | ❌ 同上 |

**偏移漂移实测**（服务器 zenoh 桥日志，11-15 00:03 至 11-16 17:00）：
48.80 s → 52.61 s，**+3.8 s / 41 h ≈ 26 ppm ≈ 2.2 s/天**。这说明 MCU 的时钟在自由漂移
（没有任何外部时间源），且没有任何机制会把它拉回真值——漂移会无限增长。

## 3. 核心约束：厂商跨板对时机制（D-024 红线的来源）

已知（2026-11-15 实测）：
- 板卡内核日志出现 `test_time / send time_diff`，系统时钟被**锁死在与 MCU 一致**；
- 人工 `date -s` 校正后**数秒内被拉回**；
- 103 的 chronyd 带 `-x`，不参与控时（当时为避免打架特意配置）。

**三个未知数（决定走哪条路，现场必查）**：
1. **谁在写钟**：厂商哪个进程/服务调用 `settimeofday`/`clock_settime`；
2. **多久写一次**：`test_time` 消息间隔（秒级？分钟级？还是仅开机一次）；
3. **方向**：MCU → 板（MCU 是时间主）还是板 → MCU（板把时间发给 MCU）。

## 4. 路线对比

| 路线 | 做法 | 优点 | 缺点/风险 | 判定条件 |
|---|---|---|---|---|
| **A. 维持现状+加固**（当前架构） | 恢复并固化 103 bridge `timestamping.enabled=false`（见 §6，疑似被还原）；应用层继续用服务器钟 | 零风险，已验证可行 | 时钟本身仍漂；日志对不齐；治标 | 兜底，随时可用 |
| **B. NTP 接管**（推荐主攻） | 停用/绕开厂商写钟机制 → 103 chrony 去 `-x` + `server <SERVER_IP> iburst` + `rtcsync`；106 timesyncd `NTP=<SERVER_IP>`（或 103 中转：103 加 `allow 10.21.33.0/24`） | 治本，服务器端已就绪，**两块板都无需装任何包**（chrony/timesyncd 都已存在） | 与厂商机制冲突期会来回跳；OTA 可能恢复厂商服务；MCU 自身时间依旧漂（厂商内部用） | ①厂商写钟频率低（间隔>1h，如仅开机）→ 可共存，直接上；②频率高 → 必须先停用机制 |
| **C. 改写 MCU 时间** | 找到 MCU 时间接口（协议/字符设备），由 103 定期把服务器时间写给 MCU | 最治本，不与厂商机制冲突，MCU 和板卡一起准 | 需逆向或厂商配合；写错有风险 | 现场确认 MCU 通信接口与协议后评估 |
| **D. 偏移观测+消费端校正** | 103/106 各跑一个 SNTP 观测脚本，把 offset 暴露给应用，消费端按 offset 修正 | 完全不动系统 | 每个消费端都要改；漂移无限增长终会失控 | 兜底 |

**推荐**：B 为主攻（先按 §5 采证，再按写钟频率决定"共存"还是"停用"），A 立即恢复作为保底，
C 作为 B 被厂商机制挡死时的备选，D 不建议长期使用。

## 5. 现场调研清单（机器人恢复后执行，脚本已备好）

```bash
# 1) 两块板各跑一次（脚本自动采：时间状态 / NTP 现状 / test_time 频率 /
#    写钟嫌疑人 / 与 204 的 SNTP 偏差实测 / 106→204 与 106→103 的可达性）
pscp 到板卡后: sudo bash probe_board_clock.sh      # 产物 /home/user/clock_report_*.txt

# 2) 重点看四样东西
#   ① dmesg/journal 里 test_time 的间隔 → 决定 B 走"共存"还是"停用"
#   ② grep 到的写钟嫌疑进程/脚本名
#   ③ ntp_offset_check.py 输出的实际 offset（当前理论值 +52 s 左右）
#   ④ 106 → <SERVER_IP>:123 UDP 是否可达（不可达则 106 指 103）
```

## 6. 本次调研的新发现（与 D-024 记录冲突，需复核）

D-024 记录"103 端 bridge 关 timestamping 后 ERROR 归零（11-15）"，但 **11-15 至今服务器桥
仍以 ~20 条/s 持续刷 timestamp 拒收 ERROR**（累计 75 万条+），且 offset 在持续增长。
说明 103 端 `bridge-103.json5` 的 `timestamping.enabled=false` **大概率已失效**——
可能被厂商脚本/重启覆盖，或当时改的不是生效中的配置文件。机器人恢复后第一件事：

```bash
grep -n -A3 timestamping /opt/zenoh/bridge-103.json5
systemctl status zenoh-bridge-dds   # 确认 -c 指向哪个文件
```

修复并固化：改回 `false` 后 `chattr +i`（或部署一个校验 timer，配置被改就告警）。

> 关于 NavCmd 下行是否受时钟影响：zenoh 默认 `drop_future_timestamp=false`（本桥日志
> 显示行为是"替换后转发"），下行数据时间戳是"过去 50 s"，zenoh 只拒"未来"时间戳，
> 理论上不丢。跨机 NavCmd 不达的根因仍以 D-026 系列的排查为准，时钟不是当前主嫌疑。

## 7. 实施与回滚预案（路线 B）

```bash
# 103（假设厂商机制可停用/频率低）
# 1) 停厂商对时（具体单元名以 §5 ①② 的调研结果为准）
systemctl disable --now <vendor-time-sync.service>
# 2) chrony 控钟
sed -i 's/^ExecStart=.*chronyd.*/&/' /etc/systemd/system/chrony.service  # 确认去掉 -x
echo 'server <SERVER_IP> iburst prefer' >> /etc/chrony/chrony.conf
echo 'allow 10.21.33.0/24' >> /etc/chrony/chrony.conf   # 顺带给 106 当源
echo 'rtcsync' >> /etc/chrony/chrony.conf
systemctl restart chrony && chronyc tracking   # offset 应从 +52s 开始收敛（先 step 后 slew）

# 106
echo 'NTP=10.21.33.103' >> /etc/systemd/timesyncd.conf   # 或 <SERVER_IP>（视可达性）
systemctl restart systemd-timesyncd && timedatectl

# 回滚：systemctl enable --now <vendor-time-sync>；chrony.conf 恢复 -x；timesyncd 注释回 NTP=
# 验收：chronyc tracking 的 System time 偏差 < 0.1s 且 24h 稳定；服务器桥 timestamp ERROR 归零；
#       dmesg 无 test_time 拉回痕迹；音频/定位/导航/厂商业务 24h 无异常
```

**风险提示**：停用厂商对时后，MCU 的时间仍是漂移的，厂商自身依赖 MCU 时间的功能（日志、
加密校验等）不受影响，但**不能排除厂商有依赖"板卡与 MCU 强一致"的逻辑**——所以路线 B 必须
灰度：先改 103 观察 24 h（重点盯音频、drsec_daemon、定位），再动 106。
