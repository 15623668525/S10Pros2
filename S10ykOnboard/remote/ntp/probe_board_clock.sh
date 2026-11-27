#!/bin/bash
# 机器狗两块板卡时钟机制现场调研脚本（103 业务板 / 106 主控板通用，root 或 sudo 运行）
# 产物：/home/user/clock_report_<host>.txt
set -u
OUT=/home/user/clock_report_$(hostname)_$(date +%m%d_%H%M).txt
exec > >(tee -a "$OUT") 2>&1

echo "########## $(hostname) $(date) ##########"

echo "== 1. 当前时间与时钟状态 =="
date '+%F %T %z (epoch %s)'
timedatectl 2>/dev/null || true
ls -l /dev/rtc* 2>/dev/null
hwclock -r 2>/dev/null && echo "(hwclock 读取成功)" || echo "(hwclock 不可用/无 RTC 电池)"

echo "== 2. NTP 客户端现状 =="
systemctl is-active systemd-timesyncd chronyd chrony 2>/dev/null
systemctl status systemd-timesyncd --no-pager 2>/dev/null | sed -n '1,10p'
systemctl status chrony --no-pager 2>/dev/null | sed -n '1,8p'
echo "--- timesyncd.conf ---"
grep -v -E '^\s*#|^\s*$' /etc/systemd/timesyncd.conf 2>/dev/null || echo "(全注释=未配置)"
echo "--- chrony 配置与运行参数 ---"
grep -v -E '^\s*#|^\s*$' /etc/chrony/chrony.conf /etc/chrony/conf.d/*.conf 2>/dev/null
ps -eo pid,cmd | grep -E 'chronyd|timesyncd' | grep -v grep
chronyc tracking 2>/dev/null | head -8
chronyc sources 2>/dev/null

echo "== 3. 厂商对时机制证据（关键）=="
echo "--- dmesg 中 test_time / time_diff 最近 20 条（看频率与方向）---"
dmesg -T 2>/dev/null | grep -i -E 'test_time|time_diff' | tail -20
echo "--- 内核日志里最近 24h 的对时消息条数（推算写入频率）---"
journalctl -k --since '-24h' --no-pager 2>/dev/null | grep -ci 'test_time\|time_diff'
journalctl -k --since '-24h' --no-pager 2>/dev/null | grep -i 'test_time\|time_diff' | tail -6

echo "== 4. 谁在写系统时钟（嫌疑人排查）=="
echo "--- 名字带 time/mcu 的运行中服务 ---"
systemctl list-units --type=service --state=running --no-pager | grep -iE 'time|mcu|sync|drsec|s10' || true
echo "--- 文件内容含 test_time / time_diff 的可执行与配置 ---"
grep -rIl 'test_time\|time_diff' /usr/local /opt /etc /usr/bin 2>/dev/null | head -15
echo "--- crontab ---"
crontab -l 2>/dev/null; grep -rIl 'date -s\|hwclock\|clock_set' /etc/cron* 2>/dev/null | head
echo "--- /dev/mcu 类设备 ---"
ls -l /dev/mcu* /dev/ttyS* /dev/ttyTHS* 2>/dev/null | head

echo "== 5. 与服务器 204 的 NTP 连通性与偏差实测 =="
if command -v python3 >/dev/null; then
  python3 /home/user/ntp_offset_check.py <SERVER_IP>
  # 106 若到 204 不通，再试 103
  [ "$(hostname)" != "host" ] || python3 /home/user/ntp_offset_check.py 10.21.33.103
fi

echo "== 6. （仅 103）zenoh 桥时间戳配置复核 =="
grep -n -A3 'timestamping' /opt/zenoh/bridge-103.json5 2>/dev/null || echo "(找不到 bridge-103.json5)"
ps -eo pid,cmd | grep zenoh-bridge | grep -v grep

echo "== 7. 到 204 的 UDP 123 原始可达性 =="
timeout 2 bash -c 'echo > /dev/udp/<SERVER_IP>/123' && echo "udp123 发送成功(不代表有应答,应答看上面 offset)" || echo "udp123 发送失败"

echo "########## 报告已存 $OUT ##########"
