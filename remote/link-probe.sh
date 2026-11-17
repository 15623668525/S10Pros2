#!/bin/bash
# 机器人链路探针：每 5s 采一次 205 的 ICMP 与 ARP 状态，落盘供事后做劣化曲线
# 部署：服务器 204  /usr/local/bin/s10yk-link-probe.sh + systemd timer
TS=$(date '+%F %T')
IP=<ROBOT_IP>
OUT=$(ping -c1 -W2 "$IP" 2>/dev/null | sed -n 's/.*time=\([0-9.]*\) ms/\1/p')
ARP=$(ip neigh show "$IP" | awk '{print $NF}')
LOG=/var/log/s10yk-link-probe.log
if [ -n "$OUT" ]; then
  echo "$TS UP rtt=${OUT}ms arp=${ARP:-none}" >> "$LOG"
else
  echo "$TS DOWN arp=${ARP:-none}" >> "$LOG"
fi
# 只保留最近 20000 行（≈27 小时）
if [ "$(wc -l < "$LOG")" -gt 20000 ]; then tail -n 15000 "$LOG" > "$LOG.tmp" && mv "$LOG.tmp" "$LOG"; fi
