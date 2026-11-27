#!/bin/bash
# 音频硬件占用体检（205/103 声卡板）——建议以 root 运行：sudo bash audio_occupancy_check.sh
echo "=== /dev/snd 节点 ==="
ls -l /dev/snd/ | sed 's/^/  /'
echo
echo "=== fuser 占用 ==="
fuser -v /dev/snd/* 2>&1 | head -20
echo
echo "=== 打开 /dev/snd 的进程（扫 /proc） ==="
found=0
for pid in $(ls /proc | grep -E '^[0-9]+$'); do
  fds=$(ls -l /proc/$pid/fd 2>/dev/null | grep -o '/dev/snd/[^ ]*' | sort -u | tr '\n' ' ')
  if [ -n "$fds" ]; then
    echo "  pid=$pid comm=$(cat /proc/$pid/comm 2>/dev/null) -> $fds"
    found=1
  fi
done
[ $found -eq 0 ] && echo "  （无进程持有 /dev/snd 节点）"
echo
echo "=== 各 PCM 状态 ==="
for f in /proc/asound/card*/pcm*/sub*/status; do printf '  %s: ' "$f"; head -1 "$f"; done
echo
echo "=== pipewire/pulse 端口 ==="
echo "-- sources --"; pactl list short sources 2>&1 | head -10
echo "-- sinks   --"; pactl list short sinks 2>&1 | head -10
echo
echo "=== card1 采集 hw_params ==="
cat /proc/asound/card1/pcm0c/sub0/hw_params 2>&1 | sed 's/^/  /'
echo
echo "=== 音频相关进程 ==="
ps -eo pid,user,args | grep -Ei 'audio|pulse|pipewire|aplay|arecord' | grep -v grep | sed 's/^/  /'
