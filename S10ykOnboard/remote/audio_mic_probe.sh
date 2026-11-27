#!/bin/bash
# 麦克风占用判定：谁能打开 USB 麦（card1），以及容器侧是否有进程在用
echo "=== 启动时间 ==="
uptime -s; uptime | sed 's/^/  /'
echo
echo "=== 试占麦（6ch/16k，1s）==="
arecord -D hw:1,0 -f S16_LE -r 16000 -c 6 -d 1 /tmp/mic6.wav 2>&1 | sed 's/^/  /'
echo "  rc=$? size=$(stat -c%s /tmp/mic6.wav 2>/dev/null || echo none)"
echo
echo "=== 试占麦（plughw:1,0，1ch）==="
arecord -D plughw:1,0 -f S16_LE -r 16000 -c 1 -d 1 /tmp/mic1.wav 2>&1 | sed 's/^/  /'
echo "  rc=$? size=$(stat -c%s /tmp/mic1.wav 2>/dev/null || echo none)"
echo
echo "=== 容器（containerd）==="
which ctr crictl 2>/dev/null
ctr -n k8s.io containers ls 2>&1 | head -8
crictl ps 2>&1 | head -8
echo
echo "=== 所有含 node/robot/ai 的进程 ==="
ps -eo pid,args | grep -Ei 'node|robot|nodectl|dds' | grep -v grep | sed 's/^/  /'
echo
echo "=== dmesg 末尾 ==="
dmesg 2>/dev/null | tail -12 | sed 's/^/  /'
