#!/bin/bash
# 机器人侧运控失效排查脚本（只读，不改任何状态）
source /opt/ros/jazzy/setup.bash 2>/dev/null
echo "--- 话题 pub/sub ---"
for t in /MOTION_INFO /STEER /MOTION_STATE /NAV_CMD; do
  echo "$t:"
  timeout 8 ros2 topic info "$t" 2>&1 | grep -E "Publisher|Subscription"
done
echo "--- 实时遥测(一次) ---"
timeout 6 ros2 topic echo /MOTION_INFO --once 2>&1 | head -10
echo "--- 8554 监听者 ---"
echo <PW> | sudo -S ss -tlnp 2>/dev/null | grep -E "8554|30003"
echo "--- 厂商日志目录 ---"
ls -lt /var/opt/robot/log/ 2>/dev/null | head -10
echo "--- 厂商 log/system 最近 ---"
ls -lt /var/opt/robot/log/system 2>/dev/null | head -6
