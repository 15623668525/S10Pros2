#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仓库内脱敏工具：对仓库全部文件套用统一脱敏规则（幂等，可重复运行）。
用途：从工作区原件同步文件进仓库后运行一次；日常直接编辑仓库文件则无需运行。
"""
import os, re, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKIP_DIRS = {".git", "__pycache__"}

RULES = [
    (re.compile(r"SHA256:[A-Za-z0-9+/=]{10,}"), "SHA256:<HOSTKEY>"),
    (re.compile(r"SHA256:[A-Za-z0-9+/=]{4,}…?[A-Za-z0-9+/=]{0,8}"), "SHA256:<HOSTKEY>"),  # 截断指纹（含省略号）
    (re.compile(r"\$?<SERVER_PW>"), "<SERVER_PW>"),
    (re.compile(r"<PW_B64>"), "<PW_B64>"),                     # 205 sudo 密码的 base64
    (re.compile(r"-pw\s+\"'\""), "-pw '<PW>'"),            # bash 转义形态 -pw '<PW>'
    (re.compile(r"-pw\s+'(?!<PW>)"), "-pw '<PW>'"),        # 形态 -pw '<PW>'xxx；负向前瞻防对已脱敏输出再匹配（幂等）
    (re.compile(r"58\.250\.117\.204"), "<SERVER_IP>"),
    (re.compile(r"58\.250\.117\<ROBOT_IP>"), "<ROBOT_IP>"),
    (re.compile(r"58\.250\.117\.106"), "<BOARD_IP>"),
    (re.compile(r"58\.250\.117\.0/24"), "<SITE_NET>"),          # 网段
    (re.compile(r"58\.250\.117\.x"), "<SITE_IP>"),             # 通配写法
    (re.compile(r"58\.250\.117\.\d{1,3}"), "<SITE_IP>"),       # 其余同段主机
    (re.compile(r"(?<![\d.])\<ROBOT_IP>\b"), "<ROBOT_IP>"),          # 省略网段写法 "<ROBOT_IP>"
    # 2026-11-21 补：frps 公网入口 + 临时/旁路网段 + 硬件标识
    (re.compile(r"115\.190\.226\.22"), "<FRP_IP>"),            # frps 公网入口（旧）
    (re.compile(r"119\.91\.199\.137"), "<FRP_IP>"),            # frps 公网入口（现用，11-19 切换）
    (re.compile(r"10\.21\.41\.9(?:/24)?"), "<REMOTE_IP>"),     # 遥控器旁路 IP
    (re.compile(r"192\.168\.1\.133(?:/24)?"), "<SITE_GW_IP>"), # 服务器 ens16f1 备用口
    (re.compile(r"\b0204051207db\b"), "<UWB_GW_ID>"),          # UWB 网关 ID
    (re.compile(r"\b0204080619e5\b"), "<UWB_TAG_ID>"),         # UWB 标签 ID
    (re.compile(r"\b37313\b"), "<UWB_ANCHOR_ID>"),             # UWB 基站 ID
]

def main():
    changed = 0
    for dirpath, dirnames, files in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in files:
            p = os.path.join(dirpath, fn)
            try:
                t = open(p, encoding="utf-8").read()
            except (UnicodeDecodeError, OSError):
                continue
            t2 = t
            for rx, rep in RULES:
                t2 = rx.sub(rep, t2)
            if t2 != t:
                open(p, "w", encoding="utf-8", newline="").write(t2)
                print("sanitized:", os.path.relpath(p, ROOT))
                changed += 1
    print("changed:", changed)

if __name__ == "__main__":
    main()
