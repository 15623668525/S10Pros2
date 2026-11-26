#!/bin/bash
# =====================================================================
# s10yk_ctl.sh — S10YK 应用一键控制（204 云控版，v3.5.10 配套）
#
# 控制范围（前后端/脚本/模型全栈）：
#   s10yk-backend   控制台后端 + 前端页面（18090 明文 / 18443 TLS，前端由后端直接 serve）
#   s10yk-persond   人员检测服务（YOLOv8n 模型推理、抓拍、预警；数据落 /opt/s10yk/data/person）
#   s10yk-say       板端音频守护 sayd（205 上，经 SSH 免密通道联动；MIC 前端 mic_front.py
#                   与 talk 的 arecord 由 backend 按会话自行拉起，无需单独控制）
#   s10yk-link-probe 链路探活（仅 status 展示，由 timer 触发，不在 start/stop 范围）
#
# 用法：
#   s10yk_ctl.sh start            启动全部（backend + persond + sayd）
#   s10yk_ctl.sh stop             停止全部
#   s10yk_ctl.sh restart          重启全部
#   s10yk_ctl.sh status           全栈状态（单元/端口/健康检查/数据新鲜度）
#   s10yk_ctl.sh log [N]          尾部日志（默认各 20 行）
#   s10yk_ctl.sh sayd start|stop|restart   单独控制板端 sayd
#
# sayd 的 sudo 密码：/opt/s10yk/keys/say.sudo（chmod 600，单行明文）；缺失时
# 尝试 sudo -n 免密，失败仅告警不阻塞（sayd 常驻，一般无需启停）。
# =====================================================================
set -u

SAY_HOST="${S10YK_SAY_HOST:-user@<ROBOT_IP>}"
SSH_KEY="${S10YK_SSH_KEY:-/opt/s10yk/keys/id_ed25519}"
SAY_SUDO_FILE="/opt/s10yk/keys/say.sudo"
PERSON_DATA="/opt/s10yk/data/person"
UNITS_LOCAL="s10yk-backend s10yk-persond"

say_ssh() {
    ssh -i "$SSH_KEY" -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=no "$SAY_HOST" "$@"
}

# 远端 sudo systemctl（优先 sudo -S 注入密码，退化 sudo -n）
say_systemctl() {
    local act="$1"
    if [ -r "$SAY_SUDO_FILE" ]; then
        head -1 "$SAY_SUDO_FILE" | say_ssh "sudo -S -p '' systemctl $act s10yk-say" 2>/dev/null && return 0
    fi
    say_ssh "sudo -n systemctl $act s10yk-say" 2>/dev/null && return 0
    return 1
}

do_all() {
    local act="$1"
    echo "=== [1/3] 本机 systemctl $act $UNITS_LOCAL ==="
    systemctl $act $UNITS_LOCAL
    if [ "$act" = "stop" ]; then
        systemctl reset-failed $UNITS_LOCAL 2>/dev/null || true   # 清掉 Restart=always 残留的 failed 态
    fi
    echo "=== [2/3] 板端 sayd（$SAY_HOST）==="
    if say_systemctl "$act"; then
        echo "sayd $act OK"
    else
        echo "WARN: sayd $act 失败（sudo 需要密码且 $SAY_SUDO_FILE 缺失/错误）。sayd 常驻一般无需启停；"
        echo "      如需单独控制：s10yk_ctl.sh sayd $act"
    fi
    echo "=== [3/3] 完成，建议执行 s10yk_ctl.sh status 核验 ==="
}

status() {
    local u st
    echo "===== systemd 单元（本机 204）====="
    for u in $UNITS_LOCAL s10yk-link-probe; do
        st=$(systemctl is-active "$u" 2>/dev/null)
        printf "  %-22s %s\n" "$u" "$st"
    done
    echo "===== 端口（本机 204）====="
    for p in 18090 18443; do
        if ss -tln 2>/dev/null | grep -q ":$p "; then
            echo "  :$p  LISTENING"
        else
            echo "  :$p  CLOSED"
        fi
    done
    echo "===== 健康检查 ====="
    local code
    code=$(curl -s -m 3 -o /dev/null -w '%{http_code}' http://127.0.0.1:18090/api/auth 2>/dev/null)
    echo "  backend /api/auth -> HTTP $code (期望 200)"
    if [ -f "$PERSON_DATA/detector_status.json" ]; then
        local age=$(( $(date +%s) - $(stat -c %Y "$PERSON_DATA/detector_status.json") ))
        echo "  persond detector_status.json 年龄 ${age}s（<120s 为检测活跃）"
    else
        echo "  persond 无 detector_status.json（未运行或未产出）"
    fi
    echo "===== 板端 sayd（$SAY_HOST）====="
    local st2 rc2
    st2=$(say_ssh "systemctl is-active s10yk-say" 2>/dev/null); rc2=$?
    if [ "$rc2" -ne 0 ] && [ -z "$st2" ]; then st2="UNREACHABLE"; fi
    echo "  s10yk-say: $st2"
    if say_ssh "ss -tln 2>/dev/null | grep -q ':18081 '" 2>/dev/null; then
        echo "  :18081  LISTENING（板端）"
    else
        echo "  :18081  CLOSED（板端）"
    fi
}

show_log() {
    local n="${1:-20}"
    echo "===== s10yk-backend 最近 $n 行 ====="
    journalctl -u s10yk-backend -n "$n" --no-pager 2>/dev/null | tail -n "$n"
    echo "===== s10yk-persond 最近 $n 行 ====="
    journalctl -u s10yk-persond -n "$n" --no-pager 2>/dev/null | tail -n "$n"
}

case "${1:-}" in
    start)   do_all start ;;
    stop)    do_all stop ;;
    restart) do_all stop; sleep 2; do_all start ;;
    status)  status ;;
    log)     shift; show_log "${1:-20}" ;;
    sayd)
        case "${2:-}" in
            start|stop|restart)
                if say_systemctl "$2"; then echo "sayd $2 OK"; else echo "sayd $2 失败（检查 $SAY_SUDO_FILE）"; exit 1; fi ;;
            *) echo "用法: s10yk_ctl.sh sayd {start|stop|restart}"; exit 2 ;;
        esac ;;
    *)
        grep -E '^#   s10yk_ctl' "$0" | sed 's/^# *//'
        exit 2 ;;
esac
