# 双向对讲 "failed to fetch" 故障定位与修复（2026-11-26 15:09–15:28）

## 现象

用户反馈：手机 Web 控制台点"开始对讲"立刻弹出 `failed to fetch`；一键喊话正常。用户走 **HTTPS** 访问，报错时机是**点击瞬间**（非接通后断流）。

## 定位过程

### 1. 排除 mic_front.py 功能问题
- 明文 18090 的 `/api/status`、`/api/talk/state`、`/api/shout/state` 全部 200；
- 明文 `/api/talk/pcm` 能拉到数据（115KB/2s）；
- 204 本地 curl 对讲全链路正常。

### 2. 锁定 HTTPS 链路故障
- HTTPS 18443 端口在监听，但 `curl -sk https://127.0.0.1:18443/api/status` 返回 `HTTP=000`（12s 超时）；
- `openssl s_client` 握手无响应；
- `ss -tnp` 显示 18443 有连接 **Recv-Q 1701/1733/1797 字节未读**——客户端发了数据服务端没处理，连接卡死。

### 3. 根因
- `ThreadingHTTPServer` 默认 `request_queue_size = 5`。
- `/api/talk/pcm` 是 chunked 长流，手机切后台/网络抖动时客户端不读完就断开，服务端线程在 TLS write 上短时阻塞。
- 几个这样的连接占满 accept 队列（5），**后续所有新连接无法建立** → 手机所有 fetch（含 talk/start）报 "failed to fetch"。
- 喊话走的是 POST 短请求 + sayd 播放链路，且不依赖这个流，故"正常"。

日志佐证（15:08 时段）：`TALK start` 后 `SHOUT start`/`SHOUT watchdog: idle/timeout`/反复切换，多个连接交错，队列被占满；一条 talk 会话 `active` 卡了 279 秒未释放。

## 修复（三处）

| # | 文件 | 改动 | 作用 |
|---|---|---|---|
| 1 | `backend/app.py` | 新增 `BackendHTTPServer(request_queue_size=128, daemon_threads=True)`，HTTP/HTTPS 均用它 | 根因修复：接纳更多并发/卡死连接，不再堵 accept 队列 |
| 2 | `backend/app.py` | watchdog 的"无消费者收麦"改用 `no_stream_since` 独立计时（原按 bytes 变化，采集在跑就永不触发） | 前端 fetch 失败时 streams=0，20s 后自动收麦，不再无限占麦 |
| 3 | `remote/mic_front.py` | 滤波链系数固化为常数，改纯 Python DF-II，去掉运行时 numpy/scipy import | 消除上一版引入的冷启动回归：scipy import 2.84s 把首帧推过 talk_start 的 3s 窗口；现首帧 1.5s（原水平） |

## 验证

| 项目 | 结果 |
|---|---|
| HTTPS 连打 30 次 status | 30/30 = 200 |
| HTTPS 对讲 5 轮 | 全 ok，PCM ~65KB/轮 |
| talk/start 首帧延迟 | 1.53s（bytes=1280 已到），修复前 3.0s/bytes=0 |
| 18443 连接数（压测后） | 0（无堆积） |
| no-listener 自动收麦 | 启动 talk 不连 PCM，23s 后 `active=False why=no-listener`，进程无残留、声卡 closed |
| 最终回归 | 对讲 ok（front=true/denoise=false，PCM 64000B）、喊话 ok |

## 备份与部署

- app.py 改前：`/opt/s10yk/backend/app.py.bak.tls_backlog_20261126`（另有一次分块损坏版本已被 pscp 正确版覆盖）
- mic_front.py 改前（scipy 版）：`~/backup_mic_20261126_v2/mic_front.py.scipy_ver`
- 部署 MD5：app.py `3430dcefcb0172142cb4fb9665becfe3`；mic_front.py `73dfa0ee234f6a1158da605fa4bfabf9`
- 服务重启后日志：`HTTPS listening ... (SHOUT mic enabled, backlog=128)`

## 结论

`failed to fetch` 根因是**后端 HTTPS accept 队列被卡死的 talk/pcm 长流占满**，与 mic_front.py 滤波功能无关；但排查中发现并修复了滤波引入的首帧延迟回归、以及 watchdog 无消费者不收麦的隐患。三项修复均已上线并验证。

## 数据来源

日志 `/opt/s10yk/logs/backend.log`；`ss -tnp`；curl/openssl 自测；`pscp` 部署；脚本改动见 `backend/app.py`、`remote/mic_front.py`。
