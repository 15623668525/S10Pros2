# HTTPS 18443 全站超时：TLS 握手阻塞 accept 线程（2026-11-26，v3.5.9）

## 一、现象

16:10 用户报告 18443 访问异常。诊断：HTTPS 自测 `https=000`（8s 超时），**明文 18090 正常**（200 / 1.5ms）；`ss` 显示 18443 **accept 队列堆积 111 个连接**（backlog=128 上限附近）。

日志回溯：15:56 用户测试结束后的 14 分钟里，队列以约 8 个/分钟的速度堆积——**公网端口被扫描器/异常客户端持续打**。

## 二、根因（真正的一直以来的病灶）

旧实现（v3.5.5 引入的 128 队列之前就存在）：

```python
https = BackendHTTPServer((host, TLS_PORT), Handler)
https.socket = ctx.wrap_socket(https.socket, server_side=True)   # ← 病灶
```

`SSLContext.wrap_socket(listen_socket)` 的语义是：**accept() 返回连接时同步完成 TLS 握手**。任何一个客户端建立 TCP 后不发 ClientHello（公网扫描器的标准行为），accept 线程就阻塞在握手读上——所有后续连接只能堆进 accept 队列，全站超时。

- v3.5.5 的 `backlog=128` 只是延后症状（能堆更多），机制未变。
- 之前两次"HTTPS 卡死/failed to fetch"诊断都归因于 chunked 流占用，**真正的机制是握手阻塞 accept**。

## 三、修复（v3.5.9）

1. `BackendHTTPServer.get_request()`：只 accept 裸 TCP；`tls_ctx` 存在时 `wrap_socket(..., do_handshake_on_connect=False)` 并 `settimeout(15)`——包装本身不阻塞。
2. `Handler.setup()`：在 handler 线程里补 `do_handshake()`。握手失败/超时只废本线程（15s），accept 永不阻塞。
   - 踩坑：`self.connection` 要到 `super().setup()`（StreamRequestHandler.setup）才赋值，握手代码必须用 `self.request`（16:15 首版部署即报 `AttributeError`，已修正）。
3. `handle_error()`：`ssl.SSLError` 静默——坏握手是公网常态噪音，不打爆 journal。

## 四、验证（攻击模拟）

| 场景 | 结果 |
|---|---|
| 基线 HTTPS status | 200 / 0.01s |
| **3 个哑连接（连上不握手）挂住期间**，正常请求 | **200 / 0.01s**（旧代码必挂死）✅ |
| 哑连接 15s 超时释放后再测 | 200 ✅ |
| 10 个哑连接 + 10 并发正常请求 | **10/10 = 200** ✅ |
| 攻击期间对讲全链路 | talk/start ok=true、front=true；2s 后下行轮询 3×6400B（精确 200ms/批）✅ |

## 五、部署

- MD5：`de8c50ef1151b728e97c20cbd65ed1a7`；备份 `app.py.bak.tls_handler_20261126`。
- 服务 PID 随部署更新；启动日志 `handshake-in-handler` 标识生效。
