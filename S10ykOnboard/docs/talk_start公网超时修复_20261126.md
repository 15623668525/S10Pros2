# 双向对讲"无法开始：Failed to fetch"二次定位与修复（2026-11-26，v3.5.6）

## 一、现象

- 用户手机 Web（HTTPS，公网 IP:端口）点"开始对讲"，状态栏报 **"无法开始：Failed to fetch"**。
- **一键喊话正常**。
- 上一轮（v3.5.5）按"HTTPS accept 队列被 chunked 长流占满"修复（backlog 5→128、watchdog 无消费者收麦、mic_front 去 scipy）后，**问题仍在**。

## 二、二次定位：上一轮根因判断有误

关键证据来自后端日志与前端报错文案的交叉比对：

| 证据 | 说明 |
|---|---|
| 用户报错文案是 **"无法开始："** 前缀 | 对应前端 `talkStart()` 的 catch，即 **`/api/talk/start` 这个 POST 本身 fetch 失败**，不是下行流中断（后者前缀是"下行中断："） |
| 后端日志 `15:28:23 TALK start dev=plughw:1,0 16000Hz frame=1280B` | **请求确实到达了后端，麦克风采集也启动了** |
| 随后 `15:28:44 TALK stop (no-listener)` | 采集持续了 21 秒，前端却从未消费 —— 前端在 start 阶段就失败了 |
| 一键喊话的 `shout/start` 每次都秒级成功 | 同样的 HTTPS、同样的 fetch POST，短响应能正常返回 |

**结论**：请求到达了后端、后端也执行了，但**响应没能传回手机**。差异在于响应时间：

- `talk_start()` 原实现（1360–1397 行）在启动采集后有一段**最多等 3 秒首帧**的循环：
  ```python
  t0 = time.time()
  while time.time() - t0 < 3.0:
      if TALK["bytes"] > 0 or not TALK["active"]: break
      time.sleep(0.05)
  ```
  本机实测 `talk/start` 耗时 **1.53 秒**（优化后），优化前可达 3 秒。
- `/api/shout/start` 是毫秒级返回。
- 本机 `127.0.0.1:18443` 自测两者都成功（**没有公网中转超时**），所以之前只在本机压测没能复现。
- 公网 HTTPS 链路对**长时间不出数据的响应**有超时/断连行为：后端在等首帧，链路判定请求挂死而断开，浏览器 fetch 抛 `TypeError: Failed to fetch`。

上一轮修的 backlog/watchdog/scipy 冷启动是**真实存在的隐患**（已保留），但不是这次"无法开始"的根因。

## 三、本次修复（v3.5.6）

**后端 `backend/app.py`，前端零改动：**

`talk_start()` 不再等待首帧，启动采集线程后**立即返回会话状态**：

```python
# 不等待首帧：立即返回，避免公网 HTTPS 长响应被中断
st = talk_state()
st["ok"] = bool(st["active"])
return st
```

- 首帧不再阻塞 HTTP 响应，改由 `/api/talk/pcm` 下行流自然承载（前端 `talkDownlink()` 用 ReadableStream 边收边播，本来就能处理"先连接、后出数据"）。
- 麦克风真打不开的兜底：前端已有 status 轮询 `if (!s.talk.active) talkStop()`；后端 `_talk_reader` 重试耗尽后置 `active=False`，前端会自动收尾并显示错误。
- docstring 更新，说明公网断连事故与兜底机制。

## 四、验证

| 项 | 修复前 | 修复后 |
|---|---:|---:|
| `/api/talk/start` 响应（本机 HTTPS） | 1.53～3.0 s | **0.004～0.08 s** |
| 返回体 | `ok:true, bytes:0`（首帧未到） | `ok:true`，立即 |
| 下行 PCM | 正常 | 正常（3 秒拉到 151 KB） |
| 后端采集 | 正常 | 正常（日志 TALK start） |

3 轮 start 计时：`0.080s / 0.004s / 0.006s`；下行 PCM `151040 B`；`talk/stop` 正常收尾。

**待用户手机公网复测**（本机无法复现公网链路超时，这是唯一剩余验证项）。

## 五、备份与同步

- 备份：`/opt/s10yk/backend/app.py.bak.talk_start_wait_20261126`
- 部署 MD5：`3a8edcfee669dfbb72c30b2e8dffcfd1`
- 服务已重启（PID 989037），启动日志含 `backlog=128`，确认新代码加载。
