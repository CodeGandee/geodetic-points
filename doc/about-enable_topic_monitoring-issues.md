## 新增验证结论与定位更新（基于近期确认）

用户已确认以下两条命令在相同环境下能够产生实时输出，但日志文件中仍没有对应记录：

- `ros2 topic echo /tf_static 2>/dev/null | grep -B10 -A20 -E "frame_id:.*earth"`
- `ros2 topic echo /vio/pose_earth`

据此可以较为确定：问题主要发生在当前 `ExecuteProcess` 内部“管道/缓冲/重定向”的记录链路上，而非话题本身或 `ros2 topic echo` 的可用性。

### 更精确的问题点

1) 非 TTY 环境导致 stdout 缓冲阻塞
   - 当 `ros2 topic echo` 的输出被管道消费（非交互 TTY）时，Python/底层 stdout 可能采用全缓冲或较大缓冲，导致下游 `while read line` 长时间收不到数据，从而“看起来没有记录”。
   - 这与“直接在终端运行 echo 命令能看到输出”并不矛盾：终端是行缓冲，管道是块缓冲。

2) grep 上下文（-B/-A）放大缓冲/延迟效应
   - `-B10 -A20` 需要维护上下文窗口，结合上游块缓冲会进一步推迟内容下发至 `while read`。
   - 在消息较稀疏或打印格式跨多行时，容易表现为“迟迟不落地”。

3) 并发追加到同一日志文件的竞争与不可预期时序
   - 三个后台循环同时 `>> "$LOG_FILE"` 追加，虽然 O_APPEND 能保证偏移原子更新，但在高并发与不同缓冲策略下，仍可能出现写入时序不可预期（延迟/交错），放大“看起来没有记录”的现象。

4) 记录链路使用 `while read line; do echo ... >> "$LOG_FILE"; done` 的逐行 echo
   - 每一行都会触发一次重定向打开/写入/关闭（由 shell 处理），在高频/多路情况下开销较大；若上游不稳定、或 log_dir 在网络/容器挂载上，也可能出现间歇性丢写/延迟。

5) 头部“Log file: $LOG_FILE”漏写问题仍成立
   - 不影响正文缺失的根因，但会干扰人工排障。

### 更针对性的改进建议（仍不直接改代码，仅提供方案）

- 明确强制行缓冲
  - 在三路订阅前添加：`stdbuf -oL -eL`，示例：
    - `/calibration/transform_earth_odom`: `stdbuf -oL -eL ros2 topic echo /calibration/transform_earth_odom | stdbuf -oL -eL awk '{ print }' ...`
    - `/tf_static`: `stdbuf -oL -eL ros2 topic echo /tf_static | stdbuf -oL -eL grep -E ... | stdbuf -oL -eL awk ...`
    - `/vio/pose_earth`: 同理加 `stdbuf -oL -eL`
  - 目的：把上游进程从块缓冲改为行缓冲，确保数据及时进入下游。

- 收敛/替代 grep 上下文
  - 优先移除 `-B/-A`，改为单行匹配（必要时在 `awk` 里持有简易状态机搜集上下行）。
  - 或直接以 `awk` 进行字段过滤与前缀标注（更可控）。

- 简化记录链路，避免逐行 echo 重定向
  - 改为将前缀/时间戳在流中处理，然后统一 `tee -a "$LOG_FILE" >/dev/null`：
    - 例如：`... | stdbuf -oL awk '{ print strftime("[%Y-%m-%d_%H:%M:%S]"), "[tag]", $0 }' | tee -a "$LOG_FILE" >/dev/null`
  - 优点：由 `tee` 维护单次打开，减少多进程并发对同一文件的频繁 open/close。

- 若需保留逐行时间戳（毫秒）
  - `awk` 可用 `strftime("%Y-%m-%d_%H:%M:%S")`，毫秒可通过 `strftime` + `systime()` 与外部 `date +%3N` 组合（复杂度偏高）；或接受秒级时间戳，换取稳定性。

- 为每个话题暂存到独立日志，再合并
  - 先写入：`$LOG_FILE.tf_static`, `$LOG_FILE.pose_earth`, `$LOG_FILE.transform_earth_odom`，最后以定时任务拼接/标注入总日志；
  - 优点：消除三路并发对同一文件的竞争，定位更直观。

- 其他小建议
  - 头部改为 Here-Doc 一次性写入，并确保“Log file: $LOG_FILE”使用 `tee -a`；
  - 在子壳开头检查 `LOG_FILE` 是否非空：`[ -n "$LOG_FILE" ] || exit 1`；
  - 对 `monitor_status_report` 的“最后时间戳”解析增加空值兜底提示。

综上：在已确认 echo 命令本身有输出的前提下，最可能的瓶颈是“管道场景下的块缓冲 + grep 上下文延迟 + 多路并发写同一文件”的组合效应。优先尝试对三路链路加 `stdbuf -oL -eL` 并移除 `-B/-A`，通常即可恢复日志实时落盘；随后再考虑以 `tee -a` 取代逐行 echo 重定向、或拆分独立日志以提升稳定性与可观测性。
