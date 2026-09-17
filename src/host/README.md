# Klipper host library (src/host)

Klipper 固件以**静态库**的形式提供给其他程序使用。与 `src/simulator`
不同，本目录**不包含 main()**，也不使用标准输入输出；而是定义了一对
**线程安全的输入/输出函数**，由宿主程序（其他进程/线程）调用。

## 构建

```sh
cd third_party/klipper
make olddefconfig && make
# 产物: out/libklipper_host.a
```

Kconfig 中 `MACH_HOST` 已启用最小配置：只包含核心调度器、命令解析和
控制台 I/O，**不实现任何外设**（GPIO/SPI/I2C/ADC/PWM/传感器等均未
启用）。

## 公开 API（见 klipper_host.h）

| 函数 | 说明 |
|------|------|
| `klipper_host_init()` | 初始化库 |
| `klipper_host_input(data, len)` | 线程安全，向 klipper 输入二进制协议字节（**要么整块收下，要么返回 0**） |
| `klipper_host_output(buf, size)` | 线程安全，读取 klipper 输出的帧数据（不等待） |
| `klipper_host_output_wait(buf, size, ms)` | 同上，但在没有数据时最多等待 `ms` 毫秒；专用读取线程应当用它而不是轮询 |
| `klipper_host_run()` | 阻塞运行 klipper 主循环（在独立线程中调用） |
| `klipper_host_step()` | 非阻塞单步执行（用于集成外部事件循环） |
| `klipper_host_shutdown()` | 线程安全，请求关闭，使 run() 返回 |
| `klipper_host_is_shutdown()` | 查询 klipper 是否处于急停状态 |
| `klipper_host_get_clock()` | 获取当前 klipper 时钟计数 |

## 使用方式

```c
#include "klipper_host.h"

// 在后台线程中运行 klipper
pthread_create(&kt, NULL, (void*(*)(void*))klipper_host_run, NULL);

// 任何线程都可以安全地调用输入/输出
ssize_t n = klipper_host_input(frame_bytes, len);  // 发送命令（n == len 才算成功）
klipper_host_output(buf, sizeof(buf));             // 读取响应（不等待）

// 或者用一个专职线程阻塞等待：
size_t got = klipper_host_output_wait(buf, sizeof(buf), 100);
```

**两个缓冲区都是有限的（各 4096 字节起步），并且两条路径都不会弄丢半条消息**：

| 情况 | 行为 |
|------|------|
| 输入缓冲区放不下整块数据 | 最多等 250ms 让 console task 消化已有数据；仍然放不下就**一个字节也不收**，返回 0，调用方重发这一帧即可（绝不写半个消息块——那会让解析器失步） |
| 输出缓冲区不够 | 先搬移压缩，再**动态扩容**（上限 256KB）。真到上限才等待 250ms，最后才丢弃并在 stderr 报告 |
| 没有人读输出的后果 | klipper 会在输出缓冲区满后停下来等（就像真实 MCU 等主机读取那样），所以**读取不要和发送挤在同一个线程里**：`make test` 里的 burst 测试正是用「先灌 2048 条命令再统一读」来验证这一点 |

输出扩容是刻意的：真实固件把响应写进（内核会缓冲的）tty，而这个库没有内核帮忙，若不扩容，单线程调用方在「一边发命令一边没读响应」时会互相卡住。

数据格式为 klipper 二进制消息协议（见 `src/command.h` 中的
`MESSAGE_*` 定义）：`LEN SEQ MSGID PARAMS... CRC16 SYNC(0x7e)`，
帧只有尾部 SYNC，CRC16-CCITT 覆盖 `[0 .. LEN-3]`。

## 运行示例

```sh
cd src/host/example
make run      # 或 make && LD_LIBRARY_PATH=../../../out ./klipper_host_example
make test     # 自检：缓冲区边界（burst.c）
```

`klipper_host_example` 演示：启动 klipper 线程、接收 "starting" 帧、
发送 `get_clock` 命令并读取响应、优雅关闭。

`make test` 跑 `burst.c`：连续灌入 2048 条命令（输入 12KB、输出约 16KB，
都超过初始缓冲区），期间不读任何输出，然后验证**每条命令都有响应**。
去掉上面两条缓冲策略中的任何一条，这个测试都会失败。

## 实现说明

- `host.c`    – 主循环入口；通过 `sched_main()` 运行调度器，外部关闭
  用 SIGUSR1 + longjmp 中断（内部急停仍由 sched_main 处理，可
  `config_reset` 恢复）。
- `console.c` – 线程安全的收发缓冲区（mutex + condvar 保护）；实现
  `console_sendf()` / `console_receive_buffer()`；输入线程写入后唤醒
  console task 解析分派。输入侧全有或全无，输出侧可扩容，见上表。
- `timer.c`   – 基于 `clock_gettime(CLOCK_MONOTONIC)` 的时钟与
  irq 模拟；`timer_dispatch_many()` 等来自 `generic/timer_irq.c`。
- `klipper_host.h` / `internal.h` – 公开 API / 内部接口。
