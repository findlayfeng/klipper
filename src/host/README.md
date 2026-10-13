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
| `klipper_host_init()` | 初始化库（桥模式下同时打开串口，打不开就返回 -1） |
| `klipper_host_set_uuid(uuid[6])` | 设置本库在串口复用里的**身份**；**必须在 `init` 前**调用，`init` 后再调返回 -1，不调 = 全 `0` |
| `klipper_host_input(data, len)` | 线程安全，向 klipper 输入二进制协议字节（**要么整块收下，要么返回 0**） |
| `klipper_host_output(buf, size)` | 线程安全，读取 klipper 输出的帧数据（不等待） |
| `klipper_host_output_wait(buf, size, ms)` | 同上，但在没有数据时最多等待 `ms` 毫秒；专用读取线程应当用它而不是轮询 |
| `klipper_host_output_frame(buf, size, ms)` | 同上的等待语义，但**每次返回恰好一帧**（由帧自己的长度字节界定）；需要 `CONFIG_HOST_FRAME_API=y` 才编进库 |
| `klipper_host_input_unit(unit, len)` | 一次交**一整单元**（2 字节复用头 + 帧）。需要 `CONFIG_HOST_FRAME_API=y` **且**开启桥模式（`CONFIG_BRIDGE`）；**半个单元被拒**（返回 0），因为 mux 靠头的两字节边界切单元 |
| `klipper_host_output_unit(buf, size, ms)` | 同上的等待语义，但**每次返回恰好一整单元**（头 + 帧，头里就是来源设备）；同样需要帧接口开关 + 桥模式 |
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

### 帧接口开关（`CONFIG_HOST_FRAME_API`）

`menuconfig` 里的 "Hand whole protocol frames to the caller"，默认关闭。

关闭时（默认）调用方拿到的是**字节流**：`klipper_host_output_wait()` 可能返回半帧，也可能一次返回好几帧，边界要自己找。打开后会多出 `klipper_host_output_frame()`：klipper 本来就是把每条消息先成帧再放进缓冲区的，所以库自己知道边界——它按帧的**长度字节**取整帧（**不能靠找 SYNC**：压缩过的 identify 数据里就可能带 `0x7e`）。

两种模式都不丢消息，取舍只在"谁来负责找帧边界"：库里找（省掉调用方的重组逻辑，代价是缓冲区至少 `MESSAGE_MAX` 字节）还是调用方找。

`make frames` 是这一开关的自检（见 `example/frames.c`，需要先用 `CONFIG_HOST_FRAME_API=y` 构建库）。

### 帧接口与单元接口

两个开关共用：

- **帧接口（`CONFIG_HOST_FRAME_API`）**：`klipper_host_output_frame()`，按帧的长度字节取整帧。
- **单元接口（`CONFIG_HOST_FRAME_API` **且** `CONFIG_BRIDGE`）**：`klipper_host_input_unit()` /
  `klipper_host_output_unit()`，一次交/收**一整单元**（2 字节复用头 + 帧）。头在单元里，所以
  主机不必自己找边界、也不必另外传 id。只有桥会向上游发单元，所以没有桥就不编这两个符号。

**字节流永远是必实现的部分**：`klipper_host_input()` / `klipper_host_output_wait()` 始终存在，
**分配/扫描**那段对话（KXID、admin 信道）就走它。两套 API 是同一对缓冲区的两种视图，
调用方选定一种就一直用它。

### 桥模式（`CONFIG_HOST_BRIDGE_SERIAL`）

`menuconfig` 里的 "Bridge mode: add a serial port to the console"，默认关闭。

桥模式给控制台**再挂端口**，由固件在各端口之间转发（固件内部多路复用，串口只是其中一种端口）。它是**面向所有固件**的特性，host 只是第一个落点，因此分层为「传输无关的公共符号 + 架构给的端口」：

| 选项 | 位置 | 含义 | 默认 |
|------|------|------|------|
| `MUX` | `src/Kconfig`（公共、隐藏） | **设备半**：能被寻址、应答扫描/admin、采纳 nodeid、碰撞规则（逐字对照 CAN 的 `can_id_conflict`） | — |
| `MUX_SERIAL` | `src/Kconfig`（公共） | 设备半跑在串口/字节流上 | — |
| `BRIDGE` | `src/Kconfig`（公共、隐藏） | **桥半**：端口表 + 路由 + 扇出（`select MUX`） | — |
| `BRIDGE_PORT_SERIAL` | `src/Kconfig`（公共） | 桥有串口下游端口 | — |
| `BRIDGE_PORT_SERIAL_BAUD` | `src/Kconfig`（公共） | 该端口速率（写法同 `SERIAL_BAUD`） | `250000` |
| `HOST_BRIDGE_SERIAL` | `src/host/Kconfig` | host 上的可见开关，`select BRIDGE`/`BRIDGE_PORT_SERIAL`/`MUX_SERIAL` | `n` |
| `HOST_BRIDGE_SERIAL_PATH` | `src/host/Kconfig` | 打开的 tty 路径 | `/dev/ttyACM0` |

架构只回答"桥的是哪个端口"：host 没有固定端口表，所以是**路径字符串**（默认
`/dev/ttyACM0`）；微控制器将来照自己给控制台选 UART 的做法给一份 `choice` 备选串口
（`AVR_SERIAL_UART*` / `STM32_SERIAL_USART*` 那种）。`MUX_CAN` / `BRIDGE_PORT_CAN`
（CAN 也是端口，见 `src/Kconfig` 注释）随 CAN 后端一起落地。

本步实现到**打开并持有 fd** 为止：

- 打开时机是 `klipper_host_init()`，一个库一次；端口**非阻塞**打开
  （`O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC`），将来与其它工作一起 poll，做法同
  `src/linux/console.c` 对它的 tty。
- **打不开就让 `klipper_host_init()` 返回 -1**：桥模式是构建期显式要的，静默降级会让
  宿主以为桥在而实际没有。原因写 stderr，例如
  `bridge serial: /dev/ttyACM0 failed: (2)No such file or directory`。
- 只有 `CONFIG_BRIDGE_PORT_SERIAL=y` 才编 `bridge.c`；默认（`=n`）根本不参与构建，
  宿主 API 的行为一个字节都不变。
- **身份由主机指派**：MUX 按身份寻址，而库是主机加载的文件、自己没有可派生的东西（按路径派生还可能与其他设备撞车），所以身份由主机在 `init` 前用 `klipper_host_set_uuid()` 给出，不调就是全 `0`。已实现：存储 + `init` 时报一行 `mux uuid: <12 位十六进制>`；消费它的 MUX 设备半（应答扫描/admin、采纳 nodeid、碰撞规则）是下一步。
- 串口与控制台之间的**接线**（把串口收到的字节喂进控制台、把控制台输出写进串口）
  尚未实现，属下一步。

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

`make frames` 跑 `frames.c`（需要 `CONFIG_HOST_FRAME_API=y` 构建的库）：验证每次调用返回的确实**恰好一帧**——长度字节自洽、以 SYNC 结尾，并且 48 字节的响应与 5 字节的 ack 各自独立成帧。

`make test` 跑 `burst.c`：连续灌入 2048 条命令（输入 12KB、输出约 16KB，
都超过初始缓冲区），期间不读任何输出，然后验证**每条命令都有响应**。
去掉上面两条缓冲策略中的任何一条，这个测试都会失败。

## 实现说明

- `host.c`    – 主循环入口；通过 `sched_main()` 运行调度器，外部关闭
  用 SIGUSR1 + longjmp 中断（内部急停仍由 sched_main 处理，可
  `config_reset` 恢复）。`klipper_host_init()` 里按 Kconfig 打开桥模式的串口。
- `console.c` – 线程安全的收发缓冲区（mutex + condvar 保护）；实现
  `console_sendf()` / `console_receive_buffer()`；输入线程写入后唤醒
  console task 解析分派。输入侧全有或全无，输出侧可扩容，见上表。
- `timer.c`   – 基于 `clock_gettime(CLOCK_MONOTONIC)` 的时钟与
  irq 模拟；`timer_dispatch_many()` 等来自 `generic/timer_irq.c`。
- `bridge.c`  – host 的**串口下游端口后端**（`CONFIG_BRIDGE_PORT_SERIAL`）：打开、置 raw 8N1
  （速率经 `termios2`/`BOTHER`，支持任意速率）、持有 fd；默认不参与构建。设备身份由**主机
  指派**（不走路径派生），随 step 2 的 `MUX` 一起落地。
- `klipper_host.h` / `internal.h` – 公开 API / 内部接口。
