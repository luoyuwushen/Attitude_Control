# H 连续控制记录：开发契约

状态：project-v0.2.9 / host-v0.6.8 已完成离线集成验证，实板连续记录待验证。旧版 FPGA v0.2.8 / host-v0.6.7 不支持本文的新命令。

FPGA缓存、提交探针、UART导出与整帧仲裁，以及主机分流、GUI和后台JSON/CSV保存已接通。完整性、采样连续性、测量有效性及磁盘保存状态分开判断；验证见[本版记录](project-v0.2.9_validation.json)。

2026-09-28 的 20:18 会话中，积分没有触顶，H 仍持续周期运动；50 Hz 快照不能重建中间的 1 kHz 更新。本功能保存同一控制周期的输入、旧积分与提交后的命令，供区分估速与反馈响应。它不测量电机实际电压、电流或真实独立角度，不能仅凭新增记录判定模拟干扰或机械成因。

## 采集契约

H 获准进入平衡时开启新代次，清空记录计数，保存完整 Q4 的 D/U 和捕获摆臂参考。环形缓存最多保存最近 4096 个已经提交的 H 控制周期，名义跨度 4.096 秒。停止或故障后冻结；新 H 会替换旧缓存。记录器只观察信号，不能产生、延迟或改变控制命令。

ADC 接收沿缓存 meanQ4、encoder、sample_counter 及质量；与该样本的 estimator valid 对齐。控制器 stage5 真正提交时保存该命令所用的旧积分和状态，随后等顶层方向映射寄存器更新，保存匹配的门控后软件命令。停止、故障或标定取消的流水线不能被误记为成功提交；序号间断保留，不能插值或冒充连续样本。

每行 32 字节，小端，Python 格式 `<IiHhhhhihhHBBBB`：

| 字节 | 字段 | 单位/意义 |
| --- | --- | --- |
| 0–3 | sample_counter | 对应 ADC 窗口序号，无符号32位 |
| 4–7 | encoder_count | 估计器实际接收的有符号原计数 |
| 8–9 | adc_mean_q4 | 对应 ADC 窗口均值 |
| 10–17 | theta、omega、arm、arm_speed | 四个有符号 Q10 状态 |
| 18–21 | integral_q24 | 本次请求所用的旧积分，有符号32位 |
| 22–23 | command_permille | 本次提交的限幅后请求 |
| 24–25 | motor_command_permille | 方向映射后、门控后的软件命令；非实际力矩 |
| 26–27 | control_age_ms | 本次 H 年龄，沿用最大512 |
| 28 | h_flags | active/update_allowed/freeze/at_limit，沿用低4位 |
| 29 | adc_quality | 对应输入窗口的质量 |
| 30 | sensor_flags | 同周期的 bad/OTR/valid/ready/liveOTR 低5位 |
| 31 | fault | 本次记录时故障汇总 |

## 只读导出

命令 `T` 请求冻结缓存，仅在无运动的 IDLE/FAULT 且已有冻结记录时接受，不清故障、不标定、不启动。运动命令和硬停止继续优先；导出中重新请求运动时，只发完当前已经开始的包后中止，尚未开始的END不发送。若END本身已在发送，则发完该END，其对应的旧历史数据仍可完整。主机须将缺尾的记录标为不完整。正常遥测继续报告当前状态，不把历史记录送入实时曲线或新鲜度判断。

固件ID为 `0x00020009`。普通v2帧追加 TLV33（type在206、length在207，length=5）：flags:uint8、row_count:uint16、capture_id:uint16。flags bit0=正在采集、bit1=已冻结、bit2=正在导出、bit3=环形覆盖，其余为0。行数为0～4096。新普通帧总215字节，CRC位于213/214，既有字段位置不变。未开始H时无冻结记录，T不产生导出。

正常遥测约50Hz，停止后的导出期间约10Hz，为历史整包预留带宽；每个历史包先读取最多7行，再申请UART发送，禁止与普通帧交错字节。215字节普通帧与240字节历史包在115200/8N1下分别需要约18.7ms与20.8ms。仲裁优先待发送心跳，并在下一心跳前不足一个最大历史包的时间内停止授权；导出结束后恢复50Hz。数据与控制节拍仍为1kHz。

导出包使用独立 version=3；正常遥测仍为 version=2。每包先 `AA 55 03 length`，再 `<BBHHHBB>`：kind、schema=1、capture_id、index、total_rows、row_count、flags；末尾 CRC16/CCITT-FALSE，覆盖包头和有效载荷，低字节先发。总长度为16+payload，最大240字节。

- kind=1：元信息，index=0、row_count=0，24字节 payload，格式 `<IHHhIIIBB>`：firmware_id、D_Q4、U_Q4、capture_arm_Q10、period_cycles、freeze_device_ms、total_committed、freeze_reason、status。reason 1=停止，2=故障，3=离开H；status低位保留环形覆盖标志，其余位须为0。
- kind=2：数据，index为首行在本次导出中的顺序，row_count为1～7，payload恰为32×row_count。total_rows须与元信息一致。只接受严格连续的index，不能静默跳过、拼接不同代次或接收越界行。
- kind=3：结束，index=total_rows、row_count=0，payload为所有原始32字节行按序计算的CRC16（2字节）。只有元信息、完整行序列和结束校验均通过，才可标为完整。

每次导出以新元信息开始；重试必须新开导出记录，不向已失败记录续写。断连、缺行、坏包或缺尾必须保留失败信息。固件重启可能复用capture_id，因此ID不作跨连接全局标识。

## 验证与发布条件

独立测试需覆盖同周期绑定、积分旧值、流水线取消、坏窗valid、环形覆盖顺序、同步RAM读取、CRC/缺帧/重试、后台落盘，以及开启记录与关闭记录时控制输出逐拍相同。必须通过完整布局布线与时序检查和主机集成检查后才能发布；现场操作以本地用户手册为准。
