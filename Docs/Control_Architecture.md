# 非视觉自主控制与后续视觉接入设计草案

状态：方案草案，尚未实现。针对 STM32F407、双驱动轮差速、前被动万向轮；替代当前依赖电脑连续轮速命令的驾驶方式。MPU6050 实物、接线和安装方向尚未确认，不能因为库中有驱动就宣称传感器已可用。

补充硬件依据（2026-11-12）：用户确认JGB37-520霍尔编码器直流减速电机，输出轴空载约1000rpm；编码器PPR、减速比、左右输出轴CPR仍需独立核实，不可由型号和空载转速反推。赛题第3页固定香橙派、核心板、底盘、轮胎半径和电机，后续分工明确为香橙派视觉/F407控制，不提出更换这些硬件作为实现前提。

实现进展：已落地独立的Car_Vision接收接口及香橙派Python编码器，见[图像通信接口](Vision_Interface.md)。本篇其他自主控制、估计器、状态机和安全架构仍为草案，不能据此宣称已实现。暂不运行图像算法不等于删除图像接入口。

## 1. 控制路线与适用边界

| 路线 | 能解决什么 | 边界 | 建议 |
|---|---|---|---|
| 定时动作或单纯记录左右 PWM | 演示电机动作 | 电压、负载、摩擦一变路线就变；没有位置反馈 | 不作为自主方案 |
| 编码器里程 + MPU6050 航向 + 车端路径执行 | 按距离行驶、定角度转弯、定曲率圆弧、短路径闭环执行；不需要持续蓝牙 | 轮径/轮距误差、打滑、陀螺零偏会累积；不能观测自己相对赛道边界的位置 | 当前重构的基础方案 |
| 上述方案 + 非成像反射式边界传感器 | 增加相对赛道的反馈；断路期间短时切换里程/航向控制 | 白PVC/蓝背景的实际红外反差、45cm路宽下的传感器覆盖、环岛十字的歧义都需验证 | 若希望长期不用图像，优先实验验证此路线 |
| 上述基础方案 + 后续视觉处理器 | 视觉给道路几何和元素观测，本地闭环继续负责执行 | 必须处理延迟、数据过期、丢帧、置信度和失效停车 | 按用户后续计划预留接入 |

校赛资料为45cm白PVC蓝底、无黑色边界，长度55.061m，包含160cm断路、十字和环岛。因此不能拿常见窄黑线灰度循迹程序直接套用，也不能承诺纯里程航向推算稳定跑完整圈。MPU6050的加速度不能提供绝对航向基准；零偏校准和编码器融合不会消除长期漂移。

阶段目标先是可测量地完成直线、转弯、圆弧、组合路径，再以实测误差决定是否能扩大运行距离。未知位置的彩色障碍和数独任务不能仅靠编码器/陀螺仪完成识别；按规则选择屏蔽，或以后增加相应观测源。

## 2. 运行方式

路径存放车内，使用板载按键启动、暂停和停止。蓝牙承担参数设置、路径上传、日志与诊断；比赛模式可以彻底禁用运动遥控命令。

第一版路径使用小型动作表：按相对距离走直线、指定角度转向、指定曲率圆弧、停车等待。动作有物理量目标和反馈完成条件，不按延时假装走到目标。待几何标定与定位可靠后，再扩展按弧长采样的航点路径跟踪。示教可以用于采集几何轨迹，不重放某一遍的PWM或键盘时间序列。

## 3. 控制链路与唯一写入点

```text
编码器、IMU、电压 -> 传感器快照 -> 里程/航向估计 -> 任务/路径执行
未来视觉/反射传感器 -> 带有效期的道路/元素观测 -> 任务/路径执行
蓝牙/板载按键 -> 命令、配置或事件 -> 模式与任务管理

当前模式的运动请求 -> 控制权仲裁 -> 安全检查 -> 车体线/角速度控制
                  -> 差速运动学 -> 左右轮速度环 -> 安全最终门控 -> 电机驱动

所有模块的状态快照 -> 遥测、故障记录、主机回放
```

外部视觉与蓝牙都不能直接写PWM或修改PID内部变量。只有底盘控制任务能调用电机提交接口；底层故障/急停门控必须始终能覆盖它。

运动学使用 `v_left = v - omega * track_width / 2`、`v_right = v + omega * track_width / 2`，之后按各轮半径换算轮轴角速度。轮速上限导致饱和时同比缩放两轮目标，避免独立裁剪改变曲率。轮速环、角速度环、位置/航向外环的带宽依次降低，通过实验确定参数。

## 4. 模块边界

| 模块 | 负责内容 | 后续修改应局限于 |
|---|---|---|
| Board / Motor / Encoder / IMU | 引脚、总线、采样和实际输出；报告驱动错误 | 换板、换驱动、换传感器 |
| Estimator | 编码器里程、IMU零偏、连续与归一化航向、车体速度、健康状态 | 定位算法、标定与融合 |
| Chassis / PID | v/omega目标、限速/加减速、差速解算、轮速环、控制器复位 | 运动控制算法 |
| Motion / Path | 单个距离/角度/圆弧动作、路径进度和超时 | 路径跟踪策略 |
| Mission | 任务状态机、动作调度、元素事件、等待和终点 | 赛题流程 |
| Observation | 灰度/视觉等道路与元素观测的统一入口 | 新感知来源 |
| Protocol / Transport | 有界收发、帧校验、命令序号和确认；不执行控制算法 | 蓝牙/视觉串口协议 |
| Supervisor / Safety | 模式、控制权、安全条件、故障锁定与恢复 | 模式与保护策略 |
| Parameters / Telemetry | 参数元数据、验证、版本保存；只读快照记录 | 调参与诊断 |

模块数量不等于必须立即创建同等数量的文件；按可独立测试的边界拆分，沿用现有Keil/C结构，不先移植ROS或堆入RTOS。

## 5. 接口契约草案

以下是设计签名，不是已经存在的函数。实现时每个接口都要有实际调用方、错误返回、边界和测试，避免只有大量空钩子。

统一车体坐标：前为+x，左为+y，逆时针转向为正；控制与定位内部使用 m、s、rad、m/s、rad/s。rpm和角度只在适配层转换。轮半径、轮距和每轮CPR分别校准，不能假设两个电机完全相同。

```c
typedef struct {
    uint32_t sample_us, sequence, valid_mask;
    uint32_t encoder_sample_us, imu_sample_us;
    int32_t delta_left, delta_right;
    float gyro_z_rad_s;
    uint16_t battery_mv;
} SensorSnapshot;

typedef struct {
    uint32_t sample_us, valid_mask;
    float x_m, y_m, yaw_rad, yaw_unwrapped_rad;
    float v_mps, omega_rad_s;
} VehicleState;

typedef struct {
    uint32_t sequence, rx_us, source_time_us, valid_mask;
    float lateral_error_m, heading_error_rad;
    float curvature_inv_m, confidence;
    uint16_t element_type;
} TrackObservation;

typedef struct {
    uint32_t request_id, issued_us, valid_for_us;
    uint32_t owner_epoch;
    float v_mps, omega_rad_s;
    MotionSource source;
} MotionRequest;

CarStatus Sensors_GetSnapshot(SensorSnapshot *out);
CarStatus Estimator_Update(const SensorSnapshot *in, float dt_s,
                           VehicleState *out);
CarStatus Observation_Publish(const TrackObservation *in);
CarStatus Motion_Submit(const MotionRequest *request);
CarStatus Mission_Load(const MissionPlan *plan);
CarStatus Mission_PostEvent(const MissionEvent *event);
CarStatus Mission_GetStatus(MissionStatus *out);
CarStatus Params_Set(ParamId id, ParamValue value);
void Safety_LatchFault(FaultCode code);
CarStatus Safety_RequestClear(void);
```

- `GetSnapshot`拷贝一致快照，不暴露可修改的全局控制器指针。控制任务拥有控制器可变状态，其他任务通过有界队列/邮箱提交请求。
- 每种传感器要能给出独立采样时刻或明确规定快照同步偏差上限；结构中的单一时间不能掩盖IMU是旧值。实际实现应为IMU/编码器单独保留年龄。
- Observation的本地接收时刻与相机曝光/计算时刻分开。未做时钟同步时不能直接相减两块MCU的时间；用序号、本地接收年龄和约定的端到端延迟上限判定。不同字段由valid_mask表达，不能以0代表“缺失”。
- 上述观测定义为车体局部道路误差，不是全局绝对定位，不能收到图像误差就重置全局x/y。未来需要绝对位姿时另定义带坐标系与有效性契约的定位观测。
- MotionRequest的有效期由车端产生/验证。每个周期路径执行器生成新请求；蓝牙断链不影响本地任务请求新鲜度，但任务执行器卡死会使请求到期停车。
- Mission动作至少覆盖 `DRIVE_DISTANCE`、`TURN_RELATIVE`、`FOLLOW_ARC`、`WAIT`、`STOP`。每个动作有编号、超时、容差、最大速度、失败策略；参数或路径整包校验成功后才替换，ACK明确区分“接收”与“动作完成”。
- 路径动作的完成由反馈误差进入容差并稳定若干周期判定；不仅检查“累计距离超过某个值”。加减速应考虑剩余制动距离。
- 模块返回 `OK / BUSY / INVALID / STALE / UNAVAILABLE / FAULT` 等明确状态；API不可用时明确拒绝，不能返回伪成功。
- 控制权绑定模块入口，不能信任外部载荷自报source。模式变化、STOP、PAUSE和故障递增owner_epoch并清空旧请求；此前代次的排队消息即使TTL未过也不得执行。

## 6. 状态机与控制权

监督状态：`BOOT -> SELF_TEST -> CALIBRATING -> READY -> ACTIVE -> FINISHED`，活动期间允许显式PAUSE；任意危险状态进入锁定FAULT。模式与监督状态分开：TELEOP、AUTO_PATH、以后接入的VISUAL_TRACK。

任务状态机只在ACTIVE且模式授权时运行，按动作完成/观测事件迁移；后续环岛、断路、路障分别是任务子状态，避免把所有元素和故障塞进一个巨型switch。

| 事件 | TELEOP | AUTO_PATH | 未来VISUAL_TRACK |
|---|---|---|---|
| 蓝牙驾驶命令过期 | 停车并取消遥控权 | 拒绝遥控输入，不干扰有效本地任务 | 拒绝遥控输入 |
| 蓝牙断开/日志丢失 | 停车 | 路径与必需传感器健康时继续 | 视觉链路健康时继续 |
| 必需编码器/IMU失效 | 按模式安全条件停车 | 立即中断任务并故障停车 | 依据明确的降级契约，否则停车 |
| 视觉/边界数据过期 | 无关 | 不使用未配置观测源 | 有界短时桥接或停车，禁止无限沿用旧误差 |
| 本地急停或有效STOP | 覆盖所有请求 | 覆盖所有请求 | 覆盖所有请求 |
| 复位/重新连接 | 不自动发车 | 不自动恢复旧任务 | 不自动发车 |

STOP、PAUSE、清故障和START是不同事件。清故障只在底层故障已消失、传感器健康且车已安全停止时接受；不得收到普通STOP就无条件抹去故障。换模式也不能让旧请求或旧任务自动重新运动。

PAUSE保存动作进度但取消控制权；RESUME必须是显式事件，在车已停止、定位连续、必需传感器健康时重新授权并从剩余目标重新规划，不能重新执行已完成动作。默认WAIT只累计任务活动时间。复位、里程重置或暂停期间定位不连续则拒绝恢复旧进度，要求重新装载/定位。

视觉桥接默认关闭。只有已验证的明确任务状态才能启用，同时设置最大持续时间和最大估计行程，任一先到即退出并停车；退出桥接需要连续若干有效观测且通过位置/航向窗口检查。160cm断路是规划场景，不等于任何位置突然丢线都可进入无限盲跑。具体阈值按动作、速度、停止距离和实车误差预算确定，未配置的策略不得启用。

## 7. 时间、通信与保护

- 初始目标采用200Hz底盘控制；传感器总线采样、路径任务与遥测频率由预算和实测确定。使用单调时间和真实dt，异常dt拒绝更新并进入定义好的保护策略，不直接把漏掉的周期当作正常5ms。
- 编码器与IMU分别检查sample_us年龄，不用“刚发布的快照”掩盖旧IMU。采用无符号差值处理32位时钟回绕；所有比较的允许间隔限制在半周期内。最大采样年龄、最大控制dt及桥接阈值须在对应模式启用前配置、验证并记录，不能用无限等待或默认健康代替。
- 中断只做有界采样/缓冲发布或经过预算的控制运算；禁止在最高优先级控制中断里做HAL阻塞I2C、Flash保存、printf或整帧轮询发送。总线操作要有限时完成并能报告错误；I2C超时不得依赖会被当前中断屏蔽的SysTick。
- 多字节快照通过双缓冲或极短临界区一致发布；静态有界内存，不在控制循环malloc；通信过载丢低优先级遥测，不能饿死控制或阻塞急停。
- 调试蓝牙和未来视觉串口分离；接收层使用DMA/空闲中断或有界环形队列。帧至少包括版本、类型、长度、序号、载荷和CRC；固定最大长度、残帧超时和复位会话规则。重复START/动作不得重复执行，乱序不能覆盖新目标；CRC只检测误码，不等于身份验证。
- 参数有量纲、合法范围、可修改状态、默认值和版本；整组验证后原子提交。存储使用版本+长度+CRC以及断电保护，不在运动中擦Flash；不盲目把Ki从旧“每拍累加”单位换成“每秒积分”数值。
- 保护覆盖堵转、编码器方向、过速、欠压（需电压采样标定）、IMU无响应/陈旧、周期超时和独立看门狗；看门狗仅在各关键任务正常推进时喂，不因串口还活着就喂。
- 记录故障首因、时间、模式、请求来源、目标/实测/实际施加PWM及饱和状态。换向等待期必须报告实际施加零PWM，避免把请求值误当作实际输出。

## 8. 验收顺序

1. 传感器：确认WHO_AM_I、总线引脚、轴向与正号、静止零偏和噪声；标定轮径、轮距、左右CPR。未确认硬件时软件不得伪造成功。
2. 主机测试：正反向运动学、连续/跨零航向、直线/圆弧里程、异常dt、计时器回绕、NaN/无穷、参数原子提交、队列满、重复/乱序命令、传感器数据陈旧、状态迁移和任何状态下急停。
3. 动作执行：模拟传感器输入可完成直线、转向、圆弧和等待，能证明超时/超调/失效后的行为；不是只测试某个函数能被调用。
4. 架空单轮：方向、测速、零速、输出限幅、失效停车、换向与看门狗。
5. 落地：多次1m/2m直线、正反90度转弯、圆弧/闭合路径，测横向误差、角度误差和重复性；暂不虚构误差达标值。
6. 离线运行：启动后断开蓝牙仍按AUTO_PATH运行，任务请求/IMU过期会停车；TELEOP断链必须停车，两种策略都有回归测试。
7. 短赛段和160cm断路验证通过后，才逐步扩展整圈；记录误差预算、车宽余量和实际停止距离。编译通过不是实车完赛证据。

## 9. 当前交付范围

本轮交付开源对照、缺口清单和接口设计；不把旧蓝牙代码说成已经具备自主导航，不新建空接口伪装实现，不进行GPIO/实车动作，也不发布GitHub。确认控制路线后按模块实现并逐步验收，已有底层PWM/编码器和经过验证的PID部分可继续复用。

## 10. 已核对的实现证据

本轮阅读了源文件，未只依据README。以下是可借鉴机制，不意味着这些项目的所有实现都健壮，平台和车模差异也不能忽略。

- [ECC_BLDC / ins.c](https://github.com/engmarkc/ECC_BLDC/blob/4b5fd8ba7a6f7206909b9b5226e228027e77c8d4/code/software/ins.c#L14)：实际用编码器里程和IMU航向积分平面位置；加速度双积分路径被注释。[control.c](https://github.com/engmarkc/ECC_BLDC/blob/4b5fd8ba7a6f7206909b9b5226e228027e77c8d4/code/software/control.c#L695)有航点到达和推进逻辑，[flash.c](https://github.com/engmarkc/ECC_BLDC/blob/4b5fd8ba7a6f7206909b9b5226e228027e77c8d4/code/device/flash.c#L419)有记录/加载。该车使用TC264D、BLDC和舵机，方向PID直接驱动舵机，不是已完成的差速车角速度串级控制。
- [20th_smart_vision / fsm.c](https://github.com/0Leeeezy0/20th_smart_vision/blob/8dd003ecc91a148ccfa51cb7a695e41e88a86752/project/code/fsm.c#L24)：任务状态由角度、里程和完成标志推进；[control.c](https://github.com/0Leeeezy0/20th_smart_vision/blob/8dd003ecc91a148ccfa51cb7a695e41e88a86752/project/code/control.c#L5)有车体目标到轮速控制的链路。但原车是三轮运动学，状态机与运动目标共享全局变量，不能整体照搬。
- [Wheel-legged-autocar / vision_ipc.h](https://github.com/boatchanting/Wheel-legged-autocar/blob/00bdea46b53b9de50b8187a995ecf8bfd1bdf4f3/code/vision/vision_ipc.h#L76)有序号、有效字段、置信度和道路几何；[任务消费者](https://github.com/boatchanting/Wheel-legged-autocar/blob/00bdea46b53b9de50b8187a995ecf8bfd1bdf4f3/code/vision/vision_three_stage_control.c#L325)有观测陈旧和状态超时检查。它是双核共享内存通信，不是可直接移植的F407串口协议；字段名crc实际使用折叠求和，仍须自行定义校验与时间戳契约。
- [ros2_controllers / diff_drive_controller.cpp](https://github.com/ros-controls/ros2_controllers/blob/master/diff_drive_controller/src/diff_drive_controller.cpp)：可参考车体v/omega接口、独立轮径/轮距、目标限速、命令新鲜度、里程计和电机写入的边界；本项目不移植ROS运行时。
- [Pololu QTR官方说明](https://www.pololu.com/docs/0J19/all)：反射式传感器测的是红外反射，支持原始值和校准。由此不能推断本赛道蓝色背景一定有足够反差；必须在赛道材料、安装高度和光照条件下测量。短阵列也不天然覆盖45cm宽白色路面，需重新设计边界可观测布局。

完整源码笔记：`.team/notes/02-inertial-research.md`、`.team/notes/03-extension-research.md`。固定提交与函数行号见笔记；本轮未复制外部源码，应用层许可不明确时仅借鉴机制。

## 11. 本地审查发现与实施优先级

详见 `.team/notes/01-current-audit.md`，以下发现尚未在本轮修复：

1. MPU6050读失败仍解包未初始化或不完整raw数据，WHO_AM_I身份/配置写入未充分检查。IMU接口没有样本有效性；当前车控尚未调用它，不能说已在实车触发。
2. 库的阻塞I2C依赖HAL_GetTick超时；若直接移入优先级0控制ISR，低优先级SysTick无法推进，故障时可能卡死。新采集层必须在控制ISR外有界完成，而不是原样接入现有循环。
3. 当前测速、PID、斜坡与超时都以固定5ms调用次数计算，需要真实采样时间、异常间隔策略以及Ki/Kd单位迁移。
4. 现有协议直接修改控制状态，所有RUN共用蓝牙租约，没有AUTO控制权与车载请求有效期。下一版要明确拆开；不能靠伪造心跳或简单取消所有超时解决。
5. 参数范围仅受16位编码限制，故障首因可被后续错误覆盖，STOP会无条件清故障；需要按新模式设计集中校验和恢复前置条件。
6. 当前遥测在主循环轮询整帧发送，9600下68字节约70.8ms。现有轮速ISR仍能抢占，但不能把未来IMU采集/任务服务简单追加在其后；应改为有预算或异步发送。

实施顺序：先修采样/时间/安全和参数契约，再实现车体控制与定位，再实现动作/状态机，最后接新的通信与观测适配器。每一步都提供可独立验证的行为，避免一次加入所有概念后再依靠实车找问题。
