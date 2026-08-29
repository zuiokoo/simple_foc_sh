项目：ESP32 + ESP-IDF + FreeRTOS 手写简化版 FOC

学习目标：
我是一名初学者，希望一边学习 FreeRTOS、三相电机控制和 SimpleFOC 的理论，一边不依赖 Arduino、不直接使用 SimpleFOC 库，自己用 C 语言逐步实现 FOC。

请按“小步骤讲解 + 直接修改代码 + 编译验证”的方式继续。
不要一次写完整套 FOC，每一步都要解释：
1. 这一步解决什么问题
2. 为什么需要它
3. 数据如何流动
4. 写了哪些函数
5. 如何验证
6. 当前没有示波器、逻辑分析仪、电机和传感器，先做软件层开发，不要启动功率输出

====================
一、开发环境
====================

项目目录：
C:\Users\Administrator\Desktop\simple_foc

开发工具：
- PlatformIO
- framework = ESP-IDF
- ESP-IDF 5.5.0
- PlatformIO espressif32@6.12.0
- C语言
- FreeRTOS
- ESP32-D0WD-V3，双核
- 串口波特率 115200
- 实际 Flash 为 16MB

不是 Arduino 项目，不使用 Arduino API。

项目目前能正常：
- 编译
- 下载
- 启动
- 输出 ESP-IDF Boot 日志
- 运行 FreeRTOS WS2812 任务

====================
二、硬件连接
====================

三相栅极驱动芯片：
EG2133

PCB 每相使用一个 PWM GPIO，同时连接该相 EG2133 的：
- HINx
- /LINx

已确认：
GPIO32 对应 M1 PWM1，并同时连接 EG2133 的 HIN1 和 /LIN1。

三相 PWM：

U相：GPIO32
V相：GPIO33
W相：GPIO25

EG2133 内部具有互锁和约 100ns 死区。
高侧采用自举供电，所以暂时把 PWM 占空比限制在 5%～95%，避免长期 0% 或 100%。

AS5600：

SCL：GPIO19
SDA：GPIO21
I2C地址：0x36

电流采样：

M1_CURRENT_U：
GPIO36
ADC1_CH0
SENSOR_VP，模块引脚标号4

M1_CURRENT_V：
GPIO39
ADC1_CH3
SENSOR_VN，模块引脚标号5

GPIO36、GPIO39 只能输入，适合 ADC 采样。

====================
三、已有任务
====================

项目中已经有：

1. 普通 LED 任务
2. SPI + DMA 驱动的 WS2812 任务
3. app_main() 中初始化电机 PWM

WS2812 任务会循环打印：

WS2812: red
WS2812: green
WS2812: blue
WS2812: white
WS2812: off

app_main() 返回后 FreeRTOS 调度器仍然运行，所以：

main_task: Returned from app_main()

是正常现象。

FOC 不应该给 Clarke、Park、PI、SVPWM 分别创建任务。
这些应该是高速控制环中的普通函数。

未来建议任务结构：

- 高速 FOC 控制环：10～20kHz
- 慢速状态/传感器任务：500Hz～1kHz
- LED任务：低频
- 通信任务：按需

====================
四、当前 PWM 配置
====================

文件：
src/motor/motor_config.h

主要配置：

#define M1_PWM_U_GPIO GPIO_NUM_32
#define M1_PWM_V_GPIO GPIO_NUM_33
#define M1_PWM_W_GPIO GPIO_NUM_25

#define M1_CURRENT_U_GPIO GPIO_NUM_36
#define M1_CURRENT_V_GPIO GPIO_NUM_39

#define M1_SENSOR_SDA_GPIO GPIO_NUM_21
#define M1_SENSOR_SCL_GPIO GPIO_NUM_19
#define M1_AS5600_ADDRESS 0x36

#define M1_PWM_TIMER_RESOLUTION_HZ 10000000UL
#define M1_PWM_FREQUENCY_HZ 20000UL

#define M1_PWM_PERIOD_TICKS \
    (M1_PWM_TIMER_RESOLUTION_HZ / \
    (M1_PWM_FREQUENCY_HZ * 2UL))

#define M1_PWM_MIN_DUTY 0.05f
#define M1_PWM_MAX_DUTY 0.95f

为什么 period_ticks 是250：

Timer分辨率 = 10MHz
PWM频率 = 20kHz
采用 UP_DOWN 中心对齐计数

period_ticks =
10000000 / (20000 × 2)
= 250

一次完整 PWM 周期：

0 → 250 → 0

总计数时间为500 tick，因此频率是20kHz。

====================
五、已经完成的 motor_pwm 模块
====================

文件：

src/motor/motor_pwm.h
src/motor/motor_pwm.c

头文件已经声明：

esp_err_t motor_pwm_init(void);
esp_err_t motor_pwm_set_duty(
    float duty_u,
    float duty_v,
    float duty_w
);
esp_err_t motor_pwm_start(void);
esp_err_t motor_pwm_stop(void);

MCPWM结构：

一个公共 Timer
    ├── U相 Operator
    │     ├── Comparator
    │     └── Generator → GPIO32
    ├── V相 Operator
    │     ├── Comparator
    │     └── Generator → GPIO33
    └── W相 Operator
          ├── Comparator
          └── Generator → GPIO25

三相共用 Timer，保证 PWM 同步。

motor_pwm_init() 已完成：

1. 创建中心对齐 MCPWM Timer
2. 创建三个 Operator
3. 三个 Operator 连接公共 Timer
4. 创建三个 Comparator
5. 初始比较值设置为125，即50%
6. 创建三个 Generator
7. Generator 绑定 GPIO32、33、25
8. 配置中心对齐动作
9. 初始化结束后三相强制保持低电平
10. Timer 不会自动启动

中心对齐动作：

向上计数到 Comparator：
MCPWM_GEN_ACTION_LOW

向下计数到 Comparator：
MCPWM_GEN_ACTION_HIGH

因此：

0 → compare：高
compare → 250：低
250 → compare：低
compare → 0：高

占空比计算：

duty = compare_ticks / period_ticks

例如：

50% → 125 tick
25% → 63 tick
75% → 188 tick

motor_pwm_set_duty() 已完成：

- 输入三相浮点占空比
- 使用 clamp_duty() 限制到5%～95%
- 换算为 compare_ticks
- 调用 mcpwm_comparator_set_compare_value()
- Comparator 配置 update_cmp_on_tez=true
- 新值在 Timer 到达零点时更新，避免周期中途突然改变

示例：

motor_pwm_set_duty(0.25f, 0.50f, 0.75f);

得到：

U = 63 tick
V = 125 tick
W = 188 tick

该函数不会自动启动 PWM。

motor_pwm_start() 已完成：

1. 检查是否初始化
2. 启用 MCPWM Timer
3. 启动 Timer
4. 解除三个 Generator 的强制低电平
5. PWM 开始输出

motor_pwm_stop() 已完成：

1. 立即把三个 Generator 强制为低电平
2. 再让 Timer 停在 EMPTY
3. 避免停止后某一相保持高电平

====================
六、目前实际运行状态
====================

main.c 当前只调用：

motor_pwm_init();

暂时没有调用：

motor_pwm_start();

因此当前：

GPIO32 = LOW
GPIO33 = LOW
GPIO25 = LOW
Timer没有运行
不会输出连续PWM
不会驱动电机

最近一次正确启动日志：

MOTOR_PWM: MCPWM timer created:
resolution=10000000 Hz,
PWM=20000 Hz,
period=250 ticks

MOTOR_PWM: Three MCPWM operators created and connected
MOTOR_PWM: Three comparators created, initial duty=50%
MOTOR_PWM: Three PWM generators configured:
U=GPIO32, V=GPIO33, W=GPIO25
MOTOR_PWM: PWM initialized in safe LOW state

代码已经通过 PlatformIO 编译。

====================
七、重要安全要求
====================

当前手边没有：

- 示波器
- 逻辑分析仪
- 电机
- AS5600
- 可用于验证的完整功率硬件

所以继续开发时：

1. 不要在 app_main() 中调用 motor_pwm_start()
2. 不要自动启动三相功率输出
3. 可以实现函数和软件测试
4. 高速函数内部不要频繁 ESP_LOG
5. 测试打印放在独立测试函数或低频任务中
6. 等硬件齐全后再进行 GPIO PWM 和功率级验证

====================
八、整个 FOC 开发路线
====================

阶段1：三相PWM底层驱动
状态：软件部分基本完成，等待硬件验证

阶段2：AS5600驱动
状态：基础读取与换算已完成，等待本轮编译和串口验证

已实现：

as5600_init();
as5600_read_raw_angle();
as5600_get_mechanical_angle();
as5600_get_velocity();

计划使用 ESP-IDF 5.5 的新版 I2C Master API。

阶段3：ADC电流采样
状态：尚未开始

需要实现：

current_sense_init();
current_sense_calibrate();
current_sense_read();

采样：

iu：GPIO36 / ADC1_CH0
iv：GPIO39 / ADC1_CH3

第三相：

iw = -(iu + iv)

但是目前还不知道：

- 采样电阻阻值
- 运放增益
- ADC零点偏置
- 电流采样电路方向

因此可以先搭建 ADC 原始值读取框架，不要直接换算为安培。

阶段4：开环电压控制
状态：尚未开始

目标：

电角度不断增加
    ↓
生成旋转电压矢量
    ↓
反Park变换
    ↓
SVPWM
    ↓
调用 motor_pwm_set_duty()

阶段5：FOC数学模块
状态：尚未开始

需要实现：

clarke_transform();
park_transform();
inverse_park_transform();
svpwm();

数据流：

iu、iv、iw
    ↓ Clarke
i_alpha、i_beta
    ↓ Park
id、iq
    ↓ PI控制器
vd、vq
    ↓ 反Park
v_alpha、v_beta
    ↓ SVPWM
duty_u、duty_v、duty_w
    ↓
motor_pwm_set_duty()

阶段6：闭环FOC
状态：尚未开始

需要：

- 电机极对数
- AS5600机械角度
- 电角度零点校准
- 转向确认
- 电流零点校准
- Id/Iq PI控制器
- 电流限制
- 故障停机

电角度基本关系：

electrical_angle =
mechanical_angle × pole_pairs
- zero_electrical_angle

阶段7：FreeRTOS整合
状态：尚未开始

不要给每个数学函数创建任务。

可能架构：

PWM中断/高优先级控制环：
- 采样电流
- 读取/更新角度
- Clarke
- Park
- PI
- 反Park
- SVPWM
- 更新Comparator

低频任务：
- 串口日志
- LED状态
- 命令输入
- 故障显示

====================
九、推荐下一步
====================

因为目前没有测量工具，先不要做实际 PWM 启动测试。

推荐下一步开始 AS5600 模块的软件框架：

建议创建：

src/sensor/as5600.h
src/sensor/as5600.c

第一小步只做：

1. 创建 ESP-IDF I2C Master Bus
2. SCL设置为GPIO19
3. SDA设置为GPIO21
4. 频率先使用400kHz或100kHz
5. 添加AS5600设备地址0x36
6. 初始化时不要因为设备不存在导致系统崩溃
7. 返回清楚的esp_err_t错误
8. 编译验证
9. 当前没有AS5600，运行时“设备无响应”属于预期结果

在写代码前，先检查项目当前文件内容和 ESP-IDF 5.5 本地头文件，确认使用新版API：

i2c_new_master_bus()
i2c_master_bus_add_device()
i2c_master_transmit_receive()

不要使用 Arduino Wire，也尽量不要使用旧版：

i2c_param_config()
i2c_driver_install()

继续保持一小步一小步实现，并向初学者解释每个结构体、句柄和函数的作用。

====================
十、2026-08-26 学习与开发进度
====================

今天完成的内容：

1. 确认当前项目继续使用 PlatformIO + ESP-IDF 5.5 + 原生 FreeRTOS，不使用 Arduino Wire 或 SimpleFOC。
2. AS5600 使用 ESP-IDF 5.5 新版 I2C Master API：
   - i2c_new_master_bus()
   - i2c_master_bus_add_device()
   - i2c_master_transmit_receive()
3. 确认 AS5600 硬件配置：
   - SDA：GPIO21
   - SCL：GPIO19
   - I2C 地址：0x36
   - I2C 频率：400kHz
4. 完成原始角度读取：
   - 从 0x0C 和 0x0D 读取 RAW ANGLE
   - 合并为 12 位原始值，范围 0~4095
5. 完成机械角度转换：
   - as5600_raw_to_mechanical_angle()
   - 原始值转换为 0~2π rad
   - as5600_get_mechanical_angle() 合并“读取原始值 + 转换机械角度”
6. 完成机械角速度计算：
   - as5600_get_velocity()
   - 使用角度变化量 ÷ 时间变化量
   - 使用 esp_timer_get_time() 获取实际微秒时间
   - 处理角度从 2π 跳回 0 的情况
   - 第一次调用只保存角度和时间，速度返回 0
   - 后续每次调用都会重新计算并更新 velocity_rad_s
7. 在 main.c 的 as5600_test_task() 中同时打印：
   - mechanical angle：机械角度，单位 rad
   - velocity：机械角速度，单位 rad/s
8. 运行保护保持不变：没有调用 motor_pwm_start()，暂不进行实际三相功率输出。

重要概念记录：

- 机械角度：转轴物理上转过的位置，AS5600直接测量的就是它。
- 电角度：用于描述转子磁场周期的位置，不是第二个传感器角度。
- 电角度的基础关系：

  electrical_angle = mechanical_angle × pole_pairs

- 极对数来自转子永磁体的磁极数量：

  pole_pairs = total_magnetic_poles / 2

- 定子上是 U、V、W 三相绕组。通电后定子产生可控制的旋转磁场，转子永磁体跟随定子磁场运动。
- 当前还没有确定电机极对数，也没有做电角度零点校准，因此现在只验证机械角度和机械角速度。

当前代码位置：

- src/sensor/as5600.h
- src/sensor/as5600.c
- src/main.c

下次继续顺序：

1. 编译验证当前 AS5600 速度代码。
2. 烧录运行，观察机械角度和机械角速度日志。
3. 手动转动磁铁，确认正转速度为正、反转速度为负；静止时允许有少量噪声。
4. 注意当前测试任务每个周期会读取两次 AS5600，这不是功能错误，后面再优化。
5. AS5600验证完成后进入阶段3：ADC 原始电流采样框架。
6. 暂时不要启动三相 PWM，也不要在没有确认电机参数和电流采样前做闭环 FOC。

====================
十一、2026-08-28 最新进度
====================

以下内容以当前源代码为准，用于覆盖前面仍然保留的旧计划记录。

当前已经完成：

1. AS5600 角度和角速度读取
   - 使用 ESP-IDF 5.5 新版 I2C Master API
   - 机械角度单位为 rad
   - 机械角速度单位为 rad/s
   - 一个采样函数同时读取角度并计算速度

2. 三相电流采样
   - U相：GPIO36 / ADC1_CH0
   - V相：GPIO39 / ADC1_CH3
   - W相使用：iw = -(iu + iv)
   - 已完成 ADC 零点校准
   - INA240A2 增益为50，采样电阻为5毫欧
   - 当前零电流噪声约为几十毫安以内，属于当前 ADC 分辨率和模拟电路噪声下的测试现象

3. FOC 数学变换
   - Clarke：iu、iv、iw → i_alpha、i_beta
   - Park：i_alpha、i_beta → id、iq
   - 机械角度 → 电角度
   - 逆 Park：vd、vq → v_alpha、v_beta

4. 电角度零点校准
   - 电机为 FIT1034 2804，14极，7极对
   - 已通过低占空比固定电压矢量完成一次性校准
   - 当前校准值：

     M1_ELECTRICAL_ZERO_OFFSET_RAD = 3.253573f

   - M1_ENABLE_ELECTRICAL_ZERO_CALIBRATION 当前应保持为0
   - 这样上电不会每次重复校准

5. PI 电流控制器
   - 文件：src/control/foc_pi.h
   - 文件：src/control/foc_pi.c
   - id PI 输出 vd 电压指令
   - iq PI 输出 vq 电压指令
   - 当前测试目标：

     id_ref = 0 A
     iq_ref = 0 A

   - 当前 PI 只在100 ms测试任务中运行，用于观察数据流，不是真正的高速电流环
   - FOC_TEST_DT_S 当前为0.1 s，必须与测试任务的实际执行周期匹配

当前 main.c 数据流：

机械角度、角速度
    ↓
电角度
    ↓
三相电流 iu、iv、iw
    ↓ Clarke
i_alpha、i_beta
    ↓ Park
实际 id、iq
    ↓ 目标值 - 实际值
id_error、iq_error
    ↓ 两个PI
vd、vq
    ↓ 逆 Park
v_alpha、v_beta

当前还没有完成：

1. SVPWM：v_alpha、v_beta → duty_u、duty_v、duty_w
2. 将占空比真正交给 motor_pwm_set_duty()
3. 高速 FOC 控制环
4. 电流环参数整定
5. 转速环
6. CAN/485 控制命令
7. 完整的过流、欠压、传感器故障停机

当前安全状态：

- main.c 没有调用 motor_pwm_start()
- 当前不会持续输出三相 PWM
- 当前不会驱动电机
- 逆 Park 和 PI 结果只用于串口观察

回家后调试顺序：

1. 编译并烧录当前代码
2. 确认 AS5600 机械角度、电角度和电流日志正常
3. 确认日志中出现 id、iq、id_error、iq_error、vd、vq、valpha、vbeta
4. 暂时保持 FOC_TEST_IQ_REF_A = 0.0f
5. 不要调用 motor_pwm_start()
6. 当前代码确认无误后，下一步实现 SVPWM 占空比计算

下一步学习重点：

SVPWM 不是直接产生电压，而是把 v_alpha、v_beta 电压指令换算成三相 PWM 占空比。随后才会研究如何在确认硬件安全后，把占空比交给 motor_pwm_set_duty()。

====================
FOC 调试上下文（2026-08-29，当前源代码为准）
====================

本节用于后续继续调试。旧章节中的 GPIO32/33/25、只做软件数学测试等内容已经过期；后续以当前源代码和最新串口日志为准。

一、工程与硬件

- 工程目录：D:\platformio_workspace\simple_foc_sh
- PlatformIO + ESP-IDF 5.5.0 + 原生 FreeRTOS，不使用 Arduino/SimpleFOC。
- 芯片日志：ESP32-D0WD-V3；串口 COM7，CH340，115200。
- M1 PWM：U=GPIO26，V=GPIO27，W=GPIO14。用户已明确这组引脚，后续不得改成 GPIO32/33/25。
- M1 电流采样：U=GPIO36/ADC1_CH0，V=GPIO39/ADC1_CH3，W=-U-V。
- AS5600：SDA=GPIO21，SCL=GPIO19，地址=0x36，I2C=400kHz。
- 电机：FIT1034 2804，当前按 7 极对处理。

二、当前控制配置

- PWM：20kHz、MCPWM、中心对齐，计数器分辨率 10MHz。
- 电流环目标周期：1ms；PI 使用 esp_timer_get_time() 测得的实际采样间隔。
- 速度环：M1_ENABLE_SPEED_LOOP=0，当前只调电流环。
- 电角度零点：启动时执行一次对齐校准，M1_ENABLE_ELECTRICAL_ZERO_CALIBRATION=1。
- 当前本地起转测试点：Id_ref=0.05A，Iq_ref=0.05A。
- 电流 PI：Id/Kp=1.5，Id/Ki=8.0；Iq/Kp=1.5，Iq/Ki=8.0；单轴输出限制 -2V..+2V；软件母线电压按 8V。

三、ADC DMA 实现

文件：src/sensor/current_sense.c

- 使用 ESP-IDF adc_continuous API，ADC 连续 DMA，不再使用 adc_oneshot。
- 当前采样频率 200kHz，U/V 两个通道交替采样。
- DMA frame=8字节，结果缓存=1024字节，内部 store buffer=2048字节。
- 每次读取最多处理一块 DMA 数据，防止连续 ADC 数据产生死循环占满 CPU。
- 每个电流环周期先丢弃旧 DMA 数据，等待 PWM 低侧窗口，再在连续 4 个 PWM 窗口采集新样本并平均后送入 Clarke/Park/PI。
- U、V 电流方向按当前硬件反向处理，W 由无中性点公式重构。
- 重要限制：当前 DMA 是自由运行，尚未实现 PWM 硬件事件直接触发 ADC；软件等待 PWM 窗口只能近似同步，所以单点 d/q 遥测仍可能受 PWM 纹波影响。

四、FreeRTOS 和日志

- 实时电流任务优先级 7，不在实时任务中打印长浮点日志。
- FOC_TELEM 为低优先级遥测任务，每秒打印一次整数缩放值；任务栈为 4096，避免 newlib 浮点格式化导致栈溢出。
- WS2812/LED 任务在电流环模式下不创建，减少串口和任务调度干扰。
- 遥测字段：dt_us、speed_mrad_s、iq_ref_mA、三相电流 mA、id/iq mA、vd/vq mV、三个 duty 千分比。

五、已经验证的现象和结论

1. 启动日志确认 M1 PWM 为 GPIO26/27/14，PWM=20kHz。
2. 启动日志确认 ADC 为 GPIO36/39、ADC1_CH0/CH3、连续 DMA。
3. dt_us 多次处于约 0.95..1.04ms，PI 的时间间隔基本正确。
4. 三相电流通常满足 U+V+W≈0，采样比例和 W 相重构逻辑基本正确。
5. 遥测任务曾因大量 float printf 触发 foc_current_tel stack overflow，已改为整数打印并增大任务栈。
6. 1MHz 连续 DMA 曾因无界清空循环触发 IDLE0/task watchdog；已改成有界读取并降回 200kHz 低 CPU 负载配置。
7. 速度环关闭时，恒定 Iq 会持续给电机转矩，电机无负载会不断加速；高速后 vq 可能达到 2V 限幅，此时不能据此继续调 PI。
8. 最新运行日志中电机已能转到约 25..47rad/s，dt 正常，Iq 多点平均值接近 50mA；但单点 id/iq 仍有跳变。当前主问题是 ADC 自由运行与 PWM 低侧采样窗口没有真正硬同步，以及 PWM 电流纹波相对小电流较大。

六、本地验证

执行：

    C:\Users\jxkj\.platformio\penv\Scripts\python.exe -m unittest discover -s test -p 'test_*.py'
    C:\Users\jxkj\.platformio\penv\Scripts\platformio.exe run

当前结果：25 项 Python 测试通过，PlatformIO 固件编译成功。

编译仍有 Flash size mismatch 警告：工具链配置期望 16MB、构建环境检测到 2MB/板卡描述 4MB；这不是当前电流环主故障，后续单独统一 sdkconfig 和板卡 Flash 配置。

七、后续调试顺序

1. 下载当前本地最新镜像；检查启动日志中的编译时间、PWM 引脚、PWM 频率和 ADC 频率，确认镜像确实更新。
2. 静态验证：Id=0.05A、Iq=0A，观察 5..10 秒；目标是 id 平均接近 50mA、iq 平均接近 0、无 task_wdt。
3. 起转验证：Id=0.05A、Iq=0.05A，只观察启动阶段，不让电机长时间恒转加速。
4. 若采样仍跳变，优先实现 PWM 相位锁定/硬件触发 ADC，或在同一低侧窗口内采集并平均；不要先盲目增大 PI。
5. 只有电流采样稳定、低速 id/iq 可跟踪后，才恢复速度环。

八、串口和下载记录

- COM7 可枚举为 USB-SERIAL CH340；Cannot open COM port 曾是复位或串口监视器重连时的瞬时提示，不能单独判定为固件故障。
- 下载前关闭串口监视器；如下载检查要求，断开电机功率母线，只保留 USB。
- 每次下载后记录启动日志中的编译时间、PWM 引脚、PWM 频率、ADC 采样频率和第一段 FOC_TELEM，作为镜像是否更新的依据。
