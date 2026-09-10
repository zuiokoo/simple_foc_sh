# FOC 电流环调试器

这是 `D:\platformio_workspace\simple_foc_sh` 的 Python 上位机。它可以连接 ESP32 控制台、设置 Id/Iq 目标、显示实时电流环数据、保存 CSV，并请求固件执行安全的电角度零点对齐。

## 安装和运行

在 PowerShell 中执行：

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
$env:PYTHONPATH = (Get-Location).Path
.\.venv\Scripts\python.exe -m current_loop_tuner.app
```

如果已经在 `tools\current_loop_tuner` 目录内运行，直接执行：

```powershell
$env:PYTHONPATH = (Get-Location).Path
python -m current_loop_tuner.app
```

## 使用顺序

1. 机械部分保持安全，确认功率级和电机可以随时断电。
2. 连接 COM 口。连接时工具只设置遥测速率，不自动发送目标电流。
3. 需要校准时点击“电角度零点对齐”。固件会先清零目标，在电流环任务安全边界停止 PWM，执行约 1 秒对齐，然后重新启动 PWM 并保持目标为 0。
4. Id/Iq 目标范围为 -500～500 mA。点击“应用目标”才会发送目标。
5. 固定扭矩测试通常设置 Id=0，再设置需要的 Iq；Iq 是电流目标，不等同于未经标定的 N·m。
6. 点击“STOP / 清零”可将 Id 和 Iq 目标置零；断开连接时工具也会尽力发送 STOP。
7. 点击“串口日志”可打开类似串口助手的原始数据窗口，按接收顺序显示启动日志、FOC_DATA/FOC_TUNE、协议回应和错误，保留最近 5000 行；窗口支持暂停、清空和自动滚动。“保存CSV”仍保存当前内存中的最多 5000 条遥测数据。

## 固件协议

```text
FOC ID <signed_mA>
FOC IQ <signed_mA>
FOC STOP
FOC RATE <1..100>
FOC ALIGN
FOC STATUS
```

零点对齐期间不要触碰转子；固件仍保留电流、角度、采样超时和故障停机保护。工具不负责编译、烧录或自动启动电机。