# Motion TrafficSwitch

基于安路 PH1P35 FPGA 的相机图像处理、运动检测、交通灯/箭头识别与 PC 监控源码。

本仓库从 2026-11-03 的当前工作区整理，包含尚未提交到原仓库的最新源码。

## 目录

- `01_FPGA/SQ_anlogic2/user_source/hdl_source/`：Verilog/SystemVerilog/VHDL、图像处理、识别、显示、串口及所需 IP 源文件。
- `01_FPGA/SQ_anlogic2/user_source/ip_source/`：工程使用的 IP 配置及 HDL 包装文件。
- `01_FPGA/SQ_anlogic2/user_source/constraints_source/`：引脚与时序约束。
- `01_FPGA/SQ_anlogic2/td_project/`：TD 工程及工程时序约束。
- `02_PC/`：Tkinter 监控程序、协议与控制模块、Windows BLE 工作脚本和 Python 依赖声明。
- `tools/`：工程迁移、FPGA 构建与上位机启动脚本。

不包含 STM32、sim/仿真、测试代码、验证记录、比特流、编译结果、照片、报告、交付包、虚拟环境和旧 Git 历史。IP 配置文件与初始化表属于工程输入，随源码保留。

## 启动 PC 上位机

需要 Python 3、Tkinter 和 pySerial。Tkinter 由 Python/操作系统安装提供；Windows BLE 功能还需要 Windows PowerShell/WinRT。

```bash
python3 -m venv .venv
# Linux/macOS
source .venv/bin/activate
# Windows PowerShell 使用 .venv\Scripts\Activate.ps1
python -m pip install -r 02_PC/requirements.txt
python tools/run_car_remote.py
```

运行时生成的转向标定、日志和 CSV 已加入忽略规则。

## FPGA 工程

器件：`PH1P35MDG324`，速度等级 `-3`；工程记录的 TD 版本为 `6.2.178840`。TD 工具及器件库需自行安装。

建议先生成适用于本机路径的工作副本，再用 TD 打开生成的 `.al` 文件：

```bash
python tools/prepare_td_project.py --output-dir /tmp/motion_traffic_td
```

目标目录须尚不存在，且路径使用 ASCII 字符。原始 `.al` 的工程路径已整理为相对路径。

也可通过构建脚本指定本机 TD 可执行文件：

```bash
python tools/build_arrow.py --td /path/to/TD/bin/td.sh
# TimingHigh 策略
python tools/build_arrow_timing.py --td /path/to/TD/bin/td.sh
```

Windows 下将 `--td` 指向 `td_commands_prompt.exe`。也可通过 `TD_INSTALL_DIR` 指定 TD 安装目录。构建输出由脚本生成，默认位于忽略的 `03_Verification/runtime/`；本仓库不附预编译比特流。

## 第三方文件

保留所需厂商 IP/HDL 内的原有版权与许可声明。本仓库未额外授予覆盖全部文件的开源许可证。
