# 2026电赛省赛G题：基于STM32H750的信号分析仪

电子设计竞赛省赛期间的作品固件，基于STM32H750实现信号采集、频谱分析、参数测量及触摸屏显示。仓库英文名建议为 `edc-province-signal-analyzer`。

## 主要功能

- ADC与DMA采集模拟输入。
- FFT频谱分析、谱峰提取及信号分量估计。
- 基频、峰峰值、有效值及各信号分量的幅值等参数计算。
- LCD波形、频谱和参数显示，触摸切换界面。
- 采集与信号处理相关标定参数配置。

## 开发环境与硬件

- MCU：STM32H750VBTx，Cortex-M7。
- 工程：STM32CubeIDE，STM32 HAL及CMSIS。
- 驱动来源：包含正点原子开发板的LCD、MPU等驱动，需匹配对应硬件。
- 固件配置采样率：2 MSa/s。
- ADC缓存长度：8192点；FFT长度：4096点。
- 配置最多分析3个信号分量，具体参数见 `Core/Inc/app_config.h`。

采样率和参数属于当前固件设置，实物测量能力还取决于时钟、模拟前端、标定及接线。请根据实际开发板和电路核对配置。

## 目录

| 路径 | 内容 |
| --- | --- |
| `Core/Src/acquisition.c` | ADC、DMA及采集控制 |
| `Core/Src/signal_process.c` | 信号处理及参数计算 |
| `Core/Src/app_ui.c` | 显示与触摸交互 |
| `Core/Src/main.c` | 初始化及主循环 |
| `Core/Inc/app_config.h` | 采样、FFT和标定配置 |
| `Core/Startup/` | 启动代码 |
| `Drivers/` | HAL、CMSIS及板级驱动 |
| `provincecomp.ioc` | CubeMX配置 |
| `.project`、`.cproject`、`.settings/` | CubeIDE工程设置 |
| `STM32H750VBTX_*.ld` | 链接脚本 |

## 导入与运行

1. 使用STM32CubeIDE的 **File → Import → General → Existing Projects into Workspace** 导入本目录。
2. 核对工程工具链、链接脚本，以及开发板的LCD和触摸屏配置。
3. 编译工程，连接调试器后烧录固件。
4. 按实际模拟前端接线输入信号，并根据实物进行标定。

ADC和部分外设在代码中手动初始化。重新从 `.ioc` 生成代码前，请备份并检查变更，避免覆盖手工实现。

`Debug/`、`Release/`及中间编译文件不纳入版本控制，编译时由IDE重新生成。本次仓库整理未重新编译或开展硬件测试。

## 第三方代码说明

HAL、CMSIS及板级驱动保留原有版权声明和许可证。相关许可证见各驱动目录下的 `LICENSE.txt`；板级驱动还包含正点原子版权声明。

本仓库不对全部第三方文件统一授予新的开源许可。对外发布或再分发前，应核实各组件的许可条件及团队成员的意见。
