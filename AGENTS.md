# 项目说明

本项目是基于 STM32H723、FreeRTOS 和 GSRL 的 C/C++ 固件工程，使用 CMake、Ninja 和 arm-none-eabi 工具链构建。

## 文件结构

- `CubeMX_BSP/`：CubeMX 配置（`.ioc`）、芯片初始化、HAL/CMSIS、FreeRTOS、启动文件和链接脚本，由CubeMX生成，一般不直接编辑操作。
- `GSRL/Algorithm/`：控制、滤波、CRC 等算法；`inc/` 放头文件，`src/` 放实现。
- `GSRL/Device/`：遥控器、电机、IMU、裁判系统等设备封装；`inc/` 放头文件，`src/` 放实现。
- `GSRL/Driver/`：UART、CAN、SPI 等底层通信驱动；`inc/` 放头文件，`src/` 放实现。
- `GSRL/Include/`：板级配置、公共类型和常量。
- `GSRL/Dependence/`：CMSIS-DSP、Eigen 等第三方依赖。
- `GSRL/Documentation/`：库和设备使用文档。
- `GSRL/Template/`：代码模板。
- `Task/src/`：应用任务和设备接入代码，如 `tsk_test.cpp`。
- 根目录 `CMakeLists.txt`、`CMakePresets.json`：工程构建入口和 Debug/Release 配置。
- `cmake/`：配置生成脚本及模板；`.github/`：CI 等仓库配置；`.vscode/`：编辑器配置。
- `build/`：已有构建产物。
- `tmp/`：代理测试和临时工作文件，已被 Git 忽略；`tmp/et08a/` 放主机测试和报告，`tmp/AgentsMD/` 放本次验证的构建产物与日志。

## 文件放置规则

- **只可在明确授权下新增文件夹，包括现有目录下的新子目录。** 否则只能使用现有目录，不得通过脚本或工具间接创建新目录。
- **代理生成的测试源码、桩文件、验证脚本、测试数据、测试报告、日志及测试产物全部放入现有 `tmp/`。**
- 不得将代理新增的测试文件放入 `Tests/`、`GSRL/`、`Task/` 或项目根目录。
