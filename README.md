# mavlink_stm32
Using mavlink in stm32 to control quadrone

本项目中使用了：

* FreeRTOS
* Mavlink
* FSM

各个主要文件夹内容说明：

* FreeRTOS
  实时操作系统的核心代码，只需要根据实际情况修改`FreeRTOS/include/FreeRTOSConfig.h`中的内容即可，其余代码不用修改
* HARDWARE
  各个硬件模块的驱动程序，包括蓝牙、按键、LED、超声波、云台、激光笔等
* LOGIC
  高层控制逻辑，包括有限状态机、控制器（使用mavlink）、飞行操作等
* MESSAGE
  通信模块，处理mavlink信息、光流信息和蓝牙信息等
* USER



![design](md_files\设计.png)