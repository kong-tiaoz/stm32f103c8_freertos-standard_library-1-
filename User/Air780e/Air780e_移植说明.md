# Air780e 4G模块驱动移植说明 (MQTT 增强版)

本文档旨在记录 Air780e 模块驱动在 STM32F103C8T6 (裸机与 RTOS 兼容) 平台上的移植与使用方法。本次升级主要在原有 TCP 透传基础上，利用模块内置的 MQTT AT 指令栈，实现了完整、健壮的 MQTT 客户端模块。

---

## 一、 核心增强特性

1. **内建 MQTT 协议栈**：不再依赖外部 TCP 透传，直接使用模块的 `AT+MCONFIG` / `AT+MIPSTART` / `AT+MCONNECT` 等指令，大幅降低 MCU 端的 RAM 和 Flash 消耗。
2. **完整参数配置**：支持自定义 Broker 地址、端口、Client ID、用户名、密码、TLS 开关、遗嘱消息 (LWT) 以及 QoS 等级等关键参数。
3. **断线自动重连与指数退避**：当网络断开或服务器拒绝连接时，状态机会自动发起重连，并且重连间隔会呈指数级增长（最高 60 秒），防止频繁重连导致模块死机或 IP 被封。
4. **离线消息缓存**：受限于 C8T6 的 20KB RAM，实现了一个轻量级的环形缓冲区（最多缓存 3 条消息）。在网络断开时发布的指令将暂存在内存中，待网络恢复后自动发送。
5. **事件回调机制**：提供 `Air780e_MqttEventCallback_t`，可对连接成功、连接断开、收到订阅消息、发布成功等事件进行响应。

---

## 二、 硬件连接与引脚映射

基于 STM32F103C8T6 的资源分布，引脚映射如下：

| Air780e 模块引脚 | STM32F103C8T6 引脚 | 模式配置 | 功能说明 |
| --- | --- | --- | --- |
| **TXD** | **PB11** (USART3_RX) | 浮空/上拉输入 | 接收 4G 模块发送的 AT 回复或透传数据 |
| **RXD** | **PB10** (USART3_TX) | 推挽复用输出 | 发送 AT 指令或透传数据给 4G 模块 |
| **PWR** (开/关机)| **PB0** | 推挽输出 | 控制模块软开机/软关机 (拉高 1s 开机) |
| **RST** (硬件复位)| **PB1** | 推挽输出 | 控制模块硬件复位 (拉高 0.5s 复位) |
| **GND** | **GND** | 电源地 | 必须与 MCU 共地 |

---

## 三、 依赖配置 (STM32CubeMX)

使用本驱动前，必须在 STM32CubeMX 中完成以下底层配置：

1. **系统时钟**：配置系统时钟，确保 `HAL_GetTick()` 正常工作。
2. **GPIO**：将 `PB0` 和 `PB1` 设置为 `GPIO_Output`，推挽输出，默认电平设为 `Low`。
3. **USART3**：
   - 异步模式 (Asynchronous)，波特率 115200。
   - 开启 `USART3 global interrupt`。

由于驱动依赖 USART 的**空闲中断 (IDLE)** 接收不定长数据，必须在中断服务函数中挂载接收处理函数。在 `stm32f1xx_it.c` 中的修改如下：

```c
#include "Air780e.h"

void USART3_IRQHandler(void)
{
  if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET) {
      __HAL_UART_CLEAR_IDLEFLAG(&huart3);
      HAL_UART_AbortReceive_IT(&huart3);
      uint16_t rx_len = AIR780E_RX_BUF_SIZE - huart3.RxXferCount;
      Air780e_ReceiveProcess(rx_len);
  }
  HAL_UART_IRQHandler(&huart3);
}
```

---

## 四、 MQTT API 使用示例

### 1. 基础端到端测试验证

驱动内部提供了一个集成测试函数 `Air780e_TestRun()`，它将连接到公共测试服务器 `broker.emqx.io`，发布并订阅测试主题。

您只需在 `main.c` 中调用即可验证硬件和驱动是否正常工作：

```c
#include "Air780e.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART3_UART_Init(); 

    // 运行端到端测试验证框架
    Air780e_TestRun();
}
```

### 2. 生产环境业务代码示例

在实际项目中，请按以下流程初始化和使用：

```c
#include "Air780e.h"

// 1. 定义事件回调函数
void MyMqttCallback(Air780e_MqttEvent_t event, void *data) {
    if (event == AIR780E_MQTT_EVENT_CONNECTED) {
        // 连接成功，可以发布设备上线消息
        Air780e_MqttPublish("/device/status", "online", 1, 1);
    } else if (event == AIR780E_MQTT_EVENT_DATA_RECEIVED) {
        // 收到订阅的控制指令
        Air780e_MqttData_t *mqtt_data = (Air780e_MqttData_t *)data;
        printf("Topic: %s, Data: %s\r\n", mqtt_data->topic, mqtt_data->payload);
        
        // 执行具体业务逻辑，例如控制继电器...
    }
}

int main(void) {
    // ... 底层初始化 ...

    Air780e_Init();
    
    // 2. 配置 MQTT 核心连接参数
    Air780e_MqttSetConfig("broker.emqx.io", 1883, "my_stm32_client", "admin", "123456");
    
    // (可选) 3. 配置遗嘱消息 LWT
    Air780e_MqttSetLwt("/device/status", "offline", 1, 1);
    
    // 4. 配置订阅主题和回调
    Air780e_MqttSetCallback("/device/cmd", 0, MyMqttCallback);
    
    // 5. 模块开机
    Air780e_PowerOn();

    while (1) {
        // 6. 在主循环中持续轮询状态机 (非阻塞)
        Air780e_StateMachine();
        
        // ... 可以在此处执行其他任务，如传感器采集 ...
        static uint32_t last_time = 0;
        if (HAL_GetTick() - last_time > 10000) {
            // 每 10 秒发布一次传感器数据
            // 如果处于断网状态，消息将自动进入离线缓存队列
            Air780e_MqttPublish("/device/sensor", "{\"temp\": 25.5}", 0, 0);
            last_time = HAL_GetTick();
        }
    }
}
```

---

## 五、 常见问题及排查方案

### 1. 编译或链接错误？
- **检查**：确认已将 `Air780e.c` 添加到工程参与编译，并在用到 API 的文件中包含了 `Air780e.h`。

### 2. 无法连接到私有 MQTT Broker？
- **检查**：IP 域名和端口是否正确。如果是云服务器，请确认安全组是否放行了 1883 端口。
- **检查**：使用 SSCOM 串口助手手动发送 `AT+MIPSTART` 和 `AT+MCONNECT`，观察报错信息（如 `+CME ERROR`），以便确定是鉴权失败还是网络不通。

### 3. 离线缓存消息没有发送？
- **原因**：目前离线缓存深度为 `AIR780E_MQTT_CACHE_SIZE = 3`。断网期间如果发布了超过 3 条消息，最先进入缓存的消息会被丢弃，或导致新的消息无法入队。可以通过修改头文件中的宏定义来增加缓存深度，但请注意评估 STM32 的 RAM 消耗。
