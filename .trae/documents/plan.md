# 4G模块(Air780e) AT指令标准对齐与优化计划书

## 1. 摘要
本计划旨在根据提供的参考图片，全面梳理并重构 `Air780e` 驱动中的 AT 指令发送序列。通过对比当前代码与图片中的标准指令格式，修复在指令名称、参数引号、参数顺序及结束符等方面的差异，确保修改后的 AT 指令序列与参考图片完全一致，从而保证 4G 模块能够稳定、准确地连接到 MQTT 服务器。

## 2. 现状分析与差异对比
通过对比 `Air780e.c` 当前代码与参考图片，发现存在以下关键差异：

| 步骤 / 状态 | 参考图片中的标准指令格式 | 当前代码中的实现 | 差异分析 |
| :--- | :--- | :--- | :--- |
| **0. 波特率与保存** | `AT+IPR=115200;&W\r\n` | 缺失 | 当前代码仅发送了 `AT\r\n` 和 `ATE0\r\n`。 |
| **1. 模块复位** | `AT+RESET\r\n` (等待3500ms) | 缺失 | 当前代码无复位指令。 |
| **2. 信号质量** | `AT+CSQ\r\n` | 缺失 | 当前代码直接检查 SIM 卡 (`AT+CPIN?`)。 |
| **3. CCID获取** | `AT+CCID\r\n` | 缺失 | 当前代码缺失。 |
| **4. GPRS注册** | `AT+CGREG?\r\n` | 缺失 | 当前代码缺失。 |
| **5. GPRS附着** | `AT+CGATT?\r\n` | `AT+CGATT?\r\n` | 一致，均期望 `+CGATT: 1`。 |
| **6. MQTT配置** | `AT+MCONFIG=%s,%s,%s\r\n` | `AT+MCONFIG="%s","%s","%s"\r\n` | **重大差异**：当前代码为参数加了双引号，而图片中**无引号**。 |
| **7. MQTT连接(TCP)** | `AT+MIPSTART=%s,%d\r\n` | `AT+MIPSTART="%s",%d\r\n` | **重大差异**：当前代码为主机地址加了双引号，而图片中**无引号**；且需兼容 `ALREADY CONNECT` 响应。 |
| **8. MQTT会话** | `AT+MCONNECT=1,60\r\n` | `AT+MCONNECT=%d,%d\r\n` | **差异**：图片中硬编码为 `1,60`，当前代码为动态变量。 |
| **9. MQTT订阅** | `AT+MSUB="%s/time",1\r\n` | `AT+MSUB="%s",%d\r\n` | **差异**：图片中在主题后追加了 `/time` 且 QoS 固定为 `1`。 |

## 3. 计划修改内容 (Proposed Changes)

### 3.1 修改 `Air780e.h` (状态机枚举重构)
更新 `Air780e_State_t` 枚举，以严格匹配图片的 10 个步骤：
- `AIR780E_STATE_INIT_RESET` (对应 Case 0 & 1)
- `AIR780E_STATE_CHECK_CSQ` (对应 Case 2)
- `AIR780E_STATE_CHECK_CCID` (对应 Case 3)
- `AIR780E_STATE_CHECK_CGREG` (对应 Case 4)
- `AIR780E_STATE_CHECK_CGATT` (对应 Case 5)
- `AIR780E_STATE_MQTT_CONFIG` (对应 Case 6)
- `AIR780E_STATE_MQTT_MIPSTART` (对应 Case 7)
- `AIR780E_STATE_MQTT_MCONNECT` (对应 Case 8)
- `AIR780E_STATE_MQTT_MSUB` (对应 Case 9)
- `AIR780E_STATE_MQTT_CONNECTED` (对应 Case 10，正常工作状态)

### 3.2 修改 `Air780e.c` (状态机逻辑对齐)
在 `Air780e_StateMachine` 中，重写 `switch(s_ctx.state)` 逻辑：
1. **Case 0 & 1**: 发送 `AT+IPR=115200;&W\r\n`，期望 `OK`；成功后发送 `AT+RESET\r\n`，然后调用 `vTaskDelay(pdMS_TO_TICKS(3500))` 进行阻塞等待，随后进入下一状态。
2. **Case 2**: 发送 `AT+CSQ\r\n`，期望 `OK`。
3. **Case 3**: 发送 `AT+CCID\r\n`，期望 `OK`。
4. **Case 4**: 发送 `AT+CGREG?\r\n`，期望 `+CGREG: 0,1`。
5. **Case 5**: 发送 `AT+CGATT?\r\n`，期望 `+CGATT: 1`。
6. **Case 6**: 格式化指令 `AT+MCONFIG=%s,%s,%s\r\n` (严格**不加双引号**)，传入 ClientID、User、Password，期望 `OK`。
7. **Case 7**: 格式化指令 `AT+MIPSTART=%s,%d\r\n` (严格**不加双引号**)，期望 `CONNECT OK` 或 `ALREADY CONNECT`。
8. **Case 8**: 格式化指令 `AT+MCONNECT=1,60\r\n`，期望 `CONNACK OK`。
9. **Case 9**: 格式化指令 `AT+MSUB="%s/time",1\r\n`，传入配置的主题，期望 `SUBACK`。

### 3.3 修改 `Air780e_SendCmd` 支持双重期望响应
当前 `Air780e_SendCmd` 只支持单个期望回复。需修改或增加逻辑，使得 `AT+MIPSTART` 能够同时匹配 `"CONNECT OK"` 和 `"ALREADY CONNECT"`。

## 4. 假设与决策
- **强制对齐原则**：完全舍弃当前代码中动态拼接 QoS、KeepAlive 的逻辑，强制采用图片中的硬编码参数格式（如 `1,60` 和 QoS `1`），以满足“完全一致”的要求。
- **复位延时**：图片中 `AT+RESET` 后有一个 `CompareValue = 3500`。我们将在 RTOS 任务中使用 `vTaskDelay(3500)` 来模拟模块重启的耗时。

## 5. 验证与回归测试步骤
1. **静态代码检查**：核对所有 `sprintf` 格式化字符串，确认没有任何多余的双引号（`\"`）。
2. **运行与串口日志抓取**：
   - 观察 `[4G TX]` 输出的每一条指令，核对是否与图片列表中的 10 个 Case 一一对应。
   - 重点检查 `[4G TX]: AT+MCONFIG=...` 和 `[4G TX]: AT+MIPSTART=...` 是否符合无引号格式。
   - 检查 `[4G TX]: AT+MSUB=".../time",1` 的拼接结果。
3. **连通性测试**：确认模块最终能够打印 `[MQTT APP] Connect Success!` 且正常发布测试消息。