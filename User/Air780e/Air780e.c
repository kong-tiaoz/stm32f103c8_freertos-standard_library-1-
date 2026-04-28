/**
  ******************************************************************************
  * @file    Air780e.c
  * @author  Code Assistant
  * @version V2.0.0
  * @date    2024-xx-xx
  * @brief   Air780e 4G通信模块驱动实现文件 (MQTT 增强版)
  *          集成 MQTT 状态机、断线重连指数退避、离线缓存、事件回调与订阅管理。
  ******************************************************************************
  */

#include "Air780e.h"
#include "bsp_usart.h"

/* ----------------- 全局变量与上下文定义 ----------------- */

// 模块上下文实例
static Air780e_Context_t s_ctx = {0};

// 定义串口接收缓冲区
uint8_t g_Air780eRxBuffer[AIR780E_RX_BUF_SIZE] = {0};
uint16_t g_Air780eRxLen = 0;

// 定义接收标志位
static volatile bool s_RxFlag = false;

/* ----------------- 私有函数声明 ----------------- */

/**
 * @brief 发送 AT 指令并等待期望回复 (带超时机制和双重期望支持)
 */
static bool Air780e_SendCmd(const char *cmd, const char *expected_reply, const char *expected_reply2, uint32_t timeout_ms);

/**
 * @brief 清空接收缓冲区
 */
static void Air780e_ClearRxBuffer(void);

/**
 * @brief 离线消息缓存管理
 */
static void MqttCache_Init(void);
static bool MqttCache_Push(const char *topic, const char *payload, uint8_t qos, uint8_t retain);
static bool MqttCache_Pop(Air780e_MqttMsg_t *msg);
static void MqttCache_Flush(void);

/**
 * @brief 处理连接失败与指数退避
 */
static void HandleConnectionError(void);

/**
 * @brief 触发事件回调
 */
static void TriggerEvent(Air780e_MqttEvent_t event, void *data);

/* ----------------- 接口函数实现 ----------------- */

/**
 * @brief  初始化 Air780e 模块硬件接口
 */
void Air780e_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 开启外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // 2. 配置 PB0 (PWR) 和 PB1 (RST) 为推挽输出
    GPIO_InitStructure.GPIO_Pin = AIR780E_PWR_PIN | AIR780E_RST_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    AIR780E_PWR_OFF();
    AIR780E_RST_LOW();

    // 3. 配置 USART2 TX (PA2) 为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 4. 配置 USART2 RX (PA3) 为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 5. 配置 USART2 参数
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(AIR780E_UART, &USART_InitStructure);

    // 6. 配置中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; // 必须大于 configMAX_SYSCALL_INTERRUPT_PRIORITY 如果使用FreeRTOS API
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 7. 开启接收中断和空闲中断
    USART_ITConfig(AIR780E_UART, USART_IT_RXNE, ENABLE);
    USART_ITConfig(AIR780E_UART, USART_IT_IDLE, ENABLE);

    // 8. 使能串口
    USART_Cmd(AIR780E_UART, ENABLE);

    // 初始化状态机上下文
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = AIR780E_STATE_POWER_OFF;
    s_ctx.is_connected = false;
    s_ctx.backoff_delay_ms = 2000; // 初始退避延时 2 秒

    MqttCache_Init();
}

/**
 * @brief  设置 MQTT 核心连接参数
 */
void Air780e_MqttSetConfig(const char *broker, uint16_t port, const char *client_id, const char *username, const char *password)
{
    if (broker) strncpy(s_ctx.mqtt_cfg.broker, broker, sizeof(s_ctx.mqtt_cfg.broker) - 1);
    s_ctx.mqtt_cfg.port = port;
    if (client_id) strncpy(s_ctx.mqtt_cfg.client_id, client_id, sizeof(s_ctx.mqtt_cfg.client_id) - 1);
    if (username) strncpy(s_ctx.mqtt_cfg.username, username, sizeof(s_ctx.mqtt_cfg.username) - 1);
    if (password) strncpy(s_ctx.mqtt_cfg.password, password, sizeof(s_ctx.mqtt_cfg.password) - 1);
}

/**
 * @brief  设置 MQTT 遗嘱消息 (LWT)
 */
void Air780e_MqttSetLwt(const char *topic, const char *msg, uint8_t qos, uint8_t retain)
{
    s_ctx.mqtt_cfg.use_lwt = true;
    if (topic) strncpy(s_ctx.mqtt_cfg.lwt_topic, topic, sizeof(s_ctx.mqtt_cfg.lwt_topic) - 1);
    if (msg) strncpy(s_ctx.mqtt_cfg.lwt_msg, msg, sizeof(s_ctx.mqtt_cfg.lwt_msg) - 1);
    s_ctx.mqtt_cfg.lwt_qos = qos;
    s_ctx.mqtt_cfg.lwt_retain = retain;
}

/**
 * @brief  设置 MQTT 其他高级参数
 */
void Air780e_MqttSetAdvanced(bool use_tls, uint16_t keepalive, uint8_t clean_session)
{
    s_ctx.mqtt_cfg.use_tls = use_tls;
    s_ctx.mqtt_cfg.keepalive = keepalive;
    s_ctx.mqtt_cfg.clean_session = clean_session;
}

/**
 * @brief  设置初始默认订阅主题及事件回调
 */
void Air780e_MqttSetCallback(const char *sub_topic, uint8_t sub_qos, Air780e_MqttEventCallback_t cb)
{
    if (sub_topic) strncpy(s_ctx.mqtt_cfg.sub_topic, sub_topic, sizeof(s_ctx.mqtt_cfg.sub_topic) - 1);
    s_ctx.mqtt_cfg.sub_qos = sub_qos;
    s_ctx.mqtt_cfg.event_cb = cb;
}

/**
 * @brief  控制模块硬件开机
 */
void Air780e_PowerOn(void)
{
    AIR780E_PWR_ON();
    vTaskDelay(pdMS_TO_TICKS(1000));
    AIR780E_PWR_OFF();
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    
    s_ctx.state = AIR780E_STATE_INIT_RESET;
    s_ctx.last_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_ctx.error_count = 0;
    s_ctx.backoff_delay_ms = 2000;
    
    LOG_PRINT("[Air780e] Hardware Power On Executed.\r\n");
}

/**
 * @brief  控制模块硬件关机
 */
void Air780e_PowerOff(void)
{
    AIR780E_PWR_ON();
    vTaskDelay(pdMS_TO_TICKS(2000));
    AIR780E_PWR_OFF();
    
    s_ctx.state = AIR780E_STATE_POWER_OFF;
    if (s_ctx.is_connected) {
        s_ctx.is_connected = false;
        TriggerEvent(AIR780E_MQTT_EVENT_DISCONNECTED, NULL);
    }
}

/**
 * @brief  控制模块硬件复位
 */
void Air780e_HardwareReset(void)
{
    LOG_PRINT("[Air780e] Hardware Reset...\r\n");
    AIR780E_RST_HIGH();
    vTaskDelay(pdMS_TO_TICKS(500));
    AIR780E_RST_LOW();
    vTaskDelay(pdMS_TO_TICKS(2000)); 
    
    s_ctx.state = AIR780E_STATE_INIT_RESET;
    if (s_ctx.is_connected) {
        s_ctx.is_connected = false;
        TriggerEvent(AIR780E_MQTT_EVENT_DISCONNECTED, NULL);
    }
}

/**
 * @brief  清空接收缓冲区
 */
static void Air780e_ClearRxBuffer(void)
{
    memset(g_Air780eRxBuffer, 0, AIR780E_RX_BUF_SIZE);
    g_Air780eRxLen = 0;
    s_RxFlag = false;
}

/**
 * @brief 发送 AT 指令并等待期望回复
 */
static bool Air780e_SendCmd(const char *cmd, const char *expected_reply, const char *expected_reply2, uint32_t timeout_ms)
{
    uint32_t start_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    Air780e_ClearRxBuffer();
    
    if (cmd != NULL) {
        // 打印 4G 发送的指令
        LOG_PRINT("[4G TX]: %s", cmd);
        unsigned int len = strlen(cmd);
        for(unsigned int i = 0; i < len; i++) {
            USART_SendData(AIR780E_UART, (uint8_t)cmd[i]);
            while (USART_GetFlagStatus(AIR780E_UART, USART_FLAG_TXE) == RESET);
        }
    }
    
    if (expected_reply == NULL && expected_reply2 == NULL) return true;
    
    while (((xTaskGetTickCount() * portTICK_PERIOD_MS) - start_tick) < timeout_ms) {
        if (s_RxFlag) {
                // 默认情况下打印 4G 模块收到的数据（如 +CME ERROR, OK, SUBACK 等）
                LOG_PRINT("[4G RX]: %s\r\n", g_Air780eRxBuffer);
                
                bool match1 = (expected_reply != NULL) && (strstr((const char*)g_Air780eRxBuffer, expected_reply) != NULL);
            bool match2 = (expected_reply2 != NULL) && (strstr((const char*)g_Air780eRxBuffer, expected_reply2) != NULL);
            
            if (match1 || match2) {
                return true;
            }
            
            // 捕获致命错误（如模块重启、服务端断开、MQTT无连接）
            if (strstr((const char*)g_Air780eRxBuffer, "RDY") != NULL ||
                strstr((const char*)g_Air780eRxBuffer, "CLOSED") != NULL ||
                strstr((const char*)g_Air780eRxBuffer, "DISCONNECT") != NULL ||
                strstr((const char*)g_Air780eRxBuffer, "+CME ERROR: 767") != NULL) {
                
                LOG_PRINT("[Air780e] Fatal error detected during command: %s\r\n", g_Air780eRxBuffer);
                if (s_ctx.is_connected || s_ctx.state > AIR780E_STATE_INIT_RESET) {
                    HandleConnectionError();
                }
                return false;
            }
            
            s_RxFlag = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

/**
 * @brief 处理连接失败与指数退避
 */
static void HandleConnectionError(void)
{
    s_ctx.error_count++;
    LOG_PRINT("[Air780e] Error count: %d. Entering backoff...\r\n", s_ctx.error_count);
    
    if (s_ctx.is_connected) {
        s_ctx.is_connected = false;
        TriggerEvent(AIR780E_MQTT_EVENT_DISCONNECTED, NULL);
    }
    
    if (s_ctx.error_count > 5) {
        // 多次重试失败，硬件复位
        LOG_PRINT("[Air780e] Too many errors, hardware reset.\r\n");
        Air780e_HardwareReset();
    } else {
        // 指数退避计算
        s_ctx.backoff_delay_ms *= 2;
        if (s_ctx.backoff_delay_ms > 60000) s_ctx.backoff_delay_ms = 60000; // 最大 60 秒
        s_ctx.error_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;
        s_ctx.state = AIR780E_STATE_WAIT_BACKOFF;
        LOG_PRINT("[Air780e] Will retry after %d ms.\r\n", s_ctx.backoff_delay_ms);
    }
}

/**
 * @brief  Air780e 核心非阻塞状态机处理函数
 */
void Air780e_StateMachine(void)
{
    char cmd_buf[256];
    
    switch (s_ctx.state) {
        
        case AIR780E_STATE_POWER_OFF:
            break;
            
        case AIR780E_STATE_INIT_RESET:
            if (Air780e_SendCmd("AT+IPR=115200;&W\r\n", "OK", NULL, 1000)) {
                Air780e_SendCmd("AT+RESET\r\n", NULL, NULL, 1000);
                vTaskDelay(pdMS_TO_TICKS(3500)); // 模块重启需等待
                s_ctx.state = AIR780E_STATE_CHECK_CSQ;
                s_ctx.error_count = 0;
            } else {
                HandleConnectionError();
            }
            break;
            
        case AIR780E_STATE_CHECK_CSQ:
            if (Air780e_SendCmd("AT+CSQ\r\n", "OK", NULL, 2000)) {
                s_ctx.state = AIR780E_STATE_CHECK_CCID;
                s_ctx.error_count = 0;
            } else {
                HandleConnectionError();
            }
            break;
            
        case AIR780E_STATE_CHECK_CCID:
            if (Air780e_SendCmd("AT+CCID\r\n", "OK", NULL, 2000)) {
                s_ctx.state = AIR780E_STATE_CHECK_CGREG;
                s_ctx.error_count = 0;
            } else {
                HandleConnectionError();
            }
            break;
            
        case AIR780E_STATE_CHECK_CGREG:
            if (Air780e_SendCmd("AT+CGREG?\r\n", "+CGREG: 0,1", NULL, 3000)) {
                s_ctx.state = AIR780E_STATE_CHECK_CGATT;
                s_ctx.error_count = 0;
            } else {
                HandleConnectionError();
            }
            break;
            
        case AIR780E_STATE_CHECK_CGATT:
            if (Air780e_SendCmd("AT+CGATT?\r\n", "+CGATT: 1", NULL, 2000)) {
                s_ctx.state = AIR780E_STATE_MQTT_CONFIG;
                s_ctx.error_count = 0;
                LOG_PRINT("[Air780e] Network Attached.\r\n");
            } else {
                HandleConnectionError();
            }
            break;
            
        case AIR780E_STATE_MQTT_CONFIG:
            // 每次重新配置前，先尝试断开旧的 MQTT 和 TCP 连接，防止 "ALREADY CONNECT" 导致的 +CME ERROR 767 死锁
            Air780e_SendCmd("AT+MDISCONNECT\r\n", NULL, NULL, 500);
            Air780e_SendCmd("AT+MIPCLOSE\r\n", NULL, NULL, 500);
            
            // 拼接 MCONFIG 指令，严格无引号格式
            if (s_ctx.mqtt_cfg.use_lwt) {
                sprintf(cmd_buf, "AT+MCONFIG=%s,%s,%s,0,%d,%d,\"%s\",\"%s\"\r\n", 
                        s_ctx.mqtt_cfg.client_id, s_ctx.mqtt_cfg.username, s_ctx.mqtt_cfg.password,
                        s_ctx.mqtt_cfg.lwt_qos, s_ctx.mqtt_cfg.lwt_retain, 
                        s_ctx.mqtt_cfg.lwt_topic, s_ctx.mqtt_cfg.lwt_msg);
            } else {
                sprintf(cmd_buf, "AT+MCONFIG=%s,%s,%s\r\n", 
                        s_ctx.mqtt_cfg.client_id, s_ctx.mqtt_cfg.username, s_ctx.mqtt_cfg.password);
            }
            
            if (Air780e_SendCmd(cmd_buf, "OK", NULL, 2000)) {
                s_ctx.state = AIR780E_STATE_MQTT_MIPSTART;
                s_ctx.error_count = 0;
            } else {
                LOG_PRINT("[Air780e] MQTT Config Failed.\r\n");
                HandleConnectionError();
            }
            break;
            
        case AIR780E_STATE_MQTT_MIPSTART:
            if (s_ctx.mqtt_cfg.broker[0] != '\0' && s_ctx.mqtt_cfg.port > 0) {
                // 发起 TCP 握手，严格无引号格式
                sprintf(cmd_buf, "AT+MIPSTART=%s,%d\r\n", s_ctx.mqtt_cfg.broker, s_ctx.mqtt_cfg.port);
                LOG_PRINT("[Air780e] MQTT Connecting to %s:%d...\r\n", s_ctx.mqtt_cfg.broker, s_ctx.mqtt_cfg.port);
                
                // 允许 "CONNECT OK" 或 "ALREADY CONNECT"
                if (Air780e_SendCmd(cmd_buf, "CONNECT OK", "ALREADY CONNECT", 5000)) {
                    s_ctx.state = AIR780E_STATE_MQTT_MCONNECT;
                    s_ctx.error_count = 0;
                } else {
                    LOG_PRINT("[Air780e] MQTT MIPSTART Failed.\r\n");
                    HandleConnectionError();
                }
            } else {
                s_ctx.state = AIR780E_STATE_ERROR;
            }
            break;

        case AIR780E_STATE_MQTT_MCONNECT:
            // 发起 MQTT 握手，硬编码为 1,60 以完全匹配要求
            sprintf(cmd_buf, "AT+MCONNECT=1,60\r\n");
            
            if (Air780e_SendCmd(cmd_buf, "CONNACK OK", NULL, 5000)) {
                s_ctx.state = AIR780E_STATE_MQTT_MSUB;
                s_ctx.error_count = 0;
            } else {
                LOG_PRINT("[Air780e] MQTT MCONNECT Failed.\r\n");
                HandleConnectionError();
            }
            break;
            
        case AIR780E_STATE_MQTT_MSUB:
            // 默认订阅，按图片追加 /time，并使用 QoS=1
            if (s_ctx.mqtt_cfg.sub_topic[0] != '\0') {
                sprintf(cmd_buf, "AT+MSUB=\"%s/time\",1\r\n", s_ctx.mqtt_cfg.sub_topic);
                if (Air780e_SendCmd(cmd_buf, "SUBACK", NULL, 3000)) {
                    LOG_PRINT("[Air780e] Default Subscription Success.\r\n");
                } else {
                    LOG_PRINT("[Air780e] Default Subscription Failed, ignoring.\r\n");
                }
            }
            
            // 状态完全就绪，允许发布消息
            s_ctx.state = AIR780E_STATE_MQTT_CONNECTED;
            s_ctx.is_connected = true;
            s_ctx.backoff_delay_ms = 2000; // 成功连接后重置退避延时
            s_ctx.heartbeat_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            LOG_PRINT("[Air780e] MQTT Connected Successfully.\r\n");
            
            // 刷新离线缓存队列
            MqttCache_Flush();
            
            // 触发回调，通知应用层可以发布消息了
            TriggerEvent(AIR780E_MQTT_EVENT_CONNECTED, NULL);
            break;
            
        case AIR780E_STATE_MQTT_CONNECTED:
            // 处理接收数据 (被动 URC)
            if (s_RxFlag) {
                char *rx_str = (char*)g_Air780eRxBuffer;
                // 打印异步收到的 URC 和服务端下发数据
                LOG_PRINT("[4G URC RX]: %s\r\n", rx_str);
                
                // 检查是否断开或重启
                if (strstr(rx_str, "CLOSED") != NULL || 
                    strstr(rx_str, "DISCONNECT") != NULL || 
                    strstr(rx_str, "RDY") != NULL ||
                    strstr(rx_str, "+CME ERROR: 767") != NULL) {
                    LOG_PRINT("[Air780e] MQTT Disconnected or Module Rebooted.\r\n");
                    HandleConnectionError();
                } 
                // 解析 MQTT 下发消息 (+MSUB: "topic",len byte,payload)
                else {
                    char *msub_ptr = strstr(rx_str, "+MSUB:");
                    if (msub_ptr != NULL) {
                        Air780e_MqttData_t mqtt_data;
                        memset(&mqtt_data, 0, sizeof(mqtt_data));
                        
                        // 简单解析，提取 topic 和 payload (忽略长度解析的严格校验)
                        char *topic_start = strchr(msub_ptr, '"');
                        if (topic_start) {
                            topic_start++;
                            char *topic_end = strchr(topic_start, '"');
                            if (topic_end) {
                                int topic_len = topic_end - topic_start;
                                if (topic_len < sizeof(mqtt_data.topic)) {
                                    strncpy(mqtt_data.topic, topic_start, topic_len);
                                }
                                
                                // 查找 "byte," 之后的数据即为 payload
                                char *payload_start = strstr(topic_end, "byte,");
                                if (payload_start) {
                                    payload_start += 5; // 移动到 payload 开头
                                    // 剩余字符串作为 payload
                                    strncpy((char*)mqtt_data.payload, payload_start, sizeof(mqtt_data.payload) - 1);
                                    mqtt_data.payload_len = strlen((char*)mqtt_data.payload);
                                    
                                    // 触发数据回调
                                    TriggerEvent(AIR780E_MQTT_EVENT_DATA_RECEIVED, &mqtt_data);
                                }
                            }
                        }
                    }
                }
                Air780e_ClearRxBuffer();
            }
            
            // 定期保活 (心跳)
            // 注：MCONNECT 本身由模块维持心跳，这里加入一层应用层保障，若需要发 PINGREQ 则调用，但 Air780e 固件通常自动处理心跳。
            // 这里只需维护超时重启等。
            break;
            
        case AIR780E_STATE_WAIT_BACKOFF:
            if ((xTaskGetTickCount() * portTICK_PERIOD_MS) - s_ctx.error_tick > s_ctx.backoff_delay_ms) {
                s_ctx.state = AIR780E_STATE_CHECK_CSQ; // 从检查信号开始重新尝试
                LOG_PRINT("[Air780e] Backoff done, retrying...\r\n");
            }
            break;
            
        case AIR780E_STATE_ERROR:
            // 严重错误，10秒后硬件复位
            if ((xTaskGetTickCount() * portTICK_PERIOD_MS) - s_ctx.last_tick > 10000) {
                Air780e_HardwareReset();
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief  获取当前 MQTT 连接状态
 */
bool Air780e_IsConnected(void)
{
    return s_ctx.is_connected;
}

/**
 * @brief  发布 MQTT 消息
 */
bool Air780e_MqttPublish(const char *topic, const char *payload, uint8_t qos, uint8_t retain)
{
    if (s_ctx.is_connected && s_ctx.state >= AIR780E_STATE_MQTT_CONNECTED) {
        char cmd_buf[256];
        // 拼接 AT+MPUB 指令
        sprintf(cmd_buf, "AT+MPUB=\"%s\",%d,%d,\"%s\"\r\n", topic, qos, retain, payload);
        
        // 由于 QoS=1 时，模块需要等待服务端的 PUBACK，这个过程可能会长达几秒钟
        // 如果把超时设为 3000ms 可能不足以等到 OK 后的 PUBACK，如果还没等到就提前退出并判定失败
        // 会导致应用层立刻重发，从而顶掉前一个连接引发 +CME ERROR 767
        // 我们把发布指令超时提升至 5000ms，以确保它有充足的时间获取到发送确认
        if (Air780e_SendCmd(cmd_buf, "OK", NULL, 5000)) {
            TriggerEvent(AIR780E_MQTT_EVENT_PUBLISHED, NULL);
            return true;
        }
        return false;
    } else {
        // 未连接，尝试存入离线缓存
        LOG_PRINT("[Air780e] Offline, caching message...\r\n");
        if (MqttCache_Push(topic, payload, qos, retain)) {
            return true;
        } else {
            LOG_PRINT("[Air780e] Cache full, message dropped.\r\n");
            return false;
        }
    }
}

/**
 * @brief  动态订阅 MQTT 主题
 */
bool Air780e_MqttSubscribe(const char *topic, uint8_t qos)
{
    if (!s_ctx.is_connected) return false;
    
    char cmd_buf[128];
    sprintf(cmd_buf, "AT+MSUB=\"%s/time\",1\r\n", topic);
    return Air780e_SendCmd(cmd_buf, "SUBACK", NULL, 3000);
}

/**
 * @brief  串口2中断服务函数
 */
void USART2_IRQHandler(void)
{
    volatile uint16_t sr = AIR780E_UART->SR;
    
    // 检查是否有接收中断或错误中断 (ORE, NE, FE, PE)
    if(USART_GetITStatus(AIR780E_UART, USART_IT_RXNE) != RESET ||
       (sr & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)) != 0)
    {
        // 读取 DR 会同时清除 RXNE 标志和绝大多数硬件错误标志
        uint8_t ch = USART_ReceiveData(AIR780E_UART);
        
        // 仅在无错误的情况下保存数据
        if ((sr & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)) == 0) {
            if(g_Air780eRxLen < AIR780E_RX_BUF_SIZE - 1)
            {
                g_Air780eRxBuffer[g_Air780eRxLen++] = ch;
            }
        }
    }
    
    // 空闲中断，代表一帧数据接收完成
    if(USART_GetITStatus(AIR780E_UART, USART_IT_IDLE) != RESET)
    {
        // 软件清除 IDLE 标志位: 先读 SR, 再读 DR
        volatile uint32_t temp = AIR780E_UART->SR;
        temp = AIR780E_UART->DR;
        (void)temp;
        
        g_Air780eRxBuffer[g_Air780eRxLen] = '\0';
        s_RxFlag = true;
    }
}

/* ----------------- 私有辅助函数实现 ----------------- */

/**
 * @brief 初始化离线缓存
 */
static void MqttCache_Init(void)
{
    s_ctx.cache.head = 0;
    s_ctx.cache.tail = 0;
    s_ctx.cache.count = 0;
    memset(s_ctx.cache.msgs, 0, sizeof(s_ctx.cache.msgs));
}

/**
 * @brief 压入离线缓存
 */
static bool MqttCache_Push(const char *topic, const char *payload, uint8_t qos, uint8_t retain)
{
    if (s_ctx.cache.count >= AIR780E_MQTT_CACHE_SIZE) return false;
    
    Air780e_MqttMsg_t *msg = &s_ctx.cache.msgs[s_ctx.cache.tail];
    strncpy(msg->topic, topic, sizeof(msg->topic)-1);
    strncpy(msg->payload, payload, sizeof(msg->payload)-1);
    msg->qos = qos;
    msg->retain = retain;
    msg->valid = true;
    
    s_ctx.cache.tail = (s_ctx.cache.tail + 1) % AIR780E_MQTT_CACHE_SIZE;
    s_ctx.cache.count++;
    return true;
}

/**
 * @brief 弹出离线缓存
 */
static bool MqttCache_Pop(Air780e_MqttMsg_t *msg)
{
    if (s_ctx.cache.count == 0) return false;
    
    *msg = s_ctx.cache.msgs[s_ctx.cache.head];
    s_ctx.cache.msgs[s_ctx.cache.head].valid = false;
    
    s_ctx.cache.head = (s_ctx.cache.head + 1) % AIR780E_MQTT_CACHE_SIZE;
    s_ctx.cache.count--;
    return true;
}

/**
 * @brief 刷新发送离线缓存中的数据
 */
static void MqttCache_Flush(void)
{
    Air780e_MqttMsg_t msg;
    while (MqttCache_Pop(&msg)) {
        LOG_PRINT("[Air780e] Flushing cached msg to topic %s\r\n", msg.topic);
        Air780e_MqttPublish(msg.topic, msg.payload, msg.qos, msg.retain);
        vTaskDelay(pdMS_TO_TICKS(100)); // 防止连续发送过快
    }
}

/**
 * @brief 触发回调事件并记录日志
 */
static void TriggerEvent(Air780e_MqttEvent_t event, void *data)
{
    // 日志记录
    switch (event) {
        case AIR780E_MQTT_EVENT_CONNECTED:
            LOG_PRINT("[Air780e Event] Connected to Broker.\r\n");
            break;
        case AIR780E_MQTT_EVENT_DISCONNECTED:
            LOG_PRINT("[Air780e Event] Disconnected.\r\n");
            break;
        case AIR780E_MQTT_EVENT_ERROR:
            LOG_PRINT("[Air780e Event] Error Occurred.\r\n");
            break;
        default:
            break;
    }
    
    // 执行用户回调
    if (s_ctx.mqtt_cfg.event_cb) {
        s_ctx.mqtt_cfg.event_cb(event, data);
    }
}

/* ----------------- 测试与验证框架 ----------------- */

/**
 * @brief  测试事件回调函数
 */
static void TestMqttCallback(Air780e_MqttEvent_t event, void *data)
{
    switch (event) {
        case AIR780E_MQTT_EVENT_CONNECTED:
            LOG_PRINT("[Test] MQTT Connected! Sending test message...\r\n");
            Air780e_MqttPublish("/test/air780e/pub", "Hello from Air780e!", 0, 0);
            break;
            
        case AIR780E_MQTT_EVENT_DATA_RECEIVED: {
            Air780e_MqttData_t *mqtt_data = (Air780e_MqttData_t *)data;
            LOG_PRINT("[Test] Received MSG on topic: %s\r\n", mqtt_data->topic);
            LOG_PRINT("[Test] Payload: %s\r\n", mqtt_data->payload);
            
            // 收到特定消息时测试自动回复
            if (strstr((char*)mqtt_data->payload, "ping") != NULL) {
                Air780e_MqttPublish("/test/air780e/pub", "pong", 0, 0);
            }
            break;
        }
        default:
            break;
    }
}

/**
 * @brief  运行一次测试框架，用于端到端验证
 * @note   阻塞函数，仅用于测试目的。正式环境请将配置和状态机放入 main.c。
 */
void Air780e_TestRun(void)
{
    LOG_PRINT("\r\n================ Air780e MQTT Test Start ================\r\n");
    
    // 1. 初始化
    Air780e_Init();
    
    // 2. 配置 MQTT 核心参数 (示例使用公共测试 broker，用户需替换为自己的)
    Air780e_MqttSetConfig("broker.emqx.io", 1883, "air780e_test_client", "", "");
    
    // 3. 配置高级参数与回调
    Air780e_MqttSetAdvanced(false, 60, 1);
    Air780e_MqttSetCallback("/test/air780e/sub", 0, TestMqttCallback);
    
    // 4. 测试离线缓存：在未连接前发布消息
    LOG_PRINT("[Test] Testing offline cache...\r\n");
    Air780e_MqttPublish("/test/air780e/pub", "This is an offline cached message.", 0, 0);
    
    // 5. 开机
    Air780e_PowerOn();
    
    // 6. 进入主循环状态机
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while (1) {
        Air780e_StateMachine();
        
        // 测试心跳：每隔30秒打印一次状态
        if ((xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time > 30000) {
            LOG_PRINT("[Test] Alive tick. Connected: %d\r\n", Air780e_IsConnected());
            start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 释放CPU
    }
}
