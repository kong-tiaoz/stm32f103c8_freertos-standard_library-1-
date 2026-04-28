#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "queue.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "semphr.h"
#include "Air780e.h"
#include "bsp_usart.h"

TaskHandle_t mqttTaskHandle;
QueueHandle_t mqttPublishQueue = NULL; // 用于缓存真实 GPS 数据以供 MQTT 任务发布

/* ================== GPS 模块 (TAU1201) 配置与处理 ================== */
#define GPS_USART             USART3
#define GPS_USART_CLK         RCC_APB1Periph_USART3
#define GPS_USART_GPIO_CLK    RCC_APB2Periph_GPIOB
#define GPS_USART_TX_PIN      GPIO_Pin_10
#define GPS_USART_RX_PIN      GPIO_Pin_11
#define GPS_USART_GPIO_PORT   GPIOB
#define GPS_USART_IRQn        USART3_IRQn

static QueueHandle_t gpsQueue = NULL;
static TaskHandle_t gpsTaskHandle = NULL;

// 添加用于同步的信号量
static SemaphoreHandle_t gpsCaptureSync = NULL;

const uint8_t close_gll[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};
const uint8_t close_gsa[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};
const uint8_t close_grs[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
const uint8_t close_gsv[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17};
const uint8_t close_rmc[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19};
const uint8_t close_vtg[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x06, 0x00, 0x00, 0x1B};
const uint8_t close_zda[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x07, 0x00, 0x01, 0x1D};
const uint8_t close_gst[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x08, 0x00, 0x02, 0x1F};
const uint8_t close_txt[]     = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x40, 0x00, 0x3A, 0x8F};
const uint8_t close_txt_ant[] = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x20, 0x00, 0x1A, 0x4F};

static void GPS_SendCmd(const uint8_t* cmd, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        USART_SendData(GPS_USART, cmd[i]);
        while (USART_GetFlagStatus(GPS_USART, USART_FLAG_TXE) == RESET);
    }
}

static void GPS_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(GPS_USART_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(GPS_USART_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPS_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPS_USART_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPS_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPS_USART_GPIO_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200; // 根据要求重新配置为 115200 bps 以进行逻辑分析仪测试
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(GPS_USART, &USART_InitStructure);

    // 提升 NVIC 中断优先级以确保串口高速接收不丢包
    NVIC_InitStructure.NVIC_IRQChannel = GPS_USART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; // 设为5，高于串口2（一般设为6）且需大于 configMAX_SYSCALL_INTERRUPT_PRIORITY
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(GPS_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(GPS_USART, ENABLE);
    
    if (gpsQueue == NULL) {
        // 创建队列，增加队列深度以应对 115200 高速数据流
        gpsQueue = xQueueCreate(512, sizeof(uint8_t));
    }
}

static void GPS_Disable(void) {
    USART_Cmd(GPS_USART, DISABLE);
    USART_ITConfig(GPS_USART, USART_IT_RXNE, DISABLE);
}

static void GPS_Enable(void) {
    // 每次重新使能前，先彻底清空可能残留的数据和标志位
    volatile uint16_t sr = GPS_USART->SR;
    volatile uint16_t dr = GPS_USART->DR;
    (void)sr; (void)dr;
    
    if (gpsQueue != NULL) {
        xQueueReset(gpsQueue);
    }
    
    USART_ITConfig(GPS_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(GPS_USART, ENABLE);
}

static void TAU1201_Config(void) {
    // 延迟等待模块启动
    vTaskDelay(pdMS_TO_TICKS(500));
    GPS_SendCmd(close_gll, sizeof(close_gll));
    vTaskDelay(pdMS_TO_TICKS(50));
    GPS_SendCmd(close_gsa, sizeof(close_gsa));
    vTaskDelay(pdMS_TO_TICKS(50));
    GPS_SendCmd(close_grs, sizeof(close_grs));
    vTaskDelay(pdMS_TO_TICKS(50));
    GPS_SendCmd(close_gsv, sizeof(close_gsv));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 发送关闭 RMC 语句的指令，确保仅输出 GGA
    GPS_SendCmd(close_rmc, sizeof(close_rmc));
    vTaskDelay(pdMS_TO_TICKS(50));
    
    GPS_SendCmd(close_vtg, sizeof(close_vtg));
    vTaskDelay(pdMS_TO_TICKS(50));
    GPS_SendCmd(close_zda, sizeof(close_zda));
    vTaskDelay(pdMS_TO_TICKS(50));
    GPS_SendCmd(close_gst, sizeof(close_gst));
    vTaskDelay(pdMS_TO_TICKS(50));
    GPS_SendCmd(close_txt, sizeof(close_txt));
    vTaskDelay(pdMS_TO_TICKS(50));
    GPS_SendCmd(close_txt_ant, sizeof(close_txt_ant));
    vTaskDelay(pdMS_TO_TICKS(50));
}

void USART3_IRQHandler(void) {
    // 读取 SR 寄存器状态
    volatile uint16_t sr = GPS_USART->SR;
    
    // 检查是否有接收中断或错误中断 (ORE, NE, FE, PE)
    if(USART_GetITStatus(GPS_USART, USART_IT_RXNE) != RESET ||
       (sr & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)) != 0) {
        
        // 读取 DR 会同时清除 RXNE 标志和绝大多数硬件错误标志 (ORE, NE, FE, PE)
        uint8_t ch = USART_ReceiveData(GPS_USART);
        
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        
        // 只有当没有发生任何帧错误、噪声错误或溢出错误时，才将有效数据压入队列
        if ((sr & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)) == 0) {
            if (gpsQueue != NULL) {
                xQueueSendFromISR(gpsQueue, &ch, &xHigherPriorityTaskWoken);
            }
        } else {
            // 发生错误时，这里仅丢弃该字节，硬件标志位已通过前面的 SR -> DR 读取序列被安全清除
            // 避免了 MCU 陷入无限中断循环 (Hard Fault 或 Freeze)
        }
        
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * 将原始 NMEA GGA 语句解析并格式化为指定协议的报文。
 * 格式：设备唯一标识符（ID）,经度坐标,纬度坐标,高程信息,模式状态,电压值
 */
static bool format_gps_data(const char* raw_gga, char* out_buf, size_t out_len) {
    char temp[128];
    strncpy(temp, raw_gga, sizeof(temp)-1);
    temp[sizeof(temp)-1] = '\0';
    
    char* fields[20];
    int count = 0;
    
    // NMEA 是逗号分隔的，使用手动分割保留空字段
    char* p = temp;
    fields[count++] = p;
    while (*p && count < 20) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
        p++;
    }
    
    // 确保是 GGA 报文且字段数足够 (至少包含到 altitude 的第9个字段，通常有14个)
    if (count < 10 || strstr(fields[0], "GGA") == NULL) {
        return false;
    }
    
    const char* lat_str = fields[2];
    const char* lon_str = fields[4];
    const char* mode_str = fields[6];
    const char* alt_str = fields[9];
    
    // 未定位时，经纬度字段可能为空，自动填充 0.0 或 0 以符合格式要求
    if (lat_str[0] == '\0') lat_str = "0.0";
    if (lon_str[0] == '\0') lon_str = "0.0";
    if (mode_str[0] == '\0') mode_str = "0";
    if (alt_str[0] == '\0') alt_str = "0.0";
    
    double lat = atof(lat_str);
    double lon = atof(lon_str);
    int mode = atoi(mode_str);
    
    // 数据完整性校验：验证数据类型和取值范围
    // 纬度范围 0~9000 (ddmm.mmmm 格式)
    if (lat < 0.0 || lat > 9000.0) return false;
    // 经度范围 0~18000 (dddmm.mmmm 格式)
    if (lon < 0.0 || lon > 18000.0) return false;
    // 模式状态 0~9
    if (mode < 0 || mode > 9) return false;
    
    // 固定的字段
    const char* device_id = "BEIDOU_CLIENT_001";
    const char* voltage_str = "3.3";
    
    // 拼接重构格式
    snprintf(out_buf, out_len, "%s,%s,%s,%s,%s,%s", 
             device_id, lon_str, lat_str, alt_str, mode_str, voltage_str);
             
    return true;
}

static void task_gps_forward(void *arg) {
    uint8_t ch;
    char latest_gga[128] = {0};
    int gga_idx = 0;
    
    GPS_Init();

    LOG_PRINT("[GPS] Device powered on. Waiting 5 seconds for system stabilization...\r\n");
    
    // 1. 上电后强制等待 5 秒钟的稳定期，给 GPS 模块充足时间启动
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // 2. 5秒后，立即发送配置指令关闭所有非必要数据输出，仅保留 GGA
    LOG_PRINT("[GPS] 5s elapsed. Configuring TAU1201 (Keep ONLY GGA)...\r\n");
    TAU1201_Config();
    LOG_PRINT("[GPS] TAU1201 configured to output ONLY GGA.\r\n");
    
    // 清空配置期间积压在队列中的多余数据，确保转发通道干净
    xQueueReset(gpsQueue);

    // 3. 等待 4G 模块初始化完成并连接至服务器
    LOG_PRINT("[GPS] Waiting for 4G module to be fully initialized and connected...\r\n");
    while (!Air780e_IsConnected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // 4. 4G 初始化完成后，记录状态
    LOG_PRINT("[GPS] 4G Connected! Maintaining debug logs for AT command monitoring.\r\n");
    vTaskDelay(pdMS_TO_TICKS(100)); 
    // 不再关闭全局日志，保留 AT 指令和系统状态的打印
    // g_log_output_enabled = false;   

    // 释放信号量，允许第一次采集
    if (gpsCaptureSync != NULL) {
        xSemaphoreGive(gpsCaptureSync);
    }

    // 5. 建立纯净数据接收通道并缓存以供 MQTT 上发
    while(1) {
        // 阻塞等待采集许可信号（由 4G 发送成功后释放）
        if (xSemaphoreTake(gpsCaptureSync, portMAX_DELAY) == pdTRUE) {
            // 重新使能串口三，开始接收最新数据
            GPS_Enable();
            
            bool gga_collected = false;
            
            // 采集一轮（仅 GGA）
            while(!gga_collected) {
                if (xQueueReceive(gpsQueue, &ch, portMAX_DELAY) == pdTRUE) {
                    if (ch == '$') {
                        gga_idx = 0;
                    }
                    if (gga_idx < sizeof(latest_gga) - 1) {
                        latest_gga[gga_idx++] = ch;
                        latest_gga[gga_idx] = '\0';
                    }
                    
                    if (ch == '\n' || ch == '\r') {
                        if (gga_idx > 5) { 
                            bool is_gga = (strstr(latest_gga, "GGA") != NULL);
                            
                            if (is_gga) {
                                char *rn = strpbrk(latest_gga, "\r\n");
                                if (rn) *rn = '\0';
                                
                                if (mqttPublishQueue != NULL) {
                                    char formatted_buf[256];
                                    // 进行数据提取、校验并重构格式
                                    if (format_gps_data(latest_gga, formatted_buf, sizeof(formatted_buf))) {
                                        // 校验通过后，送入发布队列
                                        xQueueSend(mqttPublishQueue, formatted_buf, 0);
                                    } else {
                                        // 校验失败或数据非法，直接丢弃该包
                                    }
                                }
                                
                                gga_collected = true;
                            }
                        }
                        gga_idx = 0; 
                        latest_gga[0] = '\0';
                    }
                }
            }
            
            // 一轮采集完成，立刻关闭串口三，避免堆积和中断开销
            GPS_Disable();
            
            // 数据已进入 mqttPublishQueue，等待 task_mqtt_app 发送完成并再次释放 gpsCaptureSync
        }
    }
}

/* ================== MQTT 事件回调函数 ================== */
static void MyMqttCallback(Air780e_MqttEvent_t event, void *data)
{
    // 在此屏蔽或精简回调中的日志输出，避免在透传模式下污染串口一的 GPS 数据流
    switch (event) {
        case AIR780E_MQTT_EVENT_CONNECTED:
            // 首次连接成功后，在 task_gps_forward 中会执行关闭全局打印的动作
            break;
        case AIR780E_MQTT_EVENT_DISCONNECTED:
            // 断线时，可选择性通过其他端口(如 UART2/LED) 提示
            break;
        case AIR780E_MQTT_EVENT_DATA_RECEIVED:
            LED1_TOGGLE; // 仅保留 LED 翻转提示
            break;
        default:
            break;
    }
}

/* ================== MQTT 主任务 ================== */
void task_mqtt_app(void *arg)
{
    LOG_PRINT("[MQTT Task] Initializing Air780e...\r\n");
    
    // 1. 初始化 4G 模块硬件接口 (串口、引脚)
    Air780e_Init();
    
    // 2. 配置服务器连接参数
    Air780e_MqttSetConfig("8.136.109.126", 1883, "BEIDOU_CLIENT_001", "mosquitto", "hfut");
    
    // 3. 配置高级参数
    Air780e_MqttSetAdvanced(false, 60, 1);
    
    // 4. 配置事件回调与默认订阅主题
    Air780e_MqttSetCallback("/BEIDOU001/cmd", 0, MyMqttCallback);
    
    // 5. 硬件开机
    Air780e_PowerOn();

    LOG_PRINT("[MQTT Task] Entering State Machine loop...\r\n");
    
    char publish_buf[256];

    while(1)
    {
        // 持续轮询 4G 模块状态机，处理 AT 指令收发、网络检查、断线重连
        Air780e_StateMachine();

        // 不论是否连网，只要队列里有数据就取出来。如果没连网，会在 Air780e_MqttPublish 内部转存到离线缓存
        if (xQueueReceive(mqttPublishQueue, publish_buf, 0) == pdTRUE) {
            bool success = false;
            if (Air780e_IsConnected()) {
                // 执行异常重发机制：尝试发送 3 次
                for (int retry = 0; retry < 3; retry++) {
                    // AT+MPUB 指令在 Air780e.c 中阻塞超时为 5000ms
                    if (Air780e_MqttPublish("/BEIDOU001/datas", publish_buf, 1, 0)) {
                        success = true;
                        
                        // 【修改】：关闭串口一的原始语句实时回显功能
                        // 为保证调试界面干净（只看 AT 指令），这里不再使用 Usart_SendByte 打印数据
                        /*
                        for (int i = 0; publish_buf[i] != '\0'; i++) {
                            Usart_SendByte(DEBUG_USARTx, publish_buf[i]);
                        }
                        Usart_SendByte(DEBUG_USARTx, '\r');
                        Usart_SendByte(DEBUG_USARTx, '\n');
                        */
                        LOG_PRINT("[MQTT] Formatted Data Sent successfully.\r\n");
                        
                        break;
                    } else {
                        // 发送失败后短暂延时，重试
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                }
            } else {
                // 如果当前未连网，强制走一遍 Publish，它会自动触发 MqttCache_Push 缓存该报文
                success = Air780e_MqttPublish("/BEIDOU001/datas", publish_buf, 1, 0);
            }
            
            // 每处理完一条消息（无论是成功发往网络，还是存入了离线缓存，或是彻底失败丢弃）
            // 我们都释放信号量，让 GPS 任务立即去抓取下一句最新的报文
            if (gpsCaptureSync != NULL) {
                xSemaphoreGive(gpsCaptureSync);
            }
        }
        
        // 释放 CPU 给其他任务
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/* ================== 外部调用入口 ================== */
void myCodeStart(void)
{
    // 初始化同步信号量
    gpsCaptureSync = xSemaphoreCreateBinary();

    // 将发送缓存队列从深度 5 扩容到 10，以缓冲更多数据，128 调整到 256 以确保数据完整写入
    mqttPublishQueue = xQueueCreate(10, 256); 

    // 创建 MQTT 应用任务
    // 调整为相对较低优先级 3
    xTaskCreate(task_mqtt_app, "MQTT_APP", 384, NULL, 3, &mqttTaskHandle);
    
    // 创建 GPS 数据转发和处理任务
    // 提升优先级到 4，确保高速数据不溢出
    xTaskCreate(task_gps_forward, "GPS_TASK", 256, NULL, 4, &gpsTaskHandle);
    
    LOG_PRINT("[myCodeStart] Tasks Created.\r\n");
}