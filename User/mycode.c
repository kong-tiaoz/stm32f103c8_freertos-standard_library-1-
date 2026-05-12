/**
 * @file    mycode.c
 * @brief   系统主应用逻辑文件
 * @details 负责 GPS 数据采集 (TAU1201) 和 4G/MQTT 消息发布 (Air780e)。
 *          实现了多任务数据流的同步、NMEA GGA 报文解析、坐标格式转换，
 *          以及 JSON 格式化与 AT 报文组装。
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "queue.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "semphr.h"
#include "Air780e.h"
#include "bsp_usart.h"
#include <stdlib.h>

/**
 * @brief MQTT 发布任务句柄
 */
TaskHandle_t mqttTaskHandle;

/**
 * @brief MQTT 消息发布队列，用于 GPS 任务将组装好的 JSON 发送给 MQTT 任务
 * @note 队列元素类型为 char[256] 字符串。
 */
QueueHandle_t mqttPublishQueue = NULL;

/* ================== GPS 模块 (TAU1201) 配置与处理 ================== */

// GPS 硬件接口宏定义
#define GPS_USART             USART3
#define GPS_USART_CLK         RCC_APB1Periph_USART3
#define GPS_USART_GPIO_CLK    RCC_APB2Periph_GPIOB
#define GPS_USART_TX_PIN      GPIO_Pin_10
#define GPS_USART_RX_PIN      GPIO_Pin_11
#define GPS_USART_GPIO_PORT   GPIOB
#define GPS_USART_IRQn        USART3_IRQn

/**
 * @brief GPS 接收缓存队列，中断中将字符压入此队列，任务中提取。
 */
static QueueHandle_t gpsQueue = NULL;

/**
 * @brief GPS 任务句柄
 */
static TaskHandle_t gpsTaskHandle = NULL;

/**
 * @brief 二值信号量，用于同步 MQTT 发送和 GPS 采集。
 * @details 为了避免数据积压和串口长开带来的中断开销，
 *          只有当上一条 MQTT 数据发送完成后，才会 Give 该信号量允许下一次采集。
 */
static SemaphoreHandle_t gpsCaptureSync = NULL;

/*
 * TAU1201 (GPS) 模块配置指令集
 * 用于关闭不需要的 NMEA 语句输出，仅保留 GGA，减少 CPU 串口处理压力。
 */

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
/**
 * @brief 向 GPS 模块发送二进制指令 (轮询方式)
 * @param cmd 指令数组首地址
 * @param len 指令长度
 */
static void GPS_SendCmd(const uint8_t* cmd, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        USART_SendData(GPS_USART, cmd[i]);
        while (USART_GetFlagStatus(GPS_USART, USART_FLAG_TXE) == RESET);
    }
}

/**
 * @brief 初始化 GPS 连接的 USART3 接口及 NVIC
 * @details 包含引脚配置、波特率设置(115200)以及中断优先级配置。
 *          配置了较高的中断优先级以防高速数据流下发生 ORE(溢出) 错误。
 */
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

/**
 * @brief 关闭 GPS 串口接收
 * @details 禁用 USART3 及其 RXNE 中断，避免非采样期发生无用的中断从而节省 CPU 资源。
 */
static void GPS_Disable(void) {
    USART_Cmd(GPS_USART, DISABLE);
    USART_ITConfig(GPS_USART, USART_IT_RXNE, DISABLE);
}

/**
 * @brief 使能 GPS 串口接收
 * @details 使能前会先清空 SR、DR 以及接收队列中的残留数据，防止旧数据干扰最新位置。
 */
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

/**
 * @brief 向 GPS 发送多条关闭无关 NMEA 语句的指令
 * @details 通过连续发送专有协议指令，关闭 GLL、GSA、RMC、GSV 等语句，只留下 GGA。
 */
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

/**
 * @brief USART3 串口中断服务函数 (接收 GPS 数据)
 * @details 将接收到的单个字符压入 gpsQueue 队列。具备基本的错误处理能力，
 *          当发生 ORE/NE/FE 等硬件错误时丢弃该字节并清除标志位，防止死锁。
 */
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
 * @brief 提取 CSV 格式中指定索引的字段
 * @param csv 源字符串(逗号分隔)
 * @param field_idx 目标字段索引(从0开始)
 * @param out_buf 提取出的内容缓冲区
 * @param max_len out_buf 的最大长度
 * @note 边界条件：若找不到该字段，out_buf 将为空字符串。
 */
static void get_csv_field(const char *csv, int field_idx, char *out_buf, int max_len) {
    int cur_idx = 0;
    int out_idx = 0;
    out_buf[0] = '\0';
    for (int i = 0; csv[i] != '\0'; i++) {
        if (csv[i] == ',') {
            cur_idx++;
            if (cur_idx > field_idx) break;
        } else if (cur_idx == field_idx) {
            if (out_idx < max_len - 1) {
                out_buf[out_idx++] = csv[i];
                out_buf[out_idx] = '\0';
            }
        }
    }
}

/**
 * @brief 格式化时间并转换为北京时间 (UTC+8) 字符串 (YYYY-MM-DD HH:MM:SS)
 * @param raw 原始 NMEA UTC 时间字符串，格式通常为 hhmmss.sss
 * @param out 格式化输出缓冲区
 * @note 异常处理：若 raw 为空，则输出 "2026-05-10 00:00:00"。
 *       跨天处理：加 8 小时后对 24 取模。
 */
static void format_time(const char *raw, char *out) {
    if (raw[0] == '\0') {
        strcpy(out, "2026-05-10 00:00:00");
        return;
    }

    // NMEA 原生时间格式为 hhmmss.sss
    // 提取前 6 个数字
    int len = 0;
    char temp[7] = "000000";
    for (int i = 0; raw[i] != '\0' && len < 6; i++) {
        if (raw[i] >= '0' && raw[i] <= '9') {
            temp[len++] = raw[i];
        }
    }

    // 提取小时数并转换为整数
    char hour_str[3] = {temp[0], temp[1], '\0'};
    int hour = atoi(hour_str);

    // 加上 8 小时转为北京时间 (考虑跨天情况)
    hour = (hour + 8) % 24;

    // 重新写回固定日期及时和分秒
    sprintf(out, "2026-05-10 %02d:%c%c:%c%c", hour, temp[2], temp[3], temp[4], temp[5]);
}

/**
 * @brief 从 NMEA GGA 提取并转换为标准十进制度 (Decimal Degrees) 格式
 * @param raw 原始度分格式字符串
 * @param out 格式化输出缓冲区 (保留6位小数)
 * @param is_lon 标识是否为经度(暂留做扩展兼容)
 * @details
 *  - 纬度原生：ddmm.mmmm -> 转换后：dd.dddddd
 *  - 经度原生：dddmm.mmmm -> 转换后：ddd.dddddd
 * @note 异常处理：若 raw 为空，输出 "0.000000"。
 */
static void format_degree_minute(const char *raw, char *out, int is_lon) {
    if (raw[0] == '\0') {
        strcpy(out, "0.000000");
        return;
    }

    double raw_val = atof(raw);
    // 提取度 (除以100取整)
    int degrees = (int)(raw_val / 100.0);
    // 提取分
    double minutes = raw_val - (degrees * 100.0);
    // 合并为十进制度
    double decimal_degrees = degrees + (minutes / 60.0);

    // 转换为十进制度并保留六位小数 (满足高精度地图要求)
    sprintf(out, "%.6f", decimal_degrees);
}

/**
 * @brief GPS 数据处理与转发任务
 * @param arg 任务参数(未使用)
 * @details
 * 1. 上电等待并初始化 TAU1201 (只保留 GGA)。
 * 2. 阻塞等待 4G 模块成功连接。
 * 3. 循环等待 gpsCaptureSync 信号量。
 * 4. 使能串口捕获一条完整的 GGA 报文，解析为十进制度/时间/高程。
 * 5. 将解析结果格式化为指定 JSON，放入 mqttPublishQueue。
 * 6. 立即关闭串口，避免冗余中断消耗 CPU。
 */
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
                        gga_idx = 0; // 检测到报文头，重置索引
                    }
                    if (gga_idx < sizeof(latest_gga) - 1) {
                        latest_gga[gga_idx++] = ch;
                        latest_gga[gga_idx] = '\0';
                    }

                    // 遇到换行符，代表一帧结束
                    if (ch == '\n' || ch == '\r') {
                        if (gga_idx > 5) {
                            bool is_gga = (strstr(latest_gga, "GGA") != NULL);

                            if (is_gga) {
                                char *rn = strpbrk(latest_gga, "\r\n");
                                if (rn) *rn = '\0'; // 移除换行符

                                if (mqttPublishQueue != NULL) {
                                    char time_raw[20], lat_raw[20], lon_raw[20], alt_raw[20];
                                    // 根据 NMEA GGA 协议，提取特定索引字段
                                    get_csv_field(latest_gga, 1, time_raw, sizeof(time_raw)); // 时间
                                    get_csv_field(latest_gga, 2, lat_raw, sizeof(lat_raw));   // 纬度
                                    get_csv_field(latest_gga, 4, lon_raw, sizeof(lon_raw));   // 经度
                                    get_csv_field(latest_gga, 9, alt_raw, sizeof(alt_raw));   // 海拔

                                    char time_str[24], lat_str[20], lon_str[20];
                                    format_time(time_raw, time_str);
                                    format_degree_minute(lat_raw, lat_str, 0);
                                    format_degree_minute(lon_raw, lon_str, 1);

                                    double altitude = atof(alt_raw);

                                    char json_buf[256];
                                    // 对于合宙 Air780e 模块，发送带有双引号的 JSON 负载到 AT 指令中时，
                                    // 模块内部通过 \22 来识别双引号。
                                    // 为了能直接发送文本并在接收端看到标准 JSON，我们将双引号替换为 \22
                                    snprintf(json_buf, sizeof(json_buf),
                                        "{\\22id\\22:\\229\\22,\\22dataTime\\22:\\22%s\\22,\\22deviceId\\22:\\22ice\\22,\\22longitude\\22:%s,\\22latitude\\22:%s,\\22altitude\\22:%.2f,\\22pointType\\22:\\22ICE_POINT\\22}",
                                        time_str, lon_str, lat_str, altitude);

                                    // 送入发布队列
                                    xQueueSend(mqttPublishQueue, json_buf, 0);
                                }

                                gga_collected = true; // 采集完毕跳出循环
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

/**
 * @brief MQTT 内部事件回调函数
 * @param event 触发的事件类型
 * @param data 附带的数据结构体
 * @details 负责处理如网络连接、断开及数据接收的回调。
 *          当前配置为在收到数据时翻转 LED1 作为状态指示。
 */
static void MyMqttCallback(Air780e_MqttEvent_t event, void *data)
{
    switch (event) {
        case AIR780E_MQTT_EVENT_CONNECTED:
            break;
        case AIR780E_MQTT_EVENT_DISCONNECTED:
            break;
        case AIR780E_MQTT_EVENT_DATA_RECEIVED:
            LED1_TOGGLE; // 仅保留 LED 翻转提示
            break;
        default:
            break;
    }
}

/**
 * @brief MQTT 主业务任务
 * @param arg 任务参数(未使用)
 * @details
 * 1. 初始化 4G 硬件及连接参数(服务器IP/端口/Client ID等)。
 * 2. 轮询状态机 `Air780e_StateMachine`，自动处理网络断线重连。
 * 3. 阻塞检测 `mqttPublishQueue`。若有数据，则调用底层 AT 接口发布至主题 `/BEIDOU001/datas`。
 * 4. 具有异常重传机制 (最多重试 3 次)，并在发布结束后释放 `gpsCaptureSync` 让 GPS 继续采样。
 */
void task_mqtt_app(void *arg)
{
    LOG_PRINT("[MQTT Task] Initializing Air780e...\r\n");

    // 1. 初始化 4G 模块硬件接口 (串口、引脚)
    Air780e_Init();

    // 2. 配置服务器连接参数
    Air780e_MqttSetConfig("8.136.109.126", 1883, "BEIDOU_CLIENT_001", "mosquitto", "hfut");

    // 3. 配置高级参数 (不使用TLS，保活60s，CleanSession=1)
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
                        LOG_PRINT("[MQTT] NMEA Sent successfully.\r\n");
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

/**
 * @brief 系统外部调用入口 (任务创建启动点)
 * @details 负责初始化全局信号量和队列，并创建 GPS 转发任务和 MQTT 业务任务。
 */
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