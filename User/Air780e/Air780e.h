/**
  ******************************************************************************
  * @file    Air780e.h
  * @author  Code Assistant
  * @version V2.0.0
  * @date    2024-xx-xx
  * @brief   Air780e 4G模块驱动头文件 (兼容裸机与RTOS) - MQTT增强版
  *          适用于 STM32F103C8T6 及其他 STM32 平台
  ******************************************************************************
  */

#ifndef __AIR780E_H
#define __AIR780E_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ========================== 硬件接口配置宏 ========================== */
/* 串口定义，使用 USART2 作为 4G 模块通信 */
#define AIR780E_UART USART2

/* 模块电源控制引脚 (PWR) 默认配置: PB0 */
#define AIR780E_PWR_PORT    GPIOB
#define AIR780E_PWR_PIN     GPIO_Pin_0

/* 模块硬件复位引脚 (RST) 默认配置: PB1 */
#define AIR780E_RST_PORT    GPIOB
#define AIR780E_RST_PIN     GPIO_Pin_1

/* 电源和复位操作宏 */
#define AIR780E_PWR_ON()    GPIO_SetBits(AIR780E_PWR_PORT, AIR780E_PWR_PIN)
#define AIR780E_PWR_OFF()   GPIO_ResetBits(AIR780E_PWR_PORT, AIR780E_PWR_PIN)
#define AIR780E_RST_HIGH()  GPIO_SetBits(AIR780E_RST_PORT, AIR780E_RST_PIN)
#define AIR780E_RST_LOW()   GPIO_ResetBits(AIR780E_RST_PORT, AIR780E_RST_PIN)

/* ========================== 状态机与配置定义 ========================== */
/* 接收缓冲区大小定义 (适配 C8T6 内存，不宜过大) */
#define AIR780E_RX_BUF_SIZE 512

/* 4G 模块运行状态枚举 */
typedef enum {
    AIR780E_STATE_POWER_OFF = 0,    // 关机状态
    AIR780E_STATE_INIT_RESET,       // Case 0 & 1: IPR & RESET
    AIR780E_STATE_CHECK_CSQ,        // Case 2: CSQ
    AIR780E_STATE_CHECK_CCID,       // Case 3: CCID
    AIR780E_STATE_CHECK_CGREG,      // Case 4: CGREG?
    AIR780E_STATE_CHECK_CGATT,      // Case 5: CGATT?
    AIR780E_STATE_MQTT_CONFIG,      // Case 6: MCONFIG
    AIR780E_STATE_MQTT_MIPSTART,    // Case 7: MIPSTART
    AIR780E_STATE_MQTT_MCONNECT,    // Case 8: MCONNECT
    AIR780E_STATE_MQTT_MSUB,        // Case 9: MSUB
    AIR780E_STATE_MQTT_CONNECTED,   // Case 10: Connected & Running
    AIR780E_STATE_WAIT_BACKOFF,     // 错误后等待退避重连
    AIR780E_STATE_ERROR             // 严重错误状态
} Air780e_State_t;

/* MQTT 连接事件回调枚举 */
typedef enum {
    AIR780E_MQTT_EVENT_CONNECTED = 0, // MQTT 连接成功
    AIR780E_MQTT_EVENT_DISCONNECTED,  // MQTT 断开连接
    AIR780E_MQTT_EVENT_PUBLISHED,     // MQTT 消息发布成功
    AIR780E_MQTT_EVENT_DATA_RECEIVED, // MQTT 收到订阅消息
    AIR780E_MQTT_EVENT_ERROR          // MQTT 发生错误
} Air780e_MqttEvent_t;

/* MQTT 接收到的数据结构 */
typedef struct {
    char topic[64];
    uint8_t payload[256];
    uint16_t payload_len;
} Air780e_MqttData_t;

/* MQTT 事件回调函数类型 */
typedef void (*Air780e_MqttEventCallback_t)(Air780e_MqttEvent_t event, void *data);

/* 离线缓存相关配置 */
#define AIR780E_MQTT_CACHE_SIZE 3   // 缓存最多3条离线消息以节约RAM

typedef struct {
    char topic[32];
    char payload[256];
    uint8_t qos;
    uint8_t retain;
    bool valid;
} Air780e_MqttMsg_t;

typedef struct {
    Air780e_MqttMsg_t msgs[AIR780E_MQTT_CACHE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} Air780e_MqttCache_t;

/* MQTT 初始化与网络参数结构体 */
typedef struct {
    char broker[64];                // MQTT 服务器IP地址或域名
    uint16_t port;                  // MQTT 服务器端口
    char client_id[32];             // MQTT 客户端ID
    char username[32];              // MQTT 用户名
    char password[32];              // MQTT 密码
    
    bool use_tls;                   // 是否使用 TLS 加密
    
    // LWT (遗嘱消息) 设置
    bool use_lwt;                   // 是否启用遗嘱消息
    char lwt_topic[32];             // 遗嘱主题
    char lwt_msg[32];               // 遗嘱内容
    uint8_t lwt_qos;                // 遗嘱QoS
    uint8_t lwt_retain;             // 遗嘱Retain
    
    // 保活时间与清除会话
    uint16_t keepalive;             // 保活时间 (秒)
    uint8_t clean_session;          // 清除会话 (0或1)

    // 默认订阅
    char sub_topic[64];             // 初始默认订阅主题
    uint8_t sub_qos;                // 初始默认订阅QoS
    
    // 回调
    Air780e_MqttEventCallback_t event_cb; // 事件回调函数
} Air780e_MqttConfig_t;

/* Air780e 模块运行上下文结构体 */
typedef struct {
    Air780e_State_t state;          // 模块当前状态机状态
    bool is_connected;              // 是否已成功连接到 MQTT Broker
    uint32_t last_tick;             // 上次关键操作的时间戳 (ms)
    uint32_t heartbeat_tick;        // 心跳保活时间戳 (ms)
    uint32_t error_tick;            // 发生错误时的时间戳 (ms)
    uint32_t backoff_delay_ms;      // 指数退避延时 (ms)
    uint8_t error_count;            // 连续出错次数
    
    Air780e_MqttConfig_t mqtt_cfg;  // MQTT 连接参数配置
    Air780e_MqttCache_t cache;      // 离线消息缓存队列
} Air780e_Context_t;

/* ========================== 对外 API 声明 ========================== */

/**
 * @brief  初始化 Air780e 模块硬件接口
 * @note   包含 GPIO 的初始化及串口接收中断的开启
 */
void Air780e_Init(void);

/**
 * @brief  设置 MQTT 核心连接参数
 * @param  broker: 服务器 IP 或域名
 * @param  port: 端口号 (一般 TCP 为 1883，TLS 为 8883)
 * @param  client_id: 客户端标识符
 * @param  username: 用户名
 * @param  password: 密码
 */
void Air780e_MqttSetConfig(const char *broker, uint16_t port, const char *client_id, const char *username, const char *password);

/**
 * @brief  设置 MQTT 遗嘱消息 (LWT)
 * @param  topic: 遗嘱主题
 * @param  msg: 遗嘱内容
 * @param  qos: 质量等级 (0, 1, 2)
 * @param  retain: 保留标志 (0, 1)
 */
void Air780e_MqttSetLwt(const char *topic, const char *msg, uint8_t qos, uint8_t retain);

/**
 * @brief  设置 MQTT 其他高级参数
 * @param  use_tls: 是否启用 TLS
 * @param  keepalive: 心跳保活间隔 (秒)，默认建议 120
 * @param  clean_session: 是否清除会话 (0 或 1)
 */
void Air780e_MqttSetAdvanced(bool use_tls, uint16_t keepalive, uint8_t clean_session);

/**
 * @brief  设置初始默认订阅主题及事件回调
 * @param  sub_topic: 初始需要订阅的主题
 * @param  sub_qos: 订阅的 QoS 等级
 * @param  cb: MQTT 事件回调函数
 */
void Air780e_MqttSetCallback(const char *sub_topic, uint8_t sub_qos, Air780e_MqttEventCallback_t cb);

/**
 * @brief  控制模块硬件开机
 */
void Air780e_PowerOn(void);

/**
 * @brief  控制模块硬件关机
 */
void Air780e_PowerOff(void);

/**
 * @brief  控制模块硬件复位
 */
void Air780e_HardwareReset(void);

/**
 * @brief  Air780e 核心非阻塞状态机处理函数
 * @note   需在主循环 (while(1)) 中频繁调用。
 *         自动管理 AT 指令、MQTT 连接、断线重连及退避。
 */
void Air780e_StateMachine(void);



/**
 * @brief  发布 MQTT 消息
 * @param  topic: 目标主题
 * @param  payload: 消息内容字符串
 * @param  qos: 质量等级 (0, 1, 2)
 * @param  retain: 保留标志 (0, 1)
 * @return true: 已进入发送或缓存队列, false: 缓存满或未连接且不可缓存
 */
bool Air780e_MqttPublish(const char *topic, const char *payload, uint8_t qos, uint8_t retain);

/**
 * @brief  动态订阅 MQTT 主题
 * @param  topic: 目标主题
 * @param  qos: 质量等级 (0, 1, 2)
 * @return true: 发送订阅指令成功, false: 未连接
 */
bool Air780e_MqttSubscribe(const char *topic, uint8_t qos);

/**
 * @brief  获取当前 4G 模块的 MQTT 连接状态
 * @return true: 已连接, false: 未连接
 */
bool Air780e_IsConnected(void);

/**
 * @brief  运行一次测试框架，用于端到端验证
 */
void Air780e_TestRun(void);

#ifdef __cplusplus
}
#endif

#endif /* __AIR780E_H */
