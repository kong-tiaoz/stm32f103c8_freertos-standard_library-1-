
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_led.h"
#include "bsp_usart.h"
#include "bsp_key.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"
#include "bsp_usart.h"
#include <stdio.h>

//第二次提交测试
/*Function definition*/
extern void myCodeStart(void);
static void BSP_Init(void);
void USART3_Diagnostic_Loopback_Test(void);

static void Send_Task(void* pvParameters);
static void Callback_LEDTOGGLE(void* pvParameters);

static void AppTaskCreate(void *pvParameters);


/*Task Handle*/
static TaskHandle_t AppTaskCreate_Handle=NULL;
static TaskHandle_t Send_Task_Handle=NULL;       //PRO:3


/*Timer handle*/
static TimerHandle_t Timer1=NULL;

/* ======================================================================= */
/*                 USART3 硬件回环自发自收诊断测试单元                     */
/* ======================================================================= */
void USART3_Diagnostic_Loopback_Test(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    // 2. 配置 GPIO (PB10 TX, PB11 RX)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 3. 配置 USART3
    USART_InitStructure.USART_BaudRate = 115200; // 保持 115200 验证逻辑分析仪捕获
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);
    
    LOG_PRINT("\r\n=============================================\r\n");
    LOG_PRINT("--- USART3 Diagnostic Loopback Test Start ---\r\n");
    
    // 定义所有的 NMEA 关闭指令
    const uint8_t cmds[][11] = {
        {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11}, // close_gll
        {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13}, // close_gsa
        {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15}, // close_grs
        {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17}, // close_gsv
        {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19}, // close_rmc
        {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x06, 0x00, 0x00, 0x1B}, // close_vtg
        {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x07, 0x00, 0x01, 0x1D}, // close_zda
        {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x08, 0x00, 0x02, 0x1F}, // close_gst
        {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x40, 0x00, 0x3A, 0x8F}, // close_txt
        {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x20, 0x00, 0x1A, 0x4F}  // close_txt_ant
    };
    
    uint8_t rx_buf[150] = {0};
    int rx_cnt = 0;
    
    // 清空接收寄存器中的残余数据
    volatile uint16_t temp = USART3->SR;
    temp = USART3->DR;
    (void)temp;
    
    // 发送所有数据
    LOG_PRINT("[Diag] Sending all 10 NMEA Close Commands...\r\n");
    for (int cmd_idx = 0; cmd_idx < 10; cmd_idx++) {
        for (int i = 0; i < 11; i++) {
            USART_SendData(USART3, cmds[cmd_idx][i]);
            while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); // 等待 TXE 标志
        }
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);      // 等待 TC 标志
        
        // 粗略延时 50ms，模拟任务中的 vTaskDelay
        uint32_t delay_cnt = 72000 * 50; 
        while (delay_cnt--) {
            // 在延时期间也同时尝试接收回环数据
            if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET) {
                if (rx_cnt < (int)sizeof(rx_buf)) {
                    rx_buf[rx_cnt++] = USART_ReceiveData(USART3);
                }
            }
        }
    }
    
    LOG_PRINT("[Diag] Send complete. Waiting 200ms for loopback flush...\r\n");
    
    // 最后再阻塞接收 200ms 兜底
    uint32_t timeout = 72000 * 200; 
    while (timeout--) {
        if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET) {
            if (rx_cnt < (int)sizeof(rx_buf)) {
                rx_buf[rx_cnt++] = USART_ReceiveData(USART3);
            }
        }
    }
    
    // 打印接收结果
    LOG_PRINT("[Diag] Received %d bytes.\r\n", rx_cnt);
    if (rx_cnt > 0) {
        LOG_PRINT("[Diag] Data: \r\n");
        for (int i = 0; i < rx_cnt; i++) {
            LOG_PRINT("%02X ", rx_buf[i]);
            if ((i + 1) % 11 == 0) {
                LOG_PRINT("\r\n");
            }
        }
        LOG_PRINT("\r\n");
    }
    
    // 判断测试是否通过 (10 条指令，每条 11 字节 = 110 字节)
    if (rx_cnt == 110) {
        LOG_PRINT("[Diag] Loopback Test PASSED! MCU RX path is OK.\r\n");
    } else {
        LOG_PRINT("[Diag] Loopback Test FAILED! Please check TX-RX short connection.\r\n");
    }
    LOG_PRINT("=============================================\r\n\r\n");
    
    // 恢复默认状态，避免影响后续 FreeRTOS 任务中的正式初始化
    USART_Cmd(USART3, DISABLE);
}

static void Callback_LEDTOGGLE(void* pvParameters)
{
  LED1_TOGGLE;
}


int main(void)
{  
  BaseType_t xReturn = pdPASS;
  
  BSP_Init();
  Log_Init();
  
  LOG_PRINT("FreeRTOS creates tasks dynamically!\r\n");
  
  /* Create the initial task which will create all other tasks */
  xReturn = xTaskCreate((TaskFunction_t)AppTaskCreate,
                        "APPTASKCREATE",
                        512,
                        NULL,
                        1,
                        &AppTaskCreate_Handle);
                        
  if(pdPASS == xReturn)
  {
    vTaskStartScheduler();
  }
  else
  {
    LOG_PRINT("Failed to create AppTaskCreate task!\r\n");
    return -1;
  }
  
  /* Execution should never reach here unless scheduler fails to start (e.g. out of heap) */
  while(1);
}


/*Hardware initialization*/
static void BSP_Init(void)
{
  /*Use this priority grouping uniformly, never group again
  * 4 bits for pre-emption priority
  */
  
  /*led initialization*/
  LED_GPIO_Config();
  
  /*Serial port initialization*/
  USART_Config();
  
  /*Key initialization*/
  Key_GPIO_Config();
}

/***********
 *@Function name:SEND_Task
 *@description:SEND_Task body
 *@Parameter:None
 *@return:None
 *************/
static void Send_Task(void* pvParameters)
{
  while(1)
  {  
    // Just to demonstrate task switching and scheduling.
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void AppTaskCreate(void *pvParameters)
{
  BaseType_t xReturn = pdPASS;

  /* NOTE: Removed taskENTER_CRITICAL() and taskEXIT_CRITICAL() here.
     It is generally unnecessary to disable interrupts while creating tasks and timers,
     and doing so can cause issues if any creation API blocks or takes too long. */

  /* Create Timer1 Handle*/
  Timer1 = xTimerCreate("1sec-toggle",
                        pdMS_TO_TICKS(1000),
                        pdTRUE,
                        (void*)1,
                        Callback_LEDTOGGLE);
                        
  if(NULL != Timer1)
  {
    LOG_PRINT("Timer1 created successfully.\r\n");
    if(pdPASS != xTimerStart(Timer1, 0))
    {
      LOG_PRINT("Timer1 start failed!\r\n");
    }
  }
  else
  {
    LOG_PRINT("Timer1 creation failed!\r\n");
  }

  /*SEND task*/
  xReturn = xTaskCreate((TaskFunction_t)Send_Task,
                        "SEND_TASK",
                        512,
                        NULL,
                        3,
                        &Send_Task_Handle);
  if(pdPASS == xReturn)
  {
    LOG_PRINT("Create Send_Task success!\r\n");
  }
  else
  {
    LOG_PRINT("Create Send_Task failed!\r\n");
  }
  
  /* Execute user external task creations */
  LOG_PRINT("Starting user code (myCodeStart)...\r\n");
  myCodeStart();

  /* Delete the init task safely. Passing NULL deletes the calling task. 
     This also correctly returns 2KB of stack memory to the heap. */
  LOG_PRINT("AppTaskCreate init finished. Deleting init task...\r\n");
  vTaskDelete(NULL);
}









/*********************************************END OF FILE**********************/
