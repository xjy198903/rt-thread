/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-05-06     tyustli      first version
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include <drv_gpio.h>
#include <fsl_gpio.h>
#include "fpga_config.h"
#include "cnc_can.h"
#include "board.h"

#define DBG_TAG "app"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define EXAMPLE_LED_GPIO     GPIO9
#define EXAMPLE_LED_GPIO_PIN   (3U)
#define EXAMPLE_LED_GPIO_PORT  (3U)
#define LED_PIN GET_PIN(EXAMPLE_LED_GPIO_PORT, EXAMPLE_LED_GPIO_PIN)

#define POWER_ON_GPIO_PIN (3U)
#define POWER_ON_GPIO_PORT (25U)
#define POWER_ON_GPIO GET_PIN(POWER_ON_GPIO_PIN, POWER_ON_GPIO_PORT)

#define POWER2_ON_GPIO_PIN (3U)
#define POWER2_ON_GPIO_PORT (26U)
#define POWER2_ON_GPIO GET_PIN(POWER2_ON_GPIO_PIN, POWER2_ON_GPIO_PORT)

// #ifdef BSP_USING_RPMSG
// #include <rpmsg_lite.h>
// #include <mcmgr.h>
// #define RPMSG_LITE_LINK_ID (RL_PLATFORM_IMXRT1170_M7_M4_LINK_ID)

// /* Address of memory, from which the secondary core will boot */
// #define CORE1_BOOT_ADDRESS (void *)0x20200000

// #if defined(__CC_ARM) || defined(__ARMCC_VERSION)
// extern uint32_t Image$$CORE1_REGION$$Base;
// extern uint32_t Image$$CORE1_REGION$$Length;
// #define CORE1_IMAGE_START &Image$$CORE1_REGION$$Base
// #elif defined(__ICCARM__)
// extern unsigned char core1_image_start[];
// #define CORE1_IMAGE_START core1_image_start
// #elif (defined(__GNUC__)) && (!defined(__MCUXPRESSO))
// extern const char core1_image_start[];
// extern const char *core1_image_end;
// extern int core1_image_size;
// #define CORE1_IMAGE_START ((void *)core1_image_start)
// #define CORE1_IMAGE_SIZE ((void *)core1_image_size)
// #endif
// #define REMOTE_EPT_ADDR (30U)
// #define LOCAL_EPT_ADDR (40U)
// #define APP_RPMSG_READY_EVENT_DATA (1U)
// #define APP_RPMSG_EP_READY_EVENT_DATA (2U)

// typedef struct the_message
// {
//     uint32_t DATA;
// } THE_MESSAGE, *THE_MESSAGE_PTR;

// #define SH_MEM_TOTAL_SIZE (6144U)
// #if defined(__ICCARM__) /* IAR Workbench */
// #pragma location = "rpmsg_sh_mem_section"
// static char rpmsg_lite_base[SH_MEM_TOTAL_SIZE];
// #elif defined(__CC_ARM) || defined(__ARMCC_VERSION) /* Keil MDK */
// static char rpmsg_lite_base[SH_MEM_TOTAL_SIZE] __attribute__((section("rpmsg_sh_mem_section")));
// #elif defined(__GNUC__)
// static char rpmsg_lite_base[SH_MEM_TOTAL_SIZE] __attribute__((section(".noinit.$rpmsg_sh_mem")));
// #else
// #error "RPMsg: Please provide your definition of rpmsg_lite_base[]!"
// #endif

// /*******************************************************************************
//  * Prototypes
//  ******************************************************************************/
// #ifdef CORE1_IMAGE_COPY_TO_RAM
// uint32_t get_core1_image_size(void);
// #endif

// /*******************************************************************************
//  * Code
//  ******************************************************************************/

// #ifdef CORE1_IMAGE_COPY_TO_RAM
// uint32_t get_core1_image_size(void)
// {
//     uint32_t image_size;
// #if defined(__CC_ARM) || defined(__ARMCC_VERSION)
//     image_size = (uint32_t)&Image$$CORE1_REGION$$Length;
// #elif defined(__ICCARM__)
// #pragma section = "__core1_image"
//     printf("end address:0x%x, start address: 0x%x\n", (uint32_t)__section_end("__core1_image"), (uint32_t)&core1_image_start);
//     image_size = (uint32_t)__section_end("__core1_image") - (uint32_t)&core1_image_start;
// #elif defined(__GNUC__)
//     image_size = (uint32_t)core1_image_size;
// #endif
//     return image_size;
// }
// #endif
// static THE_MESSAGE volatile msg = {0};
// static struct rt_semaphore rpmsg_sem;
// /* This is the read callback, note we are in a task context when this callback
// is invoked, so kernel primitives can be used freely */
// static int32_t my_ept_read_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv)
// {
//     // int32_t *has_received = priv;

//     if (payload_len <= sizeof(THE_MESSAGE))
//     {
//         (void)memcpy((void *)&msg, payload, payload_len);
//         // *has_received = 1;
//         rt_sem_release(&rpmsg_sem);
//     }
//     //    printf("Primary core received a msg\r\n");
//     //printf("Message: Size=%x, DATA = %i\r\n", payload_len, msg.DATA);
//     return RL_RELEASE;
// }

// static void RPMsgRemoteReadyEventHandler(uint16_t eventData, void *context)
// {
//     uint16_t *data = (uint16_t *)context;

//     *data = eventData;
// }

// /*!
//  * @brief Application-specific implementation of the SystemInitHook() weak function.
//  */
// void SystemInitHook(void)
// {
//     /* Initialize MCMGR - low level multicore management library. Call this
//        function as close to the reset entry as possible to allow CoreUp event
//        triggering. The SystemInitHook() weak function overloading is used in this
//        application. */
//     (void)MCMGR_EarlyInit();
// }

// void rpmsg_thread_entry(void *arg)
// {
//     volatile int32_t has_received;
//     volatile uint16_t RPMsgRemoteReadyEventData = 0;
//     struct rpmsg_lite_ept_static_context my_ept_context;
//     struct rpmsg_lite_endpoint *my_ept;
//     struct rpmsg_lite_instance rpmsg_ctxt;
//     struct rpmsg_lite_instance *my_rpmsg;

//     printf("enter rpmsg thread\r\n");

// #ifdef CORE1_IMAGE_COPY_TO_RAM
//     /* This section ensures the secondary core image is copied from flash location to the target RAM memory.
//        It consists of several steps: image size calculation and image copying.
//        These steps are not required on MCUXpresso IDE which copies the secondary core image to the target memory during
//        startup automatically. */
//     printf("copy core image to RAM\r\n");
//     uint32_t core1_image_size;
//     core1_image_size = get_core1_image_size();
//     (void)printf("Copy CORE1 image to address: 0x%x, size: %d\r\n", (void *)(char *)CORE1_BOOT_ADDRESS,
//                      core1_image_size);

//     /* Copy application from FLASH to RAM */
//     (void)memcpy((void *)(char *)CORE1_BOOT_ADDRESS, (void *)CORE1_IMAGE_START, core1_image_size);
// #endif

//     /* Initialize MCMGR before calling its API */
//     (void)MCMGR_Init();

//     /* Register the application event before starting the secondary core */
//     (void)MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent, RPMsgRemoteReadyEventHandler,
//                               (void *)&RPMsgRemoteReadyEventData);

//     /* Boot Secondary core application */
//     (void)MCMGR_StartCore(kMCMGR_Core1, (void *)(char *)CORE1_BOOT_ADDRESS, (uint32_t)rpmsg_lite_base,
//                           kMCMGR_Start_Synchronous);

//     /* Print the initial banner */
//     (void)printf("RPMsg starts\r\n");

//     /* Wait until the secondary core application signals the rpmsg remote has been initialized and is ready to
//      * communicate. */
//     while (APP_RPMSG_READY_EVENT_DATA != RPMsgRemoteReadyEventData)
//     {
//     };

//     my_rpmsg = rpmsg_lite_master_init(rpmsg_lite_base, SH_MEM_TOTAL_SIZE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS, &rpmsg_ctxt);

//     my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, my_ept_read_cb, (void *)&has_received, &my_ept_context);

//     // has_received = 0;
//     /* init rpmsg semaphore */
//     rt_sem_init(&rpmsg_sem, "rpmsg_sem", 0, RT_IPC_FLAG_FIFO);

//     /* Wait until the secondary core application signals the rpmsg remote endpoint has been created. */
//     while (APP_RPMSG_EP_READY_EVENT_DATA != RPMsgRemoteReadyEventData)
//     {
//     };

//     /* Send the first message to the remoteproc */
//     msg.DATA = 0U;
//     (void)rpmsg_lite_send(my_rpmsg, my_ept, REMOTE_EPT_ADDR, (char *)&msg, sizeof(THE_MESSAGE), RL_DONT_BLOCK);

//     //数据处理部分
//     while (msg.DATA <= 100)
//     {
//         rt_sem_take(&rpmsg_sem, RT_WAITING_FOREVER);
//         // if (has_received == 1)
//         // {
//         has_received = 0;
//         msg.DATA++;
//         (void)rpmsg_lite_send(my_rpmsg, my_ept, REMOTE_EPT_ADDR, (char *)&msg, sizeof(THE_MESSAGE), RL_DONT_BLOCK);
//         // }
//     }

//         (void)rpmsg_lite_destroy_ept(my_rpmsg, my_ept);
//         my_ept = ((void *)0);
//         (void)rpmsg_lite_deinit(my_rpmsg);
    
//         /* Print the ending banner */
//         (void)printf("RPMsg demo ends\r\n");
// }
// #endif

void led_thread_entry(void *arg)
{
    // //浮点数输出测试
    // int a1 = 12;
    // printf("a1 = %d\n", a1);

    // float a2 = 12.24;
    // printf("a2 = %lf\r\n", a2);

    // double a3 = 1.8234113;
    // printf("a3: %f\r\n", a3);

    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
    while (1)
    {
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
    }
}

int main(void)
{
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
    printf("hello cncbeijing for version 1.0.0\r\n");
    
    LOG_D("heap start addr: 0x%x, heap end addr: 0x%x\r\n", HEAP_BEGIN, HEAP_END);

//    rt_hw_us_delay(500000);

// #ifdef BSP_USING_RPMSG
//     //创建双核通信线程
//     rt_thread_t rpmsg_tid;
//     rpmsg_tid = rt_thread_create("rpmsg_thread",
//                                  rpmsg_thread_entry,
//                                  RT_NULL,
//                                  1024,
//                                  8,
//                                  20);
//     if (rpmsg_tid != RT_NULL)
//         rt_thread_startup(rpmsg_tid);
// #endif

    //创建LED线程
    rt_thread_t led_tid;
    led_tid = rt_thread_create("led", led_thread_entry, RT_NULL, 512, RT_THREAD_PRIORITY_MAX - 2, 2);
    if (led_tid != RT_NULL)
    {
        rt_thread_startup(led_tid);
    }
    
   config_fpga_firmware();

    //turn on power
    rt_pin_mode(POWER_ON_GPIO, PIN_MODE_OUTPUT);
    rt_pin_write(POWER_ON_GPIO, PIN_HIGH);

    //turn on power2
    rt_pin_mode(POWER2_ON_GPIO, PIN_MODE_OUTPUT);
    rt_pin_write(POWER2_ON_GPIO, PIN_HIGH);

#ifdef BSP_USING_CAN
    // can thread test
    rt_thread_t can_tid;
    can_tid = rt_thread_create("can comm", can_comm_entry, NULL, CAN_STACK_SIZE, CAN_THREAD_PRIORITY, CAN_THREAD_TICK);
    if (can_tid != RT_NULL)
    {
        rt_thread_startup(can_tid);
    }
#endif

    while (1)
    {
        //rt_pin_write(LED_PIN, PIN_LOW);
        //rt_thread_mdelay(500);
        //rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
    }
}
