/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <rtthread.h>
#ifdef BSP_USING_SDRAM

#include "sdram_port.h"
#include "board.h"
#include "fsl_semc.h"
#include "drv_sdram.h"

#define DRV_DEBUG
#define LOG_TAG             "drv.sdram"
#include <drv_log.h>


#ifdef RT_USING_MEMHEAP
static struct rt_memheap system_heap;
#endif

#ifndef CODE_RUN_ON_SDRAM
#define CODE_RUN_ON_SDRAM 1 // 1 -- code run on SDRAM memory; 0 -- code run on ram or xip flash memory
#endif

// #if (defined(USB_HOST_CONFIG_LOW_POWER_MODE) && (USB_HOST_CONFIG_LOW_POWER_MODE > 0U))
#if (defined(CODE_RUN_ON_SDRAM) && (CODE_RUN_ON_SDRAM > 0U))
#if defined(__ICCARM__) || defined(__GNUC__)
    extern uint32_t __SDRAM_HEAP_START[];
    extern uint32_t __SDRAM_HEAP_SIZE[];
    uint32_t sdram_heap_start = (uint32_t)__SDRAM_HEAP_START;
    uint32_t sdram_heap_size  = (uint32_t)__SDRAM_HEAP_SIZE;
#endif
#endif
    
int rt_hw_sdram_init(void)
{
    int result = RT_EOK;
#if !(defined(CODE_RUN_ON_SDRAM) && (CODE_RUN_ON_SDRAM > 0U))
    semc_config_t config;
    semc_sdram_config_t sdramconfig;

#if defined(SOC_IMXRT1170_SERIES)
//#if defined(USE_IS42S32400F) && (USE_IS42S32400F ==1)
//    {
//        clock_root_config_t semc_rootCfg = {0};
//        semc_rootCfg.mux = kCLOCK_SEMC_ClockRoot_MuxSysPll2Pfd1;
//        semc_rootCfg.div = 4;
//        CLOCK_SetRootClock(kCLOCK_Root_Semc, &semc_rootCfg);
//    }
//#endif
    rt_uint32_t clockFrq = CLOCK_GetRootClockFreq(kCLOCK_Root_Semc);
#else
    rt_uint32_t clockFrq = CLOCK_GetFreq(kCLOCK_SemcClk);
#endif

    /* Initializes the MAC configure structure to zero. */
    memset(&config, 0, sizeof(semc_config_t));
    memset(&sdramconfig, 0, sizeof(semc_sdram_config_t));

    /* Initialize SEMC. */
    SEMC_GetDefaultConfig(&config);
    #if !defined(USE_IS42S32400F) && (USE_IS42S32400F ==1)
      config.dqsMode = kSEMC_Loopbackdqspad;  /* For more accurate timing. */
    #endif
    SEMC_Init(SEMC, &config);

    /* Configure SDRAM. */
    sdramconfig.csxPinMux               = SDRAM_CS_PIN;
    sdramconfig.address                 = SDRAM_BANK_ADDR;
    sdramconfig.memsize_kbytes          = SDRAM_SIZE;
    sdramconfig.portSize                = SDRAM_DATA_WIDTH;
    sdramconfig.burstLen                = kSEMC_Sdram_BurstLen8;
    sdramconfig.columnAddrBitNum        = SDRAM_COLUMN_BITS;
    sdramconfig.casLatency              = SDRAM_CAS_LATENCY;
    sdramconfig.tPrecharge2Act_Ns       = SDRAM_TRP;
    sdramconfig.tAct2ReadWrite_Ns       = SDRAM_TRCD;
    sdramconfig.tRefreshRecovery_Ns     = SDRAM_REFRESH_RECOVERY;
    sdramconfig.tWriteRecovery_Ns       = SDRAM_TWR;
    sdramconfig.tCkeOff_Ns              = 42;  /* The minimum cycle of SDRAM CLK off state. CKE is off in self refresh at a minimum period tRAS.*/
    sdramconfig.tAct2Prechage_Ns        = SDRAM_TRAS;
    sdramconfig.tSelfRefRecovery_Ns     = 67;
    sdramconfig.tRefresh2Refresh_Ns     = SDRAM_TRC;
    sdramconfig.tAct2Act_Ns             = SDRAM_ACT2ACT;
    sdramconfig.tPrescalePeriod_Ns      = 160 * (1000000000 / clockFrq);
    sdramconfig.refreshPeriod_nsPerRow  = SDRAM_REFRESH_ROW;
    sdramconfig.refreshUrgThreshold     = sdramconfig.refreshPeriod_nsPerRow;
    sdramconfig.refreshBurstLen         = 1;
    result = SEMC_ConfigureSDRAM(SEMC, SDRAM_REGION, &sdramconfig, clockFrq);
    if(result != kStatus_Success)
    {
        LOG_E("SDRAM init failed!");
        result = -RT_ERROR;
    }
    else
    {
        LOG_D("sdram init success, mapped at 0x%X, size is %d Kbytes.", SDRAM_BANK_ADDR, SDRAM_SIZE);
#ifdef RT_USING_MEMHEAP
    /*
     * If RT_USING_MEMHEAP is enabled, SDRAM is initialized to the heap.
     * The heap start address is (base + half size), and the size is (half size - 2M).
     * The reasons are:
     *      1. Reserve the half space for SDRAM link case
     *      2. Reserve the 2M for non-cache space
     */
        rt_memheap_init(&system_heap, "sdram", (void *)(SDRAM_BANK_ADDR + (SDRAM_SIZE * 1024)/2 + (2 * 1024 * 1024)),
            (SDRAM_SIZE * 1024)/2 - (2 * 1024 * 1024));
#endif
    }
#else
     LOG_D("sdram init success, mapped at 0x%X, size is %d Kbytes.", sdram_heap_start, sdram_heap_size/1024);
#ifdef RT_USING_MEMHEAP
   /*
    * If RT_USING_MEMHEAP is enabled, SDRAM is initialized to the heap.
    * The heap start address is (base + half size), and the size is (half size - 2M).
    * The reasons are:
    *      1. Reserve the half space for SDRAM link case
    *      2. Reserve the 2M for non-cache space
    */
       rt_memheap_init(&system_heap, "sdram", (void *)(sdram_heap_start), sdram_heap_size);
//#endif
//
//     LOG_D("sdram init success, mapped at 0x%X, size is %d Kbytes.", (SDRAM_BANK_ADDR + (SDRAM_SIZE * 1024)/2 + (2 * 1024 * 1024)),\
//      ((SDRAM_SIZE * 1024)/2 - (2 * 1024 * 1024)) /1024);
//#ifdef RT_USING_MEMHEAP
//   /*
//    * If RT_USING_MEMHEAP is enabled, SDRAM is initialized to the heap.
//    * The heap start address is (base + half size), and the size is (half size - 2M).
//    * The reasons are:
//    *      1. Reserve the half space for SDRAM link case
//    *      2. Reserve the 2M for non-cache space
//    */
//       rt_memheap_init(&system_heap, "sdram", (void *)(SDRAM_BANK_ADDR + (SDRAM_SIZE * 1024)/2 + (2 * 1024 * 1024)),
//           (SDRAM_SIZE * 1024)/2 - (2 * 1024 * 1024));
#endif
#endif
    return result;
}
INIT_PREV_EXPORT(rt_hw_sdram_init);

#ifdef DRV_DEBUG
#ifdef FINSH_USING_MSH

#define SEMC_DATALEN                (0x1000U)
rt_uint32_t sdram_writeBuffer[SEMC_DATALEN];
rt_uint32_t sdram_readBuffer[SEMC_DATALEN];

/* read write 32bit test */
static void sdram_test(void)
{
    rt_uint32_t index;
    rt_uint32_t datalen = SEMC_DATALEN;
    rt_uint32_t *sdram = (rt_uint32_t *)SDRAM_BANK_ADDR; /* SDRAM start address. */
    bool result = true;

    LOG_D("SEMC SDRAM Memory 32 bit Write Start, Start Address 0x%x, Data Length %d !", sdram, datalen);
    /* Prepare data and write to SDRAM. */
    for (index = 0; index < datalen; index++)
    {
        sdram_writeBuffer[index] = index;
        sdram[index] = sdram_writeBuffer[index];
    }

    LOG_D("SEMC SDRAM Read 32 bit Data Start, Start Address 0x%x, Data Length %d !", sdram, datalen);
    /* Read data from the SDRAM. */
    for (index = 0; index < datalen; index++)
    {
        sdram_readBuffer[index] = sdram[index];
    }

    LOG_D("SEMC SDRAM 32 bit Data Write and Read Compare Start!");
    /* Compare the two buffers. */
    while (datalen--)
    {
        if (sdram_writeBuffer[datalen] != sdram_readBuffer[datalen])
        {
            result = false;
            break;
        }
    }

    if (!result)
    {
        LOG_E("SEMC SDRAM 32 bit Data Write and Read Compare Failed!");
    }
    else
    {
        LOG_D("SEMC SDRAM 32 bit Data Write and Read Compare Succeed!");
    }
}
MSH_CMD_EXPORT(sdram_test, sdram test)

#endif /* DRV_DEBUG */
#endif /* FINSH_USING_MSH */
#endif /* BSP_USING_SDRAM */
