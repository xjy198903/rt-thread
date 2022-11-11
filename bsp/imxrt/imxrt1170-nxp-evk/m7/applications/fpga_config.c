#include "pin_mux.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "fpga_config.h"

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitFPGAConfigPins:
- options: {callFromInitBoot: 'true', coreID: cm7, enableClock: 'true', fullInit: 'true'}
- pin_list:
  - {pin_num: P15, peripheral: GPIO3, signal: 'gpio_mux_io, 02', pin_signal: GPIO_AD_03, direction: INPUT, gpio_interrupt: kGPIO_NoIntmode, software_input_on: Enable,
    pull_up_down_config: Pull_Down, pull_keeper_select: Pull, open_drain: Disable, drive_strength: High, slew_rate: Slow}
  - {pin_num: K13, peripheral: GPIO3, signal: 'gpio_mux_io, 19', pin_signal: GPIO_AD_20, direction: INPUT, gpio_interrupt: kGPIO_NoIntmode, software_input_on: Enable,
    pull_up_down_config: Pull_Down, pull_keeper_select: Pull, open_drain: Disable, drive_strength: High, slew_rate: Slow}
  - {pin_num: K14, peripheral: GPIO3, signal: 'gpio_mux_io, 20', pin_signal: GPIO_AD_21, direction: OUTPUT, gpio_init_state: 'true', software_input_on: Disable, pull_up_down_config: Pull_Down,
    pull_keeper_select: Pull, open_drain: Disable, drive_strength: High, slew_rate: Slow}
  - {pin_num: P5, peripheral: GPIO6, signal: 'gpio_mux_io, 09', pin_signal: GPIO_LPSR_09, direction: OUTPUT, gpio_init_state: 'false', software_input_on: Disable,
    pull_up_down_config: Pull_Down, pull_keeper_select: Keeper, open_drain: Disable, drive_strength: High, slew_rate: Slow}
  - {pin_num: U8, peripheral: GPIO6, signal: 'gpio_mux_io, 08', pin_signal: GPIO_LPSR_08, direction: OUTPUT, gpio_init_state: 'false', software_input_on: Disable,
    pull_up_down_config: Pull_Down, pull_keeper_select: Keeper, open_drain: Disable, drive_strength: High, slew_rate: Slow}
  - {pin_num: F14, peripheral: GPIO4, signal: 'gpio_mux_io, 13', pin_signal: GPIO_SD_B2_04, direction: INPUT, gpio_interrupt: kGPIO_NoIntmode, software_input_on: Enable,
    pull_down_pull_up_config: Pull_Up, pdrv_config: High_Driver, open_drain: Disable}
  - {pin_num: E14, peripheral: GPIO4, signal: 'gpio_mux_io, 14', pin_signal: GPIO_SD_B2_05, direction: INPUT, gpio_interrupt: kGPIO_NoIntmode, software_input_on: Enable,
    pull_down_pull_up_config: Pull_Up, pdrv_config: High_Driver, open_drain: Disable}
  - {pin_num: L1, peripheral: GPIO2, signal: 'gpio_mux_io, 09', pin_signal: GPIO_EMC_B1_41, direction: INPUT, gpio_interrupt: kGPIO_NoIntmode, software_input_on: Enable,
    pull_down_pull_up_config: Pull_Down, pdrv_config: High_Driver, open_drain: Disable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitFPGAConfigPins, assigned for the Cortex-M7F core.
 * Description   : Configures pin routing and optionally pin electrical features. to assign gpios for configing firmware of fpga
 *
 * END ****************************************************************************************************************/
static void BOARD_InitFPGAFirmWareConfigPins(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc);      /* LPCG on: LPCG is ON. */
    CLOCK_EnableClock(kCLOCK_Iomuxc_Lpsr); /* LPCG on: LPCG is ON. */

    /* GPIO configuration of gpio3_02_pinP15 on GPIO_AD_03 (pin P15), FPGA_CONF_DONE */
    gpio_pin_config_t gpio3_02_pinP15_config = {
        .direction = kGPIO_DigitalInput,
        .outputLogic = 0U,
        .interruptMode = kGPIO_NoIntmode};
    /* Initialize GPIO functionality on GPIO_AD_03 (pin P15) */
    GPIO_PinInit(GPIO3, 2U, &gpio3_02_pinP15_config);

    /* GPIO configuration of gpio3_19_pinK13 on GPIO_AD_20 (pin K13), for FPGA_NSTATUS */
    gpio_pin_config_t gpio3_19_pinK13_config = {
        .direction = kGPIO_DigitalInput,
        .outputLogic = 0U,
        .interruptMode = kGPIO_NoIntmode};
    /* Initialize GPIO functionality on GPIO_AD_20 (pin K13) */
    GPIO_PinInit(GPIO3, 19U, &gpio3_19_pinK13_config);

    /* GPIO configuration of gpio3_20_pinK14 on GPIO_AD_21 (pin K14), for FPGA_NCONFIG */
    gpio_pin_config_t gpio3_20_pinK14_config = {
        .direction = kGPIO_DigitalOutput,
        .outputLogic = 0U,
        .interruptMode = kGPIO_NoIntmode};
    /* Initialize GPIO functionality on GPIO_AD_21 (pin K14) */
    GPIO_PinInit(GPIO3, 20U, &gpio3_20_pinK14_config);

    /* GPIO configuration of gpio6_08_pinU8 on IOMUXC_LPSR_SW_MUX_CTL_PAD_GPIO_LPSR_08 (pin U8), for FPGA_DATA0 */
    gpio_pin_config_t gpio6_08_pinU8_config = {
        .direction = kGPIO_DigitalOutput,
        .outputLogic = 0U,
        .interruptMode = kGPIO_NoIntmode};
    /* Initialize GPIO functionality on IOMUXC_LPSR_SW_MUX_CTL_PAD_GPIO_LPSR_08 (pin U8) */
    GPIO_PinInit(GPIO6, 8U, &gpio6_08_pinU8_config);

    /* GPIO configuration of gpio6_09_pinP5 on IOMUXC_LPSR_SW_MUX_CTL_PAD_GPIO_LPSR_09 (pin P5), for FPGA_DCLK */
    gpio_pin_config_t gpio6_09_pinP5_config = {
        .direction = kGPIO_DigitalOutput,
        .outputLogic = 0U,
        .interruptMode = kGPIO_NoIntmode};
    /* Initialize GPIO functionality on IOMUXC_LPSR_SW_MUX_CTL_PAD_GPIO_LPSR_09 (pin P5) */
    GPIO_PinInit(GPIO6, 9U, &gpio6_09_pinP5_config);

    IOMUXC_SetPinMux(
        IOMUXC_GPIO_AD_03_GPIO_MUX3_IO02, /* GPIO_AD_03 is configured as GPIO_MUX3_IO02 */
        1U);                              /* Software Input On Field: Force input path of pad GPIO_AD_03 */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_AD_20_GPIO_MUX3_IO19, /* GPIO_AD_20 is configured as GPIO_MUX3_IO19 */
        1U);                              /* Software Input On Field: Force input path of pad GPIO_AD_20 */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_AD_21_GPIO_MUX3_IO20, /* GPIO_AD_21 is configured as GPIO_MUX3_IO20 */
        1U);                              /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_GPR->GPR42 = ((IOMUXC_GPR->GPR42 &
                          (~(BOARD_INITFPGACONFIGPINS_IOMUXC_GPR_GPR42_GPIO_MUX3_GPIO_SEL_LOW_MASK))) /* Mask bits to zero which are setting */
                         | IOMUXC_GPR_GPR42_GPIO_MUX3_GPIO_SEL_LOW(0x00U)                             /* GPIO3 and CM7_GPIO3 share same IO MUX function, GPIO_MUX3 selects one GPIO function: 0x00U */
    );
    IOMUXC_GPR->GPR43 = ((IOMUXC_GPR->GPR43 &
                          (~(BOARD_INITFPGACONFIGPINS_IOMUXC_GPR_GPR43_GPIO_MUX3_GPIO_SEL_HIGH_MASK))) /* Mask bits to zero which are setting */
                         | IOMUXC_GPR_GPR43_GPIO_MUX3_GPIO_SEL_HIGH(0x00U)                             /* GPIO3 and CM7_GPIO3 share same IO MUX function, GPIO_MUX3 selects one GPIO function: 0x00U */
    );
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_LPSR_08_GPIO_MUX6_IO08, /* GPIO_LPSR_08 is configured as GPIO_MUX6_IO08 */
        1U);                                /* Software Input On Field: Input Path is determined by functionality */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_LPSR_09_GPIO_MUX6_IO09, /* GPIO_LPSR_09 is configured as GPIO_MUX6_IO09 */
        1U);                                /* Software Input On Field: Input Path is determined by functionality */
}

/**
 * @brief configs gpios for getting  fpga type
 * 
 */
static void BOARD_InitFPGATypeConfigPins(void)
{
    CLOCK_EnableClock(kCLOCK_Iomuxc); /* LPCG on: LPCG is ON. */

    /* GPIO configuration of gpio2_09_pinL1 on GPIO_EMC_B1_41 (pin L1) */
    gpio_pin_config_t gpio2_09_pinL1_config = {
        .direction = kGPIO_DigitalInput,
        .outputLogic = 0U,
        .interruptMode = kGPIO_NoIntmode};
    /* Initialize GPIO functionality on GPIO_EMC_B1_41 (pin L1) */
    GPIO_PinInit(GPIO2, 9U, &gpio2_09_pinL1_config);

    /* GPIO configuration of gpio4_13_pinF14 on GPIO_SD_B2_04 (pin F14) */
    gpio_pin_config_t gpio4_13_pinF14_config = {
        .direction = kGPIO_DigitalInput,
        .outputLogic = 0U,
        .interruptMode = kGPIO_NoIntmode};
    /* Initialize GPIO functionality on GPIO_SD_B2_04 (pin F14) */
    GPIO_PinInit(GPIO4, 13U, &gpio4_13_pinF14_config);

    /* GPIO configuration of gpio4_14_pinE14 on GPIO_SD_B2_05 (pin E14) */
    gpio_pin_config_t gpio4_14_pinE14_config = {
        .direction = kGPIO_DigitalInput,
        .outputLogic = 0U,
        .interruptMode = kGPIO_NoIntmode};
    /* Initialize GPIO functionality on GPIO_SD_B2_05 (pin E14) */
    GPIO_PinInit(GPIO4, 14U, &gpio4_14_pinE14_config);

    IOMUXC_SetPinMux(
        IOMUXC_GPIO_EMC_B1_41_GPIO_MUX2_IO09, /* GPIO_EMC_B1_41 is configured as GPIO_MUX2_IO09 */
        1U);                                  /* Software Input On Field: Force input path of pad GPIO_EMC_B1_41 */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B2_04_GPIO_MUX4_IO13, /* GPIO_SD_B2_04 is configured as GPIO_MUX4_IO13 */
        1U);                                 /* Software Input On Field: Force input path of pad GPIO_SD_B2_04 */
    IOMUXC_SetPinMux(
        IOMUXC_GPIO_SD_B2_05_GPIO_MUX4_IO14, /* GPIO_SD_B2_05 is configured as GPIO_MUX4_IO14 */
        1U);                                 /* Software Input On Field: Force input path of pad GPIO_SD_B2_05 */
    IOMUXC_GPR->GPR40 = ((IOMUXC_GPR->GPR40 &
                          (~(BOARD_INITFPGACONFIGPINS_IOMUXC_GPR_GPR40_GPIO_MUX2_GPIO_SEL_LOW_MASK))) /* Mask bits to zero which are setting */
                         | IOMUXC_GPR_GPR40_GPIO_MUX2_GPIO_SEL_LOW(0x00U)                             /* GPIO2 and CM7_GPIO2 share same IO MUX function, GPIO_MUX2 selects one GPIO function: 0x00U */
    );
}

/**
 * @brief get fpga tye value
 * 说明：这个函数调用了BOARD_InitFPGATypeConfigPins(),使用了NXP官方接口去初始化引脚，更具有通用性，如
 * 果需要在RT-Thread环境下获取FPGA类型值，可以使用RT-Thread的通用接口rt_pin_xxx，去初始化和读取引脚的值
 * 
 * @return return the fpga tye value
 */
int check_fpga_config(void)
{
    BOARD_InitFPGATypeConfigPins();

    int fpga_config0_pin_value;
    int fpga_config1_pin_value;
    int fpga_config2_pin_value;

    fpga_config0_pin_value = GPIO_PinReadPadStatus(FPGA_CONFIG0_GPIO_PORT, FPGA_CONFIG0_GPIO_PIN);
    fpga_config1_pin_value = GPIO_PinReadPadStatus(FPGA_CONFIG1_GPIO_PORT, FPGA_CONFIG1_GPIO_PIN);
    fpga_config2_pin_value = GPIO_PinReadPadStatus(FPGA_CONFIG2_GPIO_PORT, FPGA_CONFIG2_GPIO_PIN);

    int fpga_type_value = (fpga_config2_pin_value << 2) + (fpga_config1_pin_value << 1) + fpga_config0_pin_value;
    return fpga_type_value;
}

static void shift_clk(uint32_t counter)
{
    int i;
    for (i = 0; i < counter; i++)
    {
        GPIO_WritePinOutput(FPGA_DCLK_GPIO_PORT, FPGA_DCLK_GPIO_PIN, 0);
        GPIO_WritePinOutput(FPGA_DCLK_GPIO_PORT, FPGA_DCLK_GPIO_PIN, 1);
    }
}

static void shift_clk_reverse(uint32_t counter)
{
    int i;
    for (i = 0; i < counter; i++)
    {
        GPIO_WritePinOutput(FPGA_DCLK_GPIO_PORT, FPGA_DCLK_GPIO_PIN, 1);
        GPIO_WritePinOutput(FPGA_DCLK_GPIO_PORT, FPGA_DCLK_GPIO_PIN, 0);
    }
}

static void ShiftOneByte(char data)
{
    int i;
    for (i = 0; i < 8;i++)
    {
        GPIO_WritePinOutput(FPGA_DATA0_GPIO_PORT, FPGA_DATA0_GPIO_PIN, ((data >> i) & 0x01));
        shift_clk(1);
    }
}

static uint8_t check_gpio_value(GPIO_Type *base, uint32_t pin)
{
    return GPIO_PinReadPadStatus(base, pin);
}

/**
 * @brief config fpga firmwre
 * 此处实现需要依托SD卡文件系统，在系统挂载SD后，从SD中读取fpga固件文件，然后写入FPGA
 * 
 * @return 0 -- OK, otherwise failed
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>

#define DBG_TAG "fpga.firmware"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
int config_fpga_firmware(void)
{
    struct stat st;
    int ret;
    int i;
    uint32_t file_len;
    char buf;
    uint8_t nstatus_ok = 0;
    uint8_t conf_done_ok = 0;

    BOARD_InitFPGAFirmWareConfigPins();

    DIR *dirp;

    /* 打开 /目录 */
    do
    {
        dirp = opendir("/");
    } while (dirp == RT_NULL);
        
    closedir(dirp);

    ret = stat(FPGA_FIREWARM_FILE_PATH, &st);
    if(ret == -1)
    {
        LOG_E("get stat info failed, file: %s\n", FPGA_FIREWARM_FILE_PATH);
        return ret;
    }

    int fd = open(FPGA_FIREWARM_FILE_PATH, O_RDONLY);

    if(fd == -1)
    {
        LOG_E("open failed\n");
        return -1;
    }
    else
    {
        LOG_D("open %s ok\n", FPGA_FIREWARM_FILE_PATH);
    }

    file_len = st.st_size;
    LOG_D("fpga firmware file size: %u KB\n", file_len/1024);
    // char *buf = rt_malloc(file_len);
    // if(buf == RT_NULL)
    // {
    //     LOG_E("no memory for configuration\n");
    //     return -1;
    // }

    LOG_D("**** Start configuration process *****************\n");

    //drive NCONFIG 0 to 1
    GPIO_WritePinOutput(FPGA_NCONFIG_GPIO_PORT, FPGA_NCONFIG_GPIO_PIN, 0);
    GPIO_WritePinOutput(FPGA_NCONFIG_GPIO_PORT, FPGA_NCONFIG_GPIO_PIN, 1);

    for(i = 0; i < file_len; i++)
    {
        // memset(buf, 0, 8);
        int read_num = read(fd, &buf, 1);
        if(read_num == -1)
        {
            printf("Failed to read\n");
            return -1;
        }

        ShiftOneByte(buf);

        /* Check for error through NSTATUS for every 10KB programmed */
        if (!(i % CHECK_EVERY_X_BYTE))
        {
            nstatus_ok = check_gpio_value(FPGA_NSTATUS_GPIO_PORT, FPGA_NSTATUS_GPIO_PIN);

            if (!nstatus_ok)
            {
                LOG_E("nstatus is pulled low, configure failed\n");
                return -1;
            }
        }
    }
    
    conf_done_ok = check_gpio_value(FPGA_CONF_DONE_GPIO_PORT, FPGA_CONF_DONE_GPIO_PIN);
    if(!conf_done_ok)
    {
        LOG_E("conf_done_ok is pulled low, fpga can not be conf_done \n");
        return -1;
    }

    /* Start initialization */
    /* Clock another (INIT_CYCLE) cycles to initialize the device */
    shift_clk_reverse(INIT_CYCLE);

    // rt_hw_us_delay(2000);
    /* add another 'x' clock cycles to make sure the device is initialized (certain cases) */
    // shift_clk(5);

    LOG_D("**** Configuration process end *****************\n");
    return 0;
}