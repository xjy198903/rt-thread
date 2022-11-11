#include <rtthread.h>
#include <rtdevice.h>
#define DBG_TAG "cnc_can"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "cnc_can.h"
#include <string.h>

#ifdef BSP_USING_CAN

static rt_device_t can_dev;
static struct rt_semaphore rx_sem;     /* 用于接收消息的信号量 */
static struct rt_can_status status;    /* 获取到的 CAN 总线状态 */

/* 接收数据回调函数 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem);

    return RT_EOK;
}

void can_comm_entry(void *arg)
{
    rt_err_t res;
    struct rt_can_msg rxmsg = {0};
    int i;

    can_dev = rt_device_find(CAN_DEVICE_NAME);
    if (can_dev == RT_NULL)
    {
        LOG_E("can not find device %s\n", CAN_DEVICE_NAME);
        return ;
    }

     /* 初始化 CAN 接收信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    
    /* 以中断接收及发送方式打开 CAN 设备 */
    res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    if(res != RT_EOK)
    {
        LOG_E("open can_dev failed: %s\n", strerror(errno));
        return ;
    }

    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(can_dev, can_rx_call);

#ifdef RT_CAN_USING_HDR
    struct rt_can_filter_item items[5] =
    {
        RT_CAN_FILTER_ITEM_INIT(0x100, 0, 0, 0, 0x700, RT_NULL, RT_NULL), /* std,match ID:0x100~0x1ff，hdr 为 - 1，设置默认过滤表 */
        RT_CAN_FILTER_ITEM_INIT(0x300, 0, 0, 0, 0x700, RT_NULL, RT_NULL), /* std,match ID:0x300~0x3ff，hdr 为 - 1 */
        RT_CAN_FILTER_ITEM_INIT(0x211, 0, 0, 0, 0x7ff, RT_NULL, RT_NULL), /* std,match ID:0x211，hdr 为 - 1 */
        RT_CAN_FILTER_STD_INIT(0x486, RT_NULL, RT_NULL),                  /* std,match ID:0x486，hdr 为 - 1 */
        {0x555, 0, 0, 0, 0x7ff, 7,}                                       /* std,match ID:0x555，hdr 为 7，指定设置 7 号过滤表 */
    };
    struct rt_can_filter_config cfg = {5, 1, items}; /* 一共有 5 个过滤表 */
    /* 设置硬件过滤表 */
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    RT_ASSERT(res == RT_EOK);
#endif

    /* 获取 CAN 总线设备的状态 */
    res = rt_device_control(can_dev, RT_CAN_CMD_GET_STATUS, &status);
    if(res != RT_EOK)
    {
        LOG_E("can_control failed: %d", res);
    }

    struct rt_can_msg msg = {0}; /* CAN 消息 */
    msg.id = 0x78;               /* ID 为 0x78 */
    msg.ide = RT_CAN_STDID;      /* 标准格式 */
    msg.rtr = RT_CAN_DTR;        /* 数据帧 */
    msg.len = 8;                 /* 数据长度为 8 */
    /* 待发送的 8 字节数据 */
    msg.data[0] = 0x00;
    msg.data[1] = 0x11;
    msg.data[2] = 0x22;
    msg.data[3] = 0x33;
    msg.data[4] = 0x44;
    msg.data[5] = 0x55;
    msg.data[6] = 0x66;
    msg.data[7] = 0x77;
    /* 发送一帧 CAN 数据 */
    rt_size_t size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
    rt_kprintf("send %d bytes\n", size);

    while (1)
    {
        /* hdr 值为 - 1，表示直接从 uselist 链表读取数据 */
        rxmsg.hdr = -1;
        /* 阻塞等待接收信号量 */
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        /* 从 CAN 读取一帧数据 */
        rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
        /* 打印数据 ID 及内容 */
        rt_kprintf("ID:%x", rxmsg.id);
        for (i = 0; i < 8; i++)
        {
            rt_kprintf("%2x", rxmsg.data[i]);
        }

        rt_kprintf("\n");
    }

}
#endif