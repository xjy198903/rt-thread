/*
 * Copyright (c) 2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-09-19     Gavin        first version
 *
 */

#include <rtthread.h>
#ifdef RT_USING_DFS_RAMFS
#include <dfs_fs.h>

extern struct dfs_ramfs *dfs_ramfs_create(rt_uint8_t *pool, rt_size_t size);

int mnt_init(void)
{
    rt_uint8_t *pool = RT_NULL;
    rt_size_t size = 8*1024*1024;

    pool = rt_malloc(size);
    if (pool == RT_NULL)
        return 0;

    if (dfs_mount(RT_NULL, "/", "ram", 0, (const void *)dfs_ramfs_create(pool, size)) == 0)
        rt_kprintf("RAM file system initializated!\n");
    else
        rt_kprintf("RAM file system initializate failed!\n");

    return 0;
}
INIT_ENV_EXPORT(mnt_init);
#endif

#ifdef BSP_USING_SDCARD_FATFS
#include <dfs_fs.h>
#include <dfs_file.h>
#include <dfs_romfs.h>
#define DBG_TAG "app.filesystem"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
// static int filesystem_mount(void)
// {
//     while(rt_device_find("sd0") == RT_NULL)
//     {
//         rt_thread_mdelay(1);
//     }
  
//     int ret = dfs_mount("sd0", "/", "elm", 0, 0);
//     if (ret != 0)
//     {
//         rt_kprintf("ret: %d\n",ret);
//         LOG_E("sd0 mount to '/' failed!");
//         return ret;
//     }

//     return RT_EOK;
// }

static int onboard_sdcard_mount(void)
{
    while (rt_device_find("sd0") == RT_NULL)
    {
        rt_thread_mdelay(1);
    }
    if (dfs_mount("sd0p1", "/sdcard", "elm", 0, 0) == RT_EOK)
    {
        LOG_I("SD card mount to '/sdcard'");
    }
    else
    {
        LOG_E("SD card mount to '/sdcard' failed!");
    }

    return RT_EOK;
}

static const struct romfs_dirent _cnc_romfs_root[] =
{
#ifdef BSP_USING_SDCARD_FATFS
        {ROMFS_DIRENT_DIR, "sdcard", RT_NULL, 0},
#endif

#ifdef RT_USBH_MSTORAGE
        {ROMFS_DIRENT_DIR, "udisk", RT_NULL, 0},
#endif
};

const struct romfs_dirent romfs_root =
{
    ROMFS_DIRENT_DIR, "/", (rt_uint8_t *)_cnc_romfs_root, sizeof(_cnc_romfs_root) / sizeof(_cnc_romfs_root[0])
};

static int filesystem_mount(void)
{
    if (dfs_mount(RT_NULL, "/", "rom", 0, &(romfs_root)) != 0)
    {
        LOG_E("rom mount to '/' failed!");
    }
#ifdef BSP_USING_SDCARD_FATFS
    onboard_sdcard_mount();
#endif
    return RT_EOK;
}
INIT_ENV_EXPORT(filesystem_mount);
#endif
