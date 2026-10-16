/**
 * @file bsp_flash.h
 * @author Davyrefss
 * @brief flash bsp
 *
 */

#ifndef BSP_FLASH_H
#define BSP_FLASH_H

/* include files */
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* defines */
/* Flash 布局 */
#define FLASH_PAGE_SIZE_T     2048u
#define FLASH_BASE_ADDR     0x08000000u
#define FLASH_END_ADDR      0x08040000u   /* 256KB */
#define FLASH_PARAM_ADDR    (FLASH_END_ADDR - FLASH_PAGE_SIZE_T)  /* 参数区：最后一页 0x0803F800 ~ 0x08040000 */

/* 记录头魔数：'XBOT'，用于区分"有效数据"和"空页/野数据" */
#define FLASH_PARAM_MAGIC   0x58424F54u

/* functions declaration */
/* 低层接口 */
bool flash_erase(uint32_t start_addr, uint32_t page_count);
bool flash_write(uint32_t addr, const void *data, uint32_t len);
void flash_read(uint32_t addr, void *buf, uint32_t len);

/* 参数块接口：自动带 魔数 + 长度 + CRC16 校验 */
bool flash_param_save(const void *data, uint16_t len);
bool flash_param_load(void *data, uint16_t len);

#endif /* BSP_FLASH */
