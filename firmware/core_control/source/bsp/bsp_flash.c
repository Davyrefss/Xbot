/**
 * @file bsp_flash.c
 * @author Davyrefss
 * @brief flash bsp
 *
 */

/* include files */
#include <stdint.h>
#include <string.h>
#include "bsp_flash.h"

/* defines */
#define FLASH_PARAM_VERSION     1u

/* typedefs */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint16_t crc16;
    uint16_t reserved;
} flash_param_header_t;     /* 页内布局 */

/* max len */
#define FLASH_PARAM_MAX_LEN (FLASH_PAGE_SIZE_T - sizeof(flash_param_header_t))

/* static variables */
static uint8_t s_page_buf[FLASH_PAGE_SIZE_T]; /* 页缓冲 */

/* static functions declaration */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len);
static bool flash_program_halfwords(uint32_t addr, const uint8_t *data, uint32_t len);

/* functions implementation */
/**
 * @brief CRC16-CCITT校验算法
 * @param data 数据指针
 * @param len 数据长度
 * @return 计算得到的CRC16值
 */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/**
 * @brief 半字节编程函数
 * @param addr 地址
 * @param data 数据指针
 * @param len 数据长度
 * @return 操作是否成功
 */
static bool flash_program_halfwords(uint32_t addr, const uint8_t *data, uint32_t len) {
    bool ok = true;
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < len; i += 2u) {
        uint16_t half = (uint16_t)data[i];
        half |= (i + 1u < len) ? (uint16_t)((uint16_t)data[i + 1u] << 8) : 0xFF00u;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, half) != HAL_OK) {
            ok = false;
            break;
        }
    }
    HAL_FLASH_Lock();
    return ok;
}

/**
 * @brief 闪存擦除函数
 * @param start_addr 起始地址
 * @param page_count 页数
 * @return 操作是否成功
 */
bool flash_erase(uint32_t start_addr, uint32_t page_count) {
    if ((start_addr < FLASH_BASE_ADDR) ||
        (start_addr + page_count * FLASH_PAGE_SIZE_T > FLASH_END_ADDR) ||
        ((start_addr % FLASH_PAGE_SIZE_T) != 0u)) {
        return false;
    }
    FLASH_EraseInitTypeDef erase = {
        .TypeErase   = FLASH_TYPEERASE_PAGES,
        .Banks       = FLASH_BANK_1,
        .PageAddress = start_addr,
        .NbPages     = page_count,
    };
    uint32_t page_error = 0u;
    HAL_FLASH_Unlock();
    bool ok = (HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK);
    HAL_FLASH_Lock();
    return ok;
}

/**
 * @brief 闪存写入函数
 * @param addr 地址
 * @param data 数据指针
 * @param len 数据长度
 * @return 操作是否成功
 */
bool flash_write(uint32_t addr, const void *data, uint32_t len) {
    uint32_t page  = addr & ~(FLASH_PAGE_SIZE_T - 1u);
    uint32_t pages = ((addr + len - page) + FLASH_PAGE_SIZE_T - 1u) / FLASH_PAGE_SIZE_T;
    if (!flash_erase(page, pages)) {
        return false;
    }
    return flash_program_halfwords(addr, data, len);
}

/**
 * @brief 闪存读取函数
 * @param addr 地址
 * @param buf 缓冲区指针
 * @param len 数据长度
 */
void flash_read(uint32_t addr, void *buf, uint32_t len) {
    memcpy(buf, (const void *)addr, len);   /* Flash  memory-mapped，直接读 */
}

/**
 * @brief 参数保存函数
 * @param data 数据指针
 * @param len 数据长度
 * @return 操作是否成功
 */
bool flash_param_save(const void *data, uint16_t len) {
    if ((data == NULL) || (len == 0u) || (len > FLASH_PARAM_MAX_LEN)) {
        return false;
    }
    flash_param_header_t *hdr = (flash_param_header_t *)s_page_buf;
    hdr->magic    = FLASH_PARAM_MAGIC;
    hdr->version  = FLASH_PARAM_VERSION;
    hdr->length   = len;
    hdr->crc16    = crc16_ccitt(data, len);
    hdr->reserved = 0xFFFFu;
    memcpy(s_page_buf + sizeof(*hdr), data, len);

    /* 内容没变就不写：省一次约 20~40ms 的整片停顿 + 省擦写寿命 */
    if (memcmp((const void *)FLASH_PARAM_ADDR, s_page_buf,
               sizeof(*hdr) + len) == 0) {
        return true;
    }
    if (!flash_erase(FLASH_PARAM_ADDR, 1u)) {
        return false;
    }
    return flash_program_halfwords(FLASH_PARAM_ADDR, s_page_buf, sizeof(*hdr) + len);
}

/**
 * @brief 参数加载函数
 * @param data 数据指针
 * @param len 数据长度
 * @return 操作是否成功
 */
bool flash_param_load(void *data, uint16_t len) {
    if ((data == NULL) || (len == 0u) || (len > FLASH_PARAM_MAX_LEN)) {
        return false;
    }
    const flash_param_header_t *hdr = (const flash_param_header_t *)FLASH_PARAM_ADDR;
    /* 空页 / 结构升级导致长度变化 → 视为无效，调用方用默认值 */
    if ((hdr->magic != FLASH_PARAM_MAGIC) || (hdr->length != len)) {
        return false;
    }
    const uint8_t *payload = (const uint8_t *)FLASH_PARAM_ADDR + sizeof(*hdr);
    if (crc16_ccitt(payload, len) != hdr->crc16) {
        return false;   /* 数据损坏（比如擦后断电写了一半） */
    }
    memcpy(data, payload, len);
    return true;
}
