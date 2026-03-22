#ifndef CH554T_H
#define CH554T_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

bool ch554t_spi_init(uint8_t ch_type, uint8_t i2cBusSpeed);
void ch554t_spi_shutdown(void);
int ch554t_get_descriptor(uint8_t *buf);
int ch554tGetDescriptor(uint8_t *buf);

int ch554t_read_devid(uint8_t *rxbuf, int n_rx);
int ch554t_read_data(uint32_t from, uint8_t *buf, uint32_t len);
int ch554t_write_data(uint32_t to, const uint8_t *buf, uint32_t len);
int ch554t_chip_erase_start(void);
int ch554t_chip_erase_poll(void);
int ch554t_chip_erase_finalize(void);
int ch554t_chip_erase(void);

#ifdef __cplusplus
}
#endif

#endif
