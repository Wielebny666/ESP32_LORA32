#ifndef __SX1276_H__
#define __SX1276_H__

void sx1276_init(void);
void sx1276_reset(void);
void sx1276_set_tx_packet(const void *buffer, uint16_t size);
void sx1276_set_opmode(uint8_t opMode);
uint32_t sx1276_process(void);

#endif