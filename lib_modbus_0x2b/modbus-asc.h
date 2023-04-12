/*
 * Made in China
 */

#ifndef _MODBUS_ASC_H_
#define _MODBUS_ASC_H_

#include "modbus.h"

MODBUS_BEGIN_DECLS

/* Modbus_Application_Protocol_V1_1b.pdf Chapter 4 Section 1 Page 5
 * RS232 / RS485 ADU = 506 bytes + slave (2 byte) + CRC (2 bytes) + start (1 byte) + LF (1 byte) + CR (1 byte) = 513 bytes
 */
#define MODBUS_ASC_MAX_ADU_LENGTH  513

modbus_t* modbus_new_asc(const char *device, int baud, char parity,
                         int data_bit, int stop_bit);

#define MODBUS_ASC_RS232 0
#define MODBUS_ASC_RS485 1

int modbus_asc_set_serial_mode(modbus_t *ctx, int mode);
int modbus_asc_get_serial_mode(modbus_t *ctx);

MODBUS_END_DECLS

#endif /* _MODBUS_ASC_H_ */
