/*
 * Copyright (c) 2001-2011 Stephane Raimbault <stephane.raimbault@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef _MODBUS_PRIVATE_H_
#define _MODBUS_PRIVATE_H_

#ifndef _MSC_VER
# include <stdint.h>
# include <sys/time.h>
#else
# include "stdint.h"
# include <time.h>
typedef int ssize_t;
#endif
#include <sys/types.h>
//#include <config.h>

#include "modbus.h"

MODBUS_BEGIN_DECLS

/* It's not really the minimal length (the real one is report slave ID
 * in RTU (4 bytes)) but it's a convenient size to use in RTU or TCP
 * communications to read many values or write a single one.
 * Maximum between :
 * - HEADER_LENGTH_TCP (7) + function (1) + address (2) + number (2)
 * - HEADER_LENGTH_RTU (1) + function (1) + address (2) + number (2) + CRC (2)
 * - HEADER_LENGTH_ASC (3) + function (2) + address (4) + number (4) + CRC (2) + CR (1) + LF (1)
 */
#define _MIN_REQ_LENGTH 17

#define _REPORT_SLAVE_ID 180

#define _MODBUS_EXCEPTION_RSP_LENGTH 5

/* Timeouts in microsecond (0.5 s) */
#define _RESPONSE_TIMEOUT    500000
#define _BYTE_TIMEOUT        500000

/* Function codes */
#define _FC_READ_COILS                0x01
#define _FC_READ_DISCRETE_INPUTS      0x02
#define _FC_READ_HOLDING_REGISTERS    0x03
#define _FC_READ_INPUT_REGISTERS      0x04
#define _FC_WRITE_SINGLE_COIL         0x05
#define _FC_WRITE_SINGLE_REGISTER     0x06
#define _FC_READ_EXCEPTION_STATUS     0x07
#define _FC_WRITE_MULTIPLE_COILS      0x0F
#define _FC_WRITE_MULTIPLE_REGISTERS  0x10
#define _FC_REPORT_SLAVE_ID           0x11
#define _FC_WRITE_AND_READ_REGISTERS  0x17
/* modbus 0x2B ������*/
#define _FC_READ_DEVICE_INFO            0x2B                  // ��ȡ�豸��ʶ������ʶ����

#define USE_FUNCTION_0X2B
#define _MODBUS_RTU_PRESET_REQ_LENGTH_DEVICE_ID  5          // ���ĳ��� - CRC
#define _MODBUS_RTU_RESPONSE_0X2B_HEADER   8				// ��Ӧ���ģ�������֡����

typedef enum {
	_MODBUS_BACKEND_TYPE_RTU = 0, _MODBUS_BACKEND_TYPE_TCP
} modbus_bakend_type_t;

typedef enum {
	_MODBUS_SERIAL = 0, _MODBUS_TCP, _MODBUS_ASCII
}modbus_channel_type_t;

/* This structure reduces the number of params in functions and so
 * optimizes the speed of execution (~ 37%). */
typedef struct _sft {
	int slave;
	int function;
	int t_id;
} sft_t;

typedef struct _modbus_backend {
	unsigned int backend_type;
	unsigned int header_length;
	unsigned int checksum_length;
	unsigned int max_adu_length;
	int (*set_slave)(modbus_t *ctx, int slave);
	int (*build_request_basis)(modbus_t *ctx, int function, int addr, int nb, uint8_t *req);
	int (*build_response_basis)(sft_t *sft, uint8_t *rsp);
	int (*prepare_response_tid)(const uint8_t *req, int *req_length);
	int (*send_msg_pre)(uint8_t *req, int req_length);
	ssize_t (*send)(modbus_t *ctx, const uint8_t *req, int req_length);
	ssize_t (*recv)(modbus_t *ctx, uint8_t *rsp, int rsp_length);
	int (*recv_msg_post)(uint8_t *rsq, int rsq_length);
	int (*check_integrity)(modbus_t *ctx, uint8_t *msg, const int msg_length);
	int (*pre_check_confirmation)(modbus_t *ctx, const uint8_t *req, const uint8_t *rsp, int rsp_length);
	int (*connect)(modbus_t *ctx);
	void (*close)(modbus_t *ctx);
	int (*flush)(modbus_t *ctx);
	int (*select)(modbus_t *ctx, fd_set *rfds, struct timeval *tv, int msg_length);
	int (*filter_request)(modbus_t *ctx, int slave);
} modbus_backend_t;

struct _modbus {
	/* Slave address */
	int slave;
	/* Socket or file descriptor */
	int s;
	int debug;
	int error_recovery;
	modbus_channel_type_t channel_type;
	struct timeval response_timeout;
	struct timeval byte_timeout;
	const modbus_backend_t *backend;
	void *backend_data;
};

void _modbus_init_common(modbus_t *ctx, modbus_channel_type_t type);
void _error_print(modbus_t *ctx, const char *context);

_Bool _modbus_asc2hex(const uint8_t *from, uint8_t *to);
_Bool _modbus_hex2asc(uint8_t *to, uint8_t from);

#ifndef HAVE_STRLCPY
size_t strlcpy(char *dest, const char *src, size_t dest_size);
#endif

MODBUS_END_DECLS

#endif  /* _MODBUS_PRIVATE_H_ */
