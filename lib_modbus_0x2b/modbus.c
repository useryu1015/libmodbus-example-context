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
 *
 *
 * This library implements the Modbus protocol.
 * http://libmodbus.org/
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <time.h>
#include <termios.h>
//#include <config.h>

#include "zlog.h"
#include "modbus.h"
#include "modbus-private.h"

#include <assert.h>             // 2022年7月18日 添加


/* Internal use */
#define MSG_LENGTH_UNDEFINED -1

/* Exported version */
const unsigned int libmodbus_version_major = LIBMODBUS_VERSION_MAJOR;
const unsigned int libmodbus_version_minor = LIBMODBUS_VERSION_MINOR;
const unsigned int libmodbus_version_micro = LIBMODBUS_VERSION_MICRO;

/* Max RTU、TCP、ASC max adu length (so ASC) */
#define MAX_MESSAGE_LENGTH 513

/* 3 steps are used to parse the query */
typedef enum {
	// _STEP_FUNCTION, _STEP_META, _STEP_DATA
	_STEP_FUNCTION, _STEP_META, _STEP_INFO, _STEP_DATA, _STEP_CRC, _STEP_END
} _step_t;

const char *modbus_strerror(int errnum) {
	switch (errnum) {
	case EMBXILFUN:
		return "Illegal function";
	case EMBXILADD:
		return "Illegal data address";
	case EMBXILVAL:
		return "Illegal data value";
	case EMBXSFAIL:
		return "Slave device or server failure";
	case EMBXACK:
		return "Acknowledge";
	case EMBXSBUSY:
		return "Slave device or server is busy";
	case EMBXNACK:
		return "Negative acknowledge";
	case EMBXMEMPAR:
		return "Memory parity error";
	case EMBXGPATH:
		return "Gateway path unavailable";
	case EMBXGTAR:
		return "Target device failed to respond";
	case EMBBADCRC:
		return "Invalid CRC";
	case EMBBADDATA:
		return "Invalid data";
	case EMBBADEXC:
		return "Invalid exception code";
	case EMBMDATA:
		return "Too many data";
	default:
		return strerror(errnum);
	}
}

int _sleep_and_flush(modbus_t *ctx) {
#ifdef _WIN32
	/* usleep doesn't exist on Windows */
	Sleep((ctx->response_timeout.tv_sec * 1000) +
			(ctx->response_timeout.tv_usec / 1000));
#else
	/* usleep source code */
	struct timespec request, remaining;
	request.tv_sec = ctx->response_timeout.tv_sec;
	request.tv_nsec = ((long int) ctx->response_timeout.tv_usec % 1000000)
			* 1000;
	while (nanosleep(&request, &remaining) == -1 && errno == EINTR)
		request = remaining;
#endif
	return modbus_flush(ctx);
}

int modbus_flush(modbus_t *ctx) {
	int rc = ctx->backend->flush(ctx);
	if (rc != -1 && ctx->debug) {
		zlog_trace("%d bytes flushed", rc);
	}
	return rc;
}

/* Computes the length of the expected response */
static unsigned int compute_response_length_from_request(modbus_t *ctx,
		uint8_t *req) {
	int length;
	const int offset = ctx->backend->header_length;

	switch (req[offset]) {
	case _FC_READ_COILS:
	case _FC_READ_DISCRETE_INPUTS: {
		/* Header + nb values (code from write_bits) */
		int nb = (req[offset + 3] << 8) | req[offset + 4];
		length = 2 + (nb / 8) + ((nb % 8) ? 1 : 0);
	}
		break;
	case _FC_WRITE_AND_READ_REGISTERS:
	case _FC_READ_HOLDING_REGISTERS:
	case _FC_READ_INPUT_REGISTERS:
		/* Header + 2 * nb values */
		length = 2 + 2 * (req[offset + 3] << 8 | req[offset + 4]);
		break;
	case _FC_READ_EXCEPTION_STATUS:
		length = 3;
		break;
	case _FC_REPORT_SLAVE_ID:
		/* The response is device specific (the header provides the
		 length) */
		return MSG_LENGTH_UNDEFINED;
	default:
		length = 5;
	}

	return offset + length + ctx->backend->checksum_length;
}

/* Sends a request/response */
static int send_msg(modbus_t *ctx, uint8_t *msg, int msg_length) {
	int rc;
	int i;

	msg_length = ctx->backend->send_msg_pre(msg, msg_length);

	if (ctx->debug) {
		hflog_trace(msg, msg_length, ctx->s);
	}

	if (ctx->debug) {
		printf(" modbus send data: [%d]:    \n",msg_length);    
		for (i=0; i < msg_length ; i++)
			printf("<%.2X>", msg[i]);			// 打印接收的数据、
		printf(
"\n");
	}

	/* In recovery mode, the write command will be issued until to be
	 successful! Disabled by default. */
	do {
		rc = ctx->backend->send(ctx, msg, msg_length);
		if (rc == -1) {
			zlog_debug("%s:", modbus_strerror(errno));
			if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) {
				int saved_errno = errno;

				if ((errno == EBADF || errno == ECONNRESET || errno == EPIPE)) {
					modbus_close(ctx);
					modbus_connect(ctx);
				} else {
					_sleep_and_flush(ctx);
				}
				errno = saved_errno;
			}
		}
	} while ((ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) && rc == -1);

	if (rc > 0 && rc != msg_length) {
		errno = EMBBADDATA;
		return -1;
	}

	return rc;
}

int modbus_send_raw_request(modbus_t *ctx, uint8_t *raw_req, int raw_req_length) {
	sft_t sft;
	uint8_t req[MAX_MESSAGE_LENGTH];
	int req_length;

	if (raw_req_length < 2) {
		/* The raw request must contain function and slave at least */
		errno = EINVAL;
		return -1;
	}

	sft.slave = raw_req[0];
	sft.function = raw_req[1];
	/* The t_id is left to zero */
	sft.t_id = 0;
	/* This response function only set the header so it's convenient here */
	req_length = ctx->backend->build_response_basis(&sft, req);

	if (raw_req_length > 2) {
		/* Copy data after function code */
		memcpy(req + req_length, raw_req + 2, raw_req_length - 2);
		req_length += raw_req_length - 2;
	}

	return send_msg(ctx, req, req_length);
}

/*
 ---------- Request     Indication ----------
 | Client | ---------------------->| Server |
 ---------- Confirmation  Response ----------
 */

typedef enum {
	/* Request message on the server side */
	MSG_INDICATION,
	/* Request message on the client side */
	MSG_CONFIRMATION
} msg_type_t;

/* Computes the length to read after the function received */
static uint8_t compute_meta_length_after_function(int function,
		msg_type_t msg_type, modbus_channel_type_t channel_type) {
	int length;

	if (msg_type == MSG_INDICATION) {
		if (function <= _FC_WRITE_SINGLE_REGISTER) {
			length = 4;
		} else if (function == _FC_WRITE_MULTIPLE_COILS
				|| function == _FC_WRITE_MULTIPLE_REGISTERS) {
			length = 5;
		} else if (function == _FC_WRITE_AND_READ_REGISTERS) {
			length = 9;
		} else {
			/* _FC_READ_EXCEPTION_STATUS, _FC_REPORT_SLAVE_ID */
			length = 0;
		}
	} else {
		/* MSG_CONFIRMATION */
		switch (function) {
		case _FC_WRITE_SINGLE_COIL:
		case _FC_WRITE_SINGLE_REGISTER:
		case _FC_WRITE_MULTIPLE_COILS:
		case _FC_WRITE_MULTIPLE_REGISTERS:
			length = 4;
			break;
		default:
			length = 1;
		}
	}

	if (_MODBUS_ASCII == channel_type)
		length *= 2;

	return length;
}

/* Computes the length to read after the meta information (address, count, etc) */
static int compute_data_length_after_meta(modbus_t *ctx, uint8_t *msg,
		msg_type_t msg_type, modbus_channel_type_t channel_type) {
	uint8_t function;
	int length;

	if (_MODBUS_ASCII == channel_type)
		_modbus_asc2hex(&msg[ctx->backend->header_length], &function);
	else
		function = msg[ctx->backend->header_length];

	if (msg_type == MSG_INDICATION) {
		switch (function) {
		case _FC_WRITE_MULTIPLE_COILS:
		case _FC_WRITE_MULTIPLE_REGISTERS:
			if (_MODBUS_ASCII == channel_type) {
				uint8_t len;
				_modbus_asc2hex(&msg[ctx->backend->header_length + 10], &len);
				length = 2 * len;
			} else {
				length = msg[ctx->backend->header_length + 5];
			}
			break;
		case _FC_WRITE_AND_READ_REGISTERS:
			if (_MODBUS_ASCII == channel_type) {
				uint8_t len;
				_modbus_asc2hex(&msg[ctx->backend->header_length + 18], &len);
				length = 2 * len;
			} else {
				length = msg[ctx->backend->header_length + 9];
			}
			break;
		default:
			length = 0;
		}
	} else {
		/* MSG_CONFIRMATION */
		if (function <= _FC_READ_INPUT_REGISTERS
				|| function == _FC_REPORT_SLAVE_ID
				|| function == _FC_WRITE_AND_READ_REGISTERS) {
			if (_MODBUS_ASCII == channel_type) {
				uint8_t len;
				_modbus_asc2hex(&msg[ctx->backend->header_length + 2], &len);
				length = 2 * len;
			} else {
				length = msg[ctx->backend->header_length + 1];
			}
		} else {
			length = 0;
		}
	}

	length += ctx->backend->checksum_length;

	return length;
}


/* Waits a response from a modbus server or a request from a modbus client.
 This function blocks if there is no replies (3 timeouts).

 The function shall return the number of received characters and the received
 message in an array of uint8_t if successful. Otherwise it shall return -1
 and errno is set to one of the values defined below:
 - ECONNRESET
 - EMBBADDATA
 - EMBUNKEXC
 - ETIMEDOUT
 - read() or recv() error codes
 */
static int receive_msg(modbus_t *ctx, uint8_t *msg, msg_type_t msg_type) {
	int rc;
	int i;
	fd_set rfds;
	struct timeval tv;
	struct timeval *p_tv;
	int length_to_read;
	int msg_length = 0;
	_step_t step;

	if (ctx->debug) {
		if (msg_type == MSG_INDICATION) {
			zlog_trace("Waiting for a indication...");
		} else {
			zlog_trace("Waiting for a confirmation...");
		}
	}

	/* Add a file descriptor to the set */
	FD_ZERO(&rfds);
	FD_SET(ctx->s, &rfds);

	/* We need to analyse the message step by step.  At the first step, we want
	 * to reach the function code because all packets contain this
	 * information. */
	step = _STEP_FUNCTION;

	if (_MODBUS_ASCII == ctx->channel_type)
		length_to_read = ctx->backend->header_length + 2;
	else
		length_to_read = ctx->backend->header_length + 1;			// change： lib modbus根据请求的寄存器数量定义接收长度。 	 

	if (msg_type == MSG_INDICATION) {
		/* Wait for a message, we don't know when the message will be
		 * received */
		p_tv = NULL;
	} else {
		tv.tv_sec = ctx->response_timeout.tv_sec;
		tv.tv_usec = ctx->response_timeout.tv_usec;
		p_tv = &tv;
	}

	while (length_to_read != 0) {
		rc = ctx->backend->select(ctx, &rfds, p_tv, length_to_read);
		if (rc == -1) {
			zlog_debug("%s: select", modbus_strerror(errno));
			if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) {
				int saved_errno = errno;

				if (errno == ETIMEDOUT) {
					_sleep_and_flush(ctx);
				} else if (errno == EBADF) {
					modbus_close(ctx);
					modbus_connect(ctx);
				}
				errno = saved_errno;
			}
			return -1;
		}

		rc = ctx->backend->recv(ctx, msg + msg_length, length_to_read);
        if (ctx->debug) {
            printf(" recv one frame data[%d]:    \n",rc);    
            for (i=0; i < rc; i++)
                printf("<%.2X>", (msg + msg_length)[i]);          // 打印接收的数据
			printf("\n");
        }
		
		if (rc == 0) {
			errno = ECONNRESET;
			rc = -1;
		}

		if (rc == -1) {
			if (ctx->debug) {
				zlog_debug("%s: read", modbus_strerror(errno));
			}
			if ((ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK)
					&& (errno == ECONNRESET || errno == ECONNREFUSED ||
					errno == EBADF)) {
				int saved_errno = errno;
				modbus_close(ctx);
				modbus_connect(ctx);
				/* Could be removed by previous calls */
				errno = saved_errno;
			}
			return -1;
		}

		/* Sums bytes received */
		msg_length += rc;
		/* Computes remaining bytes */
		length_to_read -= rc;

		if (length_to_read == 0) {
			switch (step) {
			case _STEP_FUNCTION: {
				uint8_t function;

				if (_MODBUS_ASCII == ctx->channel_type)
					_modbus_asc2hex(&msg[ctx->backend->header_length],
							&function);
				else
					function = msg[ctx->backend->header_length];

				/* Function code position */
				length_to_read = compute_meta_length_after_function(function,
						msg_type, ctx->channel_type);
				if (length_to_read != 0) {
					step = _STEP_META;
					break;
				} /* else switches straight to the next step */
				/*no break*/
			}
			case _STEP_META:
				length_to_read = compute_data_length_after_meta(ctx, msg,
						msg_type, ctx->channel_type);
				if ((msg_length + length_to_read)
						> ctx->backend->max_adu_length) {
					errno = EMBBADDATA;
					zlog_debug("%s: too many data", modbus_strerror(errno));
					return -1;
				}
				step = _STEP_DATA;
				break;
			default:
				break;
			}
		}

		if (length_to_read > 0 && ctx->byte_timeout.tv_sec != -1) {
			/* If there is no character in the buffer, the allowed timeout
			 interval between two consecutive bytes is defined by
			 byte_timeout */
			tv.tv_sec = ctx->byte_timeout.tv_sec;
			tv.tv_usec = ctx->byte_timeout.tv_usec;
			p_tv = &tv;
		}
	}

	/* Display the hex code of each character received */
	if (ctx->debug) {
		hflog_trace(msg, msg_length, ctx->s);
	}

	msg_length = ctx->backend->recv_msg_post(msg, msg_length);
	if (msg_length < 0)
		return -1;

	return ctx->backend->check_integrity(ctx, msg, msg_length);
}


/* Receive the request from a modbus master */
int modbus_receive(modbus_t *ctx, uint8_t *req) {
	return receive_msg(ctx, req, MSG_INDICATION);
}

/* Receives the confirmation.

 The function shall store the read response in rsp and return the number of
 values (bits or words). Otherwise, its shall return -1 and errno is set.

 The function doesn't check the confirmation is the expected response to the
 initial request.
 */
int modbus_receive_confirmation(modbus_t *ctx, uint8_t *rsp) {
	return receive_msg(ctx, rsp, MSG_CONFIRMATION);
}

static int check_confirmation(modbus_t *ctx, uint8_t *req, int req_length,
		uint8_t *rsp, int rsp_length) {
	int rc;
	int rsp_length_computed;
	const int offset = ctx->backend->header_length;

	if (ctx->backend->pre_check_confirmation) {
		rc = ctx->backend->pre_check_confirmation(ctx, req, rsp, rsp_length);
		if (rc == -1) {
			if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
				_sleep_and_flush(ctx);
			}
			return -1;
		}
	}

	ctx->backend->recv_msg_post(req, req_length);

	rsp_length_computed = compute_response_length_from_request(ctx, req);

	/* Check length */
	if (rsp_length
			== rsp_length_computed|| rsp_length_computed == MSG_LENGTH_UNDEFINED) {
		int req_nb_value;
		int rsp_nb_value;
		const int function = rsp[offset];

		/* Check function code */
		if (function != req[offset]) {
			if (ctx->debug) {
				zlog_trace(
						"Received function not corresponding to the request (%d != %d)",
						function, req[offset]);
			}
			if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
				_sleep_and_flush(ctx);
			}
			errno = EMBBADDATA;
			return -1;
		}

		/* Check the number of values is corresponding to the request */
		switch (function) {
		case _FC_READ_COILS:
		case _FC_READ_DISCRETE_INPUTS:
			/* Read functions, 8 values in a byte (nb
			 * of values in the request and byte count in
			 * the response. */
			req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
			req_nb_value = (req_nb_value / 8) + ((req_nb_value % 8) ? 1 : 0);
			rsp_nb_value = rsp[offset + 1];
			break;
		case _FC_WRITE_AND_READ_REGISTERS:
		case _FC_READ_HOLDING_REGISTERS:
		case _FC_READ_INPUT_REGISTERS:
			/* Read functions 1 value = 2 bytes */
			req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
			rsp_nb_value = (rsp[offset + 1] / 2);
			break;
		case _FC_WRITE_MULTIPLE_COILS:
		case _FC_WRITE_MULTIPLE_REGISTERS:
			/* N Write functions */
			req_nb_value = (req[offset + 3] << 8) + req[offset + 4];
			rsp_nb_value = (rsp[offset + 3] << 8) | rsp[offset + 4];
			break;
		case _FC_REPORT_SLAVE_ID:
			/* Report slave ID (bytes received) */
			req_nb_value = rsp_nb_value = rsp[offset + 1];
			break;
		default:
			/* 1 Write functions & others */
			req_nb_value = rsp_nb_value = 1;
		}

		if (req_nb_value == rsp_nb_value) {
			rc = rsp_nb_value;
		} else {
			if (ctx->debug) {
				zlog_trace(
						"Quantity not corresponding to the request (%d != %d)",
						rsp_nb_value, req_nb_value);
			}

			if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
				_sleep_and_flush(ctx);
			}

			errno = EMBBADDATA;
			rc = -1;
		}
	} else if (rsp_length == (offset + 2 + ctx->backend->checksum_length)
			&& req[offset] == (rsp[offset] - 0x80)) {
		/* EXCEPTION CODE RECEIVED */

		int exception_code = rsp[offset + 1];
		if (exception_code < MODBUS_EXCEPTION_MAX) {
			errno = MODBUS_ENOBASE + exception_code;
		} else {
			errno = EMBBADEXC;
		}
		zlog_debug("%s", modbus_strerror(errno));
		rc = -1;
	} else {
		if (ctx->debug) {
			zlog_trace(
					"Message length not corresponding to the computed length (%d != %d)",
					rsp_length, rsp_length_computed);
		}
		if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
			_sleep_and_flush(ctx);
		}
		errno = EMBBADDATA;
		rc = -1;
	}

	return rc;
}

static int response_io_status(int address, int nb, uint8_t *tab_io_status,
		uint8_t *rsp, int offset) {
	int shift = 0;
	int byte = 0;
	int i;

	for (i = address; i < address + nb; i++) {
		byte |= tab_io_status[i] << shift;
		if (shift == 7) {
			/* Byte is full */
			rsp[offset++] = byte;
			byte = shift = 0;
		} else {
			shift++;
		}
	}

	if (shift != 0)
		rsp[offset++] = byte;

	return offset;
}

/* Build the exception response */
static int response_exception(modbus_t *ctx, sft_t *sft, int exception_code,
		uint8_t *rsp) {
	int rsp_length;

	sft->function = sft->function + 0x80;
	rsp_length = ctx->backend->build_response_basis(sft, rsp);

	/* Positive exception code */
	rsp[rsp_length++] = exception_code;

	return rsp_length;
}

/* Send a response to the received request.
 Analyses the request and constructs a response.

 If an error occurs, this function construct the response
 accordingly.
 */
int modbus_reply(modbus_t *ctx, const uint8_t *req, int req_length,
		modbus_mapping_t *mb_mapping) {
	int offset = ctx->backend->header_length;
	uint8_t slave;
	int function = req[offset];
	uint16_t address = (req[offset + 1] << 8) + req[offset + 2];
	uint8_t rsp[MAX_MESSAGE_LENGTH];
	int rsp_length = 0;
	sft_t sft;

	if (_MODBUS_ASCII == ctx->channel_type) {
		_modbus_asc2hex(&req[offset - 2], &slave);
	} else {
		slave = req[offset - 1];
	}

	if (ctx->backend->filter_request(ctx, slave) == 1) {
		/* Filtered */
		return 0;
	}

	sft.slave = slave;
	sft.function = function;
	sft.t_id = ctx->backend->prepare_response_tid(req, &req_length);

	switch (function) {
	case _FC_READ_COILS: {
		int nb = (req[offset + 3] << 8) + req[offset + 4];

		if ((address + nb) > mb_mapping->nb_bits) {
			if (ctx->debug) {
				zlog_trace("Illegal data address %0x in read_bits",
						address + nb);
			}
			rsp_length = response_exception(ctx, &sft,
					MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
		} else {
			rsp_length = ctx->backend->build_response_basis(&sft, rsp);
			rsp[rsp_length++] = (nb / 8) + ((nb % 8) ? 1 : 0);
			rsp_length = response_io_status(address, nb, mb_mapping->tab_bits,
					rsp, rsp_length);
		}
	}
		break;
	case _FC_READ_DISCRETE_INPUTS: {
		/* Similar to coil status (but too many arguments to use a
		 * function) */
		int nb = (req[offset + 3] << 8) + req[offset + 4];

		if ((address + nb) > mb_mapping->nb_input_bits) {
			if (ctx->debug) {
				zlog_trace("Illegal data address %0x in read_input_bits",
						address + nb);
			}
			rsp_length = response_exception(ctx, &sft,
					MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
		} else {
			rsp_length = ctx->backend->build_response_basis(&sft, rsp);
			rsp[rsp_length++] = (nb / 8) + ((nb % 8) ? 1 : 0);
			rsp_length = response_io_status(address, nb,
					mb_mapping->tab_input_bits, rsp, rsp_length);
		}
	}
		break;
	case _FC_READ_HOLDING_REGISTERS: {
		int nb = (req[offset + 3] << 8) + req[offset + 4];

		if ((address + nb) > mb_mapping->nb_registers) {
			if (ctx->debug) {
				zlog_trace("Illegal data address %0x in read_registers",
						address + nb);
			}
			rsp_length = response_exception(ctx, &sft,
					MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
		} else {
			int i;

			rsp_length = ctx->backend->build_response_basis(&sft, rsp);
			rsp[rsp_length++] = nb << 1;
			for (i = address; i < address + nb; i++) {
				rsp[rsp_length++] = mb_mapping->tab_registers[i] >> 8;
				rsp[rsp_length++] = mb_mapping->tab_registers[i] & 0xFF;
			}
		}
	}
		break;
	case _FC_READ_INPUT_REGISTERS: {
		/* Similar to holding registers (but too many arguments to use a
		 * function) */
		int nb = (req[offset + 3] << 8) + req[offset + 4];

		if ((address + nb) > mb_mapping->nb_input_registers) {
			if (ctx->debug) {
				zlog_trace("Illegal data address %0x in read_input_registers",
						address + nb);
			}
			rsp_length = response_exception(ctx, &sft,
					MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
		} else {
			int i;

			rsp_length = ctx->backend->build_response_basis(&sft, rsp);
			rsp[rsp_length++] = nb << 1;
			for (i = address; i < address + nb; i++) {
				rsp[rsp_length++] = mb_mapping->tab_input_registers[i] >> 8;
				rsp[rsp_length++] = mb_mapping->tab_input_registers[i] & 0xFF;
			}
		}
	}
		break;
	case _FC_WRITE_SINGLE_COIL:
		if (address >= mb_mapping->nb_bits) {
			if (ctx->debug) {
				zlog_trace("Illegal data address %0x in write_bit", address);
			}
			rsp_length = response_exception(ctx, &sft,
					MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
		} else {
			int data = (req[offset + 3] << 8) + req[offset + 4];

			if (data == 0xFF00 || data == 0x0) {
				mb_mapping->tab_bits[address] = (data) ? ON : OFF;
				if (_MODBUS_ASCII == ctx->channel_type) {
					memcpy(rsp, req + 2, req_length);
					rsp[0] = slave;
				} else {
					memcpy(rsp, req, req_length);
				}

				rsp_length = req_length;
			} else {
				if (ctx->debug) {
					zlog_trace(
							"Illegal data value %0x in write_bit request at address %0x",
							data, address);
				}
				rsp_length = response_exception(ctx, &sft,
						MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE, rsp);
			}
		}
		break;
	case _FC_WRITE_SINGLE_REGISTER:
		if (address >= mb_mapping->nb_registers) {
			if (ctx->debug) {
				zlog_trace("Illegal data address %0x in write_register",
						address);
			}
			rsp_length = response_exception(ctx, &sft,
					MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
		} else {
			int data = (req[offset + 3] << 8) + req[offset + 4];

			mb_mapping->tab_registers[address] = data;
			if (_MODBUS_ASCII == ctx->channel_type) {
				memcpy(rsp, req + 2, req_length);
				rsp[0] = slave;
			} else {
				memcpy(rsp, req, req_length);
			}
			rsp_length = req_length;
		}
		break;
	case _FC_WRITE_MULTIPLE_COILS: {
		int nb = (req[offset + 3] << 8) + req[offset + 4];

		if ((address + nb) > mb_mapping->nb_bits) {
			if (ctx->debug) {
				zlog_trace("Illegal data address %0x in write_bits",
						address + nb);
			}
			rsp_length = response_exception(ctx, &sft,
					MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
		} else {
			/* 6 = byte count */
			modbus_set_bits_from_bytes(mb_mapping->tab_bits, address, nb,
					&req[offset + 6]);

			rsp_length = ctx->backend->build_response_basis(&sft, rsp);
			/* 4 to copy the bit address (2) and the quantity of bits */
			if (_MODBUS_ASCII == ctx->channel_type)
				memcpy(rsp + rsp_length, req + rsp_length + 2, 4);
			else
				memcpy(rsp + rsp_length, req + rsp_length, 4);

			rsp_length += 4;
		}
	}
		break;
	case _FC_WRITE_MULTIPLE_REGISTERS: {
		int nb = (req[offset + 3] << 8) + req[offset + 4];

		if ((address + nb) > mb_mapping->nb_registers) {
			if (ctx->debug) {
				zlog_trace("Illegal data address %0x in write_registers",
						address + nb);
			}
			rsp_length = response_exception(ctx, &sft,
					MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
		} else {
			int i, j;
			for (i = address, j = 6; i < address + nb; i++, j += 2) {
				/* 6 and 7 = first value */
				mb_mapping->tab_registers[i] = (req[offset + j] << 8)
						+ req[offset + j + 1];
			}

			rsp_length = ctx->backend->build_response_basis(&sft, rsp);
			/* 4 to copy the address (2) and the no. of registers */
			if (_MODBUS_ASCII == ctx->channel_type)
				memcpy(rsp + rsp_length, req + rsp_length + 2, 4);
			else
				memcpy(rsp + rsp_length, req + rsp_length, 4);
			rsp_length += 4;
		}
	}
		break;
	case _FC_REPORT_SLAVE_ID: {
		int str_len;
		int byte_count_pos;

		rsp_length = ctx->backend->build_response_basis(&sft, rsp);
		/* Skip byte count for now */
		byte_count_pos = rsp_length++;
		rsp[rsp_length++] = _REPORT_SLAVE_ID;
		/* Run indicator status to ON */
		rsp[rsp_length++] = 0xFF;
		/* LMB + length of LIBMODBUS_VERSION_STRING */
		str_len = 3 + strlen(LIBMODBUS_VERSION_STRING);
		memcpy(rsp + rsp_length, "LMB" LIBMODBUS_VERSION_STRING, str_len);
		rsp_length += str_len;
		rsp[byte_count_pos] = rsp_length - byte_count_pos - 1;
	}
		break;
	case _FC_READ_EXCEPTION_STATUS:
		if (ctx->debug) {
			zlog_trace("FIXME Not implemented");
		}
		errno = ENOPROTOOPT;
		return -1;
		break;

	case _FC_WRITE_AND_READ_REGISTERS: {
		int nb = (req[offset + 3] << 8) + req[offset + 4];
		uint16_t address_write = (req[offset + 5] << 8) + req[offset + 6];
		int nb_write = (req[offset + 7] << 8) + req[offset + 8];

		if ((address + nb) > mb_mapping->nb_registers
				|| (address_write + nb_write) > mb_mapping->nb_registers) {
			if (ctx->debug) {
				zlog_trace(
						"Illegal data read address %0x or write address %0x write_and_read_registers",
						address + nb, address_write + nb_write);
			}
			rsp_length = response_exception(ctx, &sft,
					MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS, rsp);
		} else {
			int i, j;
			rsp_length = ctx->backend->build_response_basis(&sft, rsp);
			rsp[rsp_length++] = nb << 1;

			/* Write first.
			 10 and 11 are the offset of the first values to write */
			for (i = address_write, j = 10; i < address_write + nb_write;
					i++, j += 2) {
				mb_mapping->tab_registers[i] = (req[offset + j] << 8)
						+ req[offset + j + 1];
			}

			/* and read the data for the response */
			for (i = address; i < address + nb; i++) {
				rsp[rsp_length++] = mb_mapping->tab_registers[i] >> 8;
				rsp[rsp_length++] = mb_mapping->tab_registers[i] & 0xFF;
			}
		}
	}
		break;

	default:
		rsp_length = response_exception(ctx, &sft,
				MODBUS_EXCEPTION_ILLEGAL_FUNCTION, rsp);
		break;
	}

	return send_msg(ctx, rsp, rsp_length);
}

int modbus_reply_exception(modbus_t *ctx, const uint8_t *req,
		unsigned int exception_code) {
	int offset = ctx->backend->header_length;
	int slave = req[offset - 1];
	int function = req[offset];
	uint8_t rsp[MAX_MESSAGE_LENGTH];
	int rsp_length;
	int dummy_length = 99;
	sft_t sft;

	if (ctx->backend->filter_request(ctx, slave) == 1) {
		/* Filtered */
		return 0;
	}

	sft.slave = slave;
	sft.function = function + 0x80;
	;
	sft.t_id = ctx->backend->prepare_response_tid(req, &dummy_length);
	rsp_length = ctx->backend->build_response_basis(&sft, rsp);

	/* Positive exception code */
	if (exception_code < MODBUS_EXCEPTION_MAX) {
		rsp[rsp_length++] = exception_code;
		return send_msg(ctx, rsp, rsp_length);
	} else {
		errno = EINVAL;
		return -1;
	}
}
		
/* Reads IO status */
static int read_io_status(modbus_t *ctx, int function,
						  int addr, int nb, uint8_t *dest)
{
	int rc;
	int req_length;

	uint8_t req[_MIN_REQ_LENGTH];
	uint8_t rsp[MAX_MESSAGE_LENGTH];

	// case 0x2B: addr=code nb=oi;
	req_length = ctx->backend->build_request_basis(ctx, function, addr, nb, req);

	rc = send_msg(ctx, req, req_length);
	if (rc > 0) {
		int i, temp, bit;
		int pos = 0;
		int offset;
		int offset_end;

#ifdef USE_FUNCTION_0X2B
		rsp[0] = function;
#endif
		rc = receive_msg(ctx, rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

#ifdef USE_FUNCTION_0X2B
		offset = _MODBUS_RTU_RESPONSE_0X2B_HEADER - 2;			
		rc = rc - _MODBUS_RTU_RESPONSE_0X2B_HEADER - 2;									// -帧头，-校验帧
		for (i = 0; i < rc; i++) {
			dest[i] = rsp[offset + 2 + i];
		}

		if (ctx->debug) {
			printf("\n data frame [%d]:    \n",rc);    
			for (i=0; i < rc ; i++)
				printf("<%.2X>", dest[i]);			// 打印接收的数据、
		}

		return rc;
#endif
		rc = check_confirmation(ctx, req, req_length, rsp, rc);
		if (rc == -1)
			return -1;

		offset = ctx->backend->header_length + 2;
		offset_end = offset + rc;
		for (i = offset; i < offset_end; i++) {
			/* Shift reg hi_byte to temp */
			temp = rsp[i];

			for (bit = 0x01; (bit & 0xff) && (pos < nb);) {
				dest[pos++] = (temp & bit) ? TRUE : FALSE;
				bit = bit << 1;
			}

		}
	}

	return rc;
}

/* Reads the boolean status of bits and sets the array elements
 in the destination to TRUE or FALSE (single bits). */
int modbus_read_bits(modbus_t *ctx, int addr, int nb, uint8_t *dest) {
	int rc;

	if (nb > MODBUS_MAX_READ_BITS) {
		if (ctx->debug) {
			zlog_debug("Too many bits requested (%d > %d)", nb,
					MODBUS_MAX_READ_BITS);
		}
		errno = EMBMDATA;
		return -1;
	}

	rc = read_io_status(ctx, _FC_READ_COILS, addr, nb, dest);

	if (rc == -1)
		return -1;
	else
		return nb;
}

/* Same as modbus_read_bits but reads the remote device input table */
int modbus_read_input_bits(modbus_t *ctx, int addr, int nb, uint8_t *dest) {
	int rc;

	if (nb > MODBUS_MAX_READ_BITS) {
		if (ctx->debug) {
			zlog_debug("Too many discrete inputs requested (%d > %d)", nb,
					MODBUS_MAX_READ_BITS);
		}
		errno = EMBMDATA;
		return -1;
	}

	rc = read_io_status(ctx, _FC_READ_DISCRETE_INPUTS, addr, nb, dest);

	if (rc == -1)
		return -1;
	else
		return nb;
}

/* Reads the data from a remove device and put that data into an array */
static int read_registers(modbus_t *ctx, int function, int addr, int nb,
		uint16_t *dest) {
	int rc;
	int req_length;
	uint8_t req[_MIN_REQ_LENGTH];
	uint8_t rsp[MAX_MESSAGE_LENGTH];

	if (nb > MODBUS_MAX_READ_REGISTERS) {
		if (ctx->debug) {
			zlog_debug("Too many registers requested (%d > %d)", nb,
					MODBUS_MAX_READ_REGISTERS);
		}
		errno = EMBMDATA;
		return -1;
	}

	req_length = ctx->backend->build_request_basis(ctx, function, addr, nb,
			req);
	tcflush(ctx->s, TCIOFLUSH);  // TCIFLUSH 刷清输入队列
							 // TCOFLUSH 刷清输出队列
							 // TCIOFLUSH 刷清输入、输出队列

	rc = send_msg(ctx, req, req_length);
	if (rc > 0) {
		int offset;
		int i;

		req_length = rc;
		rc = receive_msg(ctx, rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(ctx, req, req_length, rsp, rc);
		if (rc == -1)
			return -1;

		offset = ctx->backend->header_length;

		for (i = 0; i < rc; i++) {
			/* shift reg hi_byte to temp OR with lo_byte */
			dest[i] = (rsp[offset + 2 + (i << 1)] << 8)
					| rsp[offset + 3 + (i << 1)];
		}
	}

	return rc;
}

/* Reads the holding registers of remote device and put the data into an
 array */
int modbus_read_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest) {
	int status;

	if (nb > MODBUS_MAX_READ_REGISTERS) {
		if (ctx->debug) {
			zlog_debug("Too many registers requested (%d > %d)", nb,
					MODBUS_MAX_READ_REGISTERS);
		}
		errno = EMBMDATA;
		return -1;
	}

	status = read_registers(ctx, _FC_READ_HOLDING_REGISTERS, addr, nb, dest);
	return status;
}

/* Reads the input registers of remote device and put the data into an array */
int modbus_read_input_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest) {
	int status;

	if (nb > MODBUS_MAX_READ_REGISTERS) {
		zlog_debug("Too many input registers requested (%d > %d)", nb,
				MODBUS_MAX_READ_REGISTERS);
		errno = EMBMDATA;
		return -1;
	}

	status = read_registers(ctx, _FC_READ_INPUT_REGISTERS, addr, nb, dest);

	return status;
}

/* Write a value to the specified register of the remote device.
 Used by write_bit and write_register */
static int write_single(modbus_t *ctx, int function, int addr, int value) {
	int rc;
	int req_length;
	uint8_t req[_MIN_REQ_LENGTH];

	req_length = ctx->backend->build_request_basis(ctx, function, addr, value,
			req);

	rc = send_msg(ctx, req, req_length);
	if (rc > 0) {
		/* Used by write_bit and write_register */
		uint8_t rsp[_MIN_REQ_LENGTH];

		req_length = rc;
		rc = receive_msg(ctx, rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(ctx, req, req_length, rsp, rc);
	}

	return rc;
}

/* Turns ON or OFF a single bit of the remote device */
int modbus_write_bit(modbus_t *ctx, int addr, int status) {
	return write_single(ctx, _FC_WRITE_SINGLE_COIL, addr, status ? 0xFF00 : 0);
}

/* Writes a value in one register of the remote device */
int modbus_write_register(modbus_t *ctx, int addr, int value) {
	return write_single(ctx, _FC_WRITE_SINGLE_REGISTER, addr, value);
}

/* Write the bits of the array in the remote device */
int modbus_write_bits(modbus_t *ctx, int addr, int nb, const uint8_t *src) {
	int rc;
	int i;
	int byte_count;
	int req_length;
	int bit_check = 0;
	int pos = 0;

	uint8_t req[MAX_MESSAGE_LENGTH];

	if (nb > MODBUS_MAX_WRITE_BITS) {
		if (ctx->debug) {
			zlog_debug("Writing too many bits (%d > %d)", nb,
					MODBUS_MAX_WRITE_BITS);
		}
		errno = EMBMDATA;
		return -1;
	}

	req_length = ctx->backend->build_request_basis(ctx,
	_FC_WRITE_MULTIPLE_COILS, addr, nb, req);
	byte_count = (nb / 8) + ((nb % 8) ? 1 : 0);
	req[req_length++] = byte_count;

	for (i = 0; i < byte_count; i++) {
		int bit;

		bit = 0x01;
		req[req_length] = 0;

		while ((bit & 0xFF) && (bit_check++ < nb)) {
			if (src[pos++])
				req[req_length] |= bit;
			else
				req[req_length] &= ~bit;

			bit = bit << 1;
		}
		req_length++;
	}

	rc = send_msg(ctx, req, req_length);
	if (rc > 0) {
		uint8_t rsp[MAX_MESSAGE_LENGTH];

		req_length = rc;
		rc = receive_msg(ctx, rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(ctx, req, req_length, rsp, rc);
	}

	return rc;
}

/* Write the values from the array to the registers of the remote device */
int modbus_write_registers(modbus_t *ctx, int addr, int nb, const uint16_t *src) {
	int rc;
	int i;
	int req_length;
	int byte_count;

	uint8_t req[MAX_MESSAGE_LENGTH];

	if (nb > MODBUS_MAX_WRITE_REGISTERS) {
		if (ctx->debug) {
			zlog_debug("Trying to write to too many registers (%d > %d)", nb,
					MODBUS_MAX_WRITE_REGISTERS);
		}
		errno = EMBMDATA;
		return -1;
	}

	req_length = ctx->backend->build_request_basis(ctx,
	_FC_WRITE_MULTIPLE_REGISTERS, addr, nb, req);
	byte_count = nb * 2;
	req[req_length++] = byte_count;

	for (i = 0; i < nb; i++) {
		req[req_length++] = src[i] >> 8;
		req[req_length++] = src[i] & 0x00FF;
	}

	rc = send_msg(ctx, req, req_length);
	if (rc > 0) {
		uint8_t rsp[MAX_MESSAGE_LENGTH];

		req_length = rc;
		rc = receive_msg(ctx, rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(ctx, req, req_length, rsp, rc);
	}

	return rc;
}

/* Write multiple registers from src array to remote device and read multiple
 registers from remote device to dest array. */
int modbus_write_and_read_registers(modbus_t *ctx, int write_addr, int write_nb,
		const uint16_t *src, int read_addr, int read_nb, uint16_t *dest)

{
	int rc;
	int req_length;
	int i;
	int byte_count;
	uint8_t req[MAX_MESSAGE_LENGTH];
	uint8_t rsp[MAX_MESSAGE_LENGTH];

	if (write_nb > MODBUS_MAX_RW_WRITE_REGISTERS) {
		if (ctx->debug) {
			zlog_debug("Too many registers to write (%d > %d)", write_nb,
					MODBUS_MAX_RW_WRITE_REGISTERS);
		}
		errno = EMBMDATA;
		return -1;
	}

	if (read_nb > MODBUS_MAX_READ_REGISTERS) {
		if (ctx->debug) {
			zlog_debug("Too many registers requested (%d > %d)", read_nb,
					MODBUS_MAX_READ_REGISTERS);
		}
		errno = EMBMDATA;
		return -1;
	}
	req_length = ctx->backend->build_request_basis(ctx,
	_FC_WRITE_AND_READ_REGISTERS, read_addr, read_nb, req);

	req[req_length++] = write_addr >> 8;
	req[req_length++] = write_addr & 0x00ff;
	req[req_length++] = write_nb >> 8;
	req[req_length++] = write_nb & 0x00ff;
	byte_count = write_nb * 2;
	req[req_length++] = byte_count;

	for (i = 0; i < write_nb; i++) {
		req[req_length++] = src[i] >> 8;
		req[req_length++] = src[i] & 0x00FF;
	}

	rc = send_msg(ctx, req, req_length);
	if (rc > 0) {
		int offset;

		req_length = rc;
		rc = receive_msg(ctx, rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(ctx, req, req_length, rsp, rc);
		if (rc == -1)
			return -1;

		offset = ctx->backend->header_length;

		/* If rc is negative, the loop is jumped ! */
		for (i = 0; i < rc; i++) {
			/* shift reg hi_byte to temp OR with lo_byte */
			dest[i] = (rsp[offset + 2 + (i << 1)] << 8)
					| rsp[offset + 3 + (i << 1)];
		}
	}

	return rc;
}

/* Send a request to get the slave ID of the device (only available in serial
 communication). */
int modbus_report_slave_id(modbus_t *ctx, uint8_t *dest) {
	int rc;
	int req_length;
	uint8_t req[_MIN_REQ_LENGTH];

	req_length = ctx->backend->build_request_basis(ctx, _FC_REPORT_SLAVE_ID, 0,
			0, req);

	/* HACKISH, addr and count are not used */
	req_length -= 4;

	rc = send_msg(ctx, req, req_length);
	if (rc > 0) {
		int i;
		int offset;
		uint8_t rsp[MAX_MESSAGE_LENGTH];

		req_length = rc;
		rc = receive_msg(ctx, rsp, MSG_CONFIRMATION);
		if (rc == -1)
			return -1;

		rc = check_confirmation(ctx, req, req_length, rsp, rc);
		if (rc == -1)
			return -1;

		offset = ctx->backend->header_length + 2;

		/* Byte count, slave id, run indicator status,
		 additional data */
		for (i = 0; i < rc; i++) {
			dest[i] = rsp[offset + i];
		}
	}

	return rc;
}

void _modbus_init_common(modbus_t *ctx, modbus_channel_type_t type) {
	/* Slave and socket are initialized to -1 */
	ctx->slave = -1;
	ctx->s = -1;

	ctx->debug = FALSE;
	ctx->error_recovery = MODBUS_ERROR_RECOVERY_NONE;

	ctx->response_timeout.tv_sec = 0;
	ctx->response_timeout.tv_usec = _RESPONSE_TIMEOUT;

	ctx->byte_timeout.tv_sec = 0;
	ctx->byte_timeout.tv_usec = _BYTE_TIMEOUT;

	ctx->channel_type = type;
}

/* Define the slave number */
int modbus_set_slave(modbus_t *ctx, int slave) {
	return ctx->backend->set_slave(ctx, slave);
}

int modbus_set_error_recovery(modbus_t *ctx,
		modbus_error_recovery_mode error_recovery) {
	if (error_recovery >= 0) {
		ctx->error_recovery = (uint8_t) error_recovery;
	} else {
		errno = EINVAL;
		return -1;
	}

	return 0;
}

void modbus_set_socket(modbus_t *ctx, int socket) {
	ctx->s = socket;
}

int modbus_get_socket(modbus_t *ctx) {
	return ctx->s;
}

/* Get the timeout interval used to wait for a response */
void modbus_get_response_timeout(modbus_t *ctx, struct timeval *timeout) {
	*timeout = ctx->response_timeout;
}

void modbus_set_response_timeout(modbus_t *ctx, const struct timeval *timeout) {
	ctx->response_timeout = *timeout;
}

/* Get the timeout interval between two consecutive bytes of a message */
void modbus_get_byte_timeout(modbus_t *ctx, struct timeval *timeout) {
	*timeout = ctx->byte_timeout;
}

void modbus_set_byte_timeout(modbus_t *ctx, const struct timeval *timeout) {
	ctx->byte_timeout = *timeout;
}

int modbus_get_header_length(modbus_t *ctx) {
	return ctx->backend->header_length;
}

int modbus_connect(modbus_t *ctx) {
	return ctx->backend->connect(ctx);
}

void modbus_close(modbus_t *ctx) {
	if (ctx == NULL)
		return;

	ctx->backend->close(ctx);
}

void modbus_free(modbus_t *ctx) {
	if (ctx == NULL)
		return;

	free(ctx->backend_data);
	free(ctx);
}

void modbus_set_debug(modbus_t *ctx, int boolean) {
	ctx->debug = boolean;
}

/* Allocates 4 arrays to store bits, input bits, registers and inputs
 registers. The pointers are stored in modbus_mapping structure.

 The modbus_mapping_new() function shall return the new allocated structure if
 successful. Otherwise it shall return NULL and set errno to ENOMEM. */
modbus_mapping_t* modbus_mapping_new(int nb_bits, int nb_input_bits,
		int nb_registers, int nb_input_registers) {
	modbus_mapping_t *mb_mapping;

	mb_mapping = (modbus_mapping_t *) malloc(sizeof(modbus_mapping_t));
	if (mb_mapping == NULL) {
		return NULL;
	}

	/* 0X */
	mb_mapping->nb_bits = nb_bits;
	if (nb_bits == 0) {
		mb_mapping->tab_bits = NULL;
	} else {
		/* Negative number raises a POSIX error */
		mb_mapping->tab_bits = (uint8_t *) malloc(nb_bits * sizeof(uint8_t));
		if (mb_mapping->tab_bits == NULL) {
			free(mb_mapping);
			return NULL;
		}
		memset(mb_mapping->tab_bits, 0, nb_bits * sizeof(uint8_t));
	}

	/* 1X */
	mb_mapping->nb_input_bits = nb_input_bits;
	if (nb_input_bits == 0) {
		mb_mapping->tab_input_bits = NULL;
	} else {
		mb_mapping->tab_input_bits = (uint8_t *) malloc(
				nb_input_bits * sizeof(uint8_t));
		if (mb_mapping->tab_input_bits == NULL) {
			free(mb_mapping->tab_bits);
			free(mb_mapping);
			return NULL;
		}
		memset(mb_mapping->tab_input_bits, 0, nb_input_bits * sizeof(uint8_t));
	}

	/* 4X */
	mb_mapping->nb_registers = nb_registers;
	if (nb_registers == 0) {
		mb_mapping->tab_registers = NULL;
	} else {
		mb_mapping->tab_registers = (uint16_t *) malloc(
				nb_registers * sizeof(uint16_t));
		if (mb_mapping->tab_registers == NULL) {
			free(mb_mapping->tab_input_bits);
			free(mb_mapping->tab_bits);
			free(mb_mapping);
			return NULL;
		}
		memset(mb_mapping->tab_registers, 0, nb_registers * sizeof(uint16_t));
	}

	/* 3X */
	mb_mapping->nb_input_registers = nb_input_registers;
	if (nb_input_registers == 0) {
		mb_mapping->tab_input_registers = NULL;
	} else {
		mb_mapping->tab_input_registers = (uint16_t *) malloc(
				nb_input_registers * sizeof(uint16_t));
		if (mb_mapping->tab_input_registers == NULL) {
			free(mb_mapping->tab_registers);
			free(mb_mapping->tab_input_bits);
			free(mb_mapping->tab_bits);
			free(mb_mapping);
			return NULL;
		}
		memset(mb_mapping->tab_input_registers, 0,
				nb_input_registers * sizeof(uint16_t));
	}

	return mb_mapping;
}

/* Frees the 4 arrays */
void modbus_mapping_free(modbus_mapping_t *mb_mapping) {
	if (mb_mapping == NULL) {
		return;
	}

	free(mb_mapping->tab_input_registers);
	free(mb_mapping->tab_registers);
	free(mb_mapping->tab_input_bits);
	free(mb_mapping->tab_bits);
	free(mb_mapping);
}

#ifndef HAVE_STRLCPY
/*
 * Function strlcpy was originally developed by
 * Todd C. Miller <Todd.Miller@courtesan.com> to simplify writing secure code.
 * See ftp://ftp.openbsd.org/pub/OpenBSD/src/lib/libc/string/strlcpy.3
 * for more information.
 *
 * Thank you Ulrich Drepper... not!
 *
 * Copy src to string dest of size dest_size.  At most dest_size-1 characters
 * will be copied.  Always NUL terminates (unless dest_size == 0).  Returns
 * strlen(src); if retval >= dest_size, truncation occurred.
 */
size_t strlcpy(char *dest, const char *src, size_t dest_size) {
	register char *d = dest;
	register const char *s = src;
	register size_t n = dest_size;

	/* Copy as many bytes as will fit */
	if (n != 0 && --n != 0) {
		do {
			if ((*d++ = *s++) == 0)
				break;
		} while (--n != 0);
	}

	/* Not enough room in dest, add NUL and traverse rest of src */
	if (n == 0) {
		if (dest_size != 0)
			*d = '\0'; /* NUL-terminate dest */
		while (*s++)
			;
	}

	return (s - src - 1); /* count does not include NUL */
}
#endif



/*****************************
 *
 * modbus 0x2B功能码
 *
 ****************************/
static int receive_msg_deviceInfo(modbus_t *ctx, uint8_t *msg, msg_type_t msg_type) {
	int rc;
	int i;
	fd_set rfds;
	struct timeval tv;
	struct timeval *p_tv;
	int length_to_read;						// 需要 recv接收的数据长度
	uint8_t num_of_read = 0;				// 已经 recv接收的数据次数
	uint8_t num_of_objects;
	int msg_length = 0;
	_step_t step;

	if (ctx->debug) {
		if (msg_type == MSG_INDICATION) {
			zlog_trace("Waiting for a indication...");
		} else {
			zlog_trace("Waiting for a confirmation...");
		}
	}

	/* Add a file descriptor to the set */
	FD_ZERO(&rfds);
	FD_SET(ctx->s, &rfds);

	/* We need to analyse the message step by step.  At the first step, we want
	 * to reach the function code because all packets contain this
	 * information. */
	step = _STEP_FUNCTION;

	length_to_read = ctx->backend->header_length + 1;

	if (msg_type == MSG_INDICATION) {
		/* Wait for a message, we don't know when the message will be
		 * received */
		p_tv = NULL;
	} else {
		tv.tv_sec = ctx->response_timeout.tv_sec;
		tv.tv_usec = ctx->response_timeout.tv_usec;
		p_tv = &tv;
	}

	// 分步 recv数据；
	while (length_to_read != 0) {
		rc = ctx->backend->select(ctx, &rfds, p_tv, length_to_read);
		if (rc == -1) {
			zlog_debug("%s: select", modbus_strerror(errno));
			if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK) {
				int saved_errno = errno;

				if (errno == ETIMEDOUT) {
					_sleep_and_flush(ctx);
				} else if (errno == EBADF) {
					modbus_close(ctx);
					modbus_connect(ctx);
				}
				errno = saved_errno;
			}
			return -1;
		}

		rc = ctx->backend->recv(ctx, msg + msg_length, length_to_read);
        if (ctx->debug) {
            printf(" recv one frame data[%d]: \n",rc);    
            for (i=0; i < rc; i++)
                printf("<%.2X>", (msg + msg_length)[i]);          // 打印接收的数据
			printf("\n");
        }
		
		if (rc == 0) {
			errno = ECONNRESET;
			rc = -1;
		}

		if (rc == -1) {
			if (ctx->debug) {
				zlog_debug("%s: read", modbus_strerror(errno));
			}
			if ((ctx->error_recovery & MODBUS_ERROR_RECOVERY_LINK)
				&& (errno == ECONNRESET || errno == ECONNREFUSED ||
					errno == EBADF)) {
				int saved_errno = errno;
				modbus_close(ctx);
				modbus_connect(ctx);
				/* Could be removed by previous calls */
				errno = saved_errno;
			}
			return -1;
		}

		/* Sums bytes received */
		msg_length += rc;
		/* Computes remaining bytes */
		length_to_read -= rc;

		if (length_to_read == 0) {
			switch (step) {
			case _STEP_FUNCTION: 
				/* 帧头的剩余长度 */ 
				length_to_read = _MODBUS_RTU_RESPONSE_0X2B_HEADER - 2;
				step = _STEP_META;
				break;
			case _STEP_META:
				/* 记录 Number of objects 的值，后续根据该值确认需要接收Object value的次数 */
				num_of_objects = *(msg + msg_length - 1);
				// zlog_info("Number of objects = %d ", num_of_objects);
				length_to_read = 2;			// Object id 和 Object length
				step = _STEP_INFO;
				break;
			case _STEP_INFO:
				length_to_read = *(msg + msg_length - 1);
				step = _STEP_DATA;
				break;
			case _STEP_DATA:			
				num_of_read++;
				length_to_read = 2;
				if (num_of_read == num_of_objects)
					step = _STEP_CRC;
				else
					step = _STEP_INFO;		// 递归去读 Object value 直到读取次数等于 Number of objects
				break;
			case _STEP_CRC: 
				length_to_read = 0;
				step = _STEP_END;
				break;
			default:
				break;
			}
		}


		if (length_to_read > 0 && ctx->byte_timeout.tv_sec != -1) {
			/* 如果缓冲区中没有字符，则两个连续字节之间允许的超时时间由byte_timeout定义       */
			tv.tv_sec = ctx->byte_timeout.tv_sec;
			tv.tv_usec = ctx->byte_timeout.tv_usec;
			p_tv = &tv;
		}
	}

	/* Display the hex code of each character received */
	if (ctx->debug) {
		// hflog_trace(msg, msg_length, ctx->s);
		;
	}

	msg_length = ctx->backend->recv_msg_post(msg, msg_length);
	if (msg_length < 0)
		return -1;

	return ctx->backend->check_integrity(ctx, msg, msg_length);
}


check_frame_0x2B(modbus_t * ctx, uint8_t * req, int req_length, uint8_t * rsp, int rsp_length)
{

	/* 校验slve add、function、mei type、code 是否相同 */
	if (strncmp(req, rsp, 4) != 0){
		zlog_info("check error for modbus 0x2B");
		errno = EMBBADDATA;
		return -1;
	}
	
	return rsp_length;
}


/* Reads IO status */
static int _modbus_read_deviceInfo(modbus_t *ctx, int function,
						  int code, int oi, uint8_t *dest)
{
	int rc;
	int req_length;

	uint8_t req[_MIN_REQ_LENGTH];
	uint8_t rsp[MAX_MESSAGE_LENGTH];

	// case 0x2B: addr=code nb=oi;
	req_length = ctx->backend->build_request_basis(ctx, function, code, oi, req);

	rc = send_msg(ctx, req, req_length);
	if (rc > 0) {
		int i;
		int offset;

		rsp[0] = function;
		rc = receive_msg_deviceInfo(ctx, rsp, MSG_CONFIRMATION);					// BUG：  串口接收有问题， 不能保证一次接收完整的一包数据。需要根据接收数据进行分步验证，多次调recv接收
		if (rc == -1)
			return -1;

		/* 校验请求与响应报文是否匹配 */
		rc = check_frame_0x2B(ctx, req, req_length, rsp, rc);
		if (rc == -1)
			return -1;

		offset = _MODBUS_RTU_RESPONSE_0X2B_HEADER - 2;			
		rc = rc - _MODBUS_RTU_RESPONSE_0X2B_HEADER - 2;									// -帧头，-校验帧
		for (i = 0; i < rc; i++) {
			dest[i] = rsp[offset + 2 + i];
		}

		if (ctx->debug) {
			printf("\n data frame [%d]:    \n",rc);    
			for (i=0; i < rc ; i++)
				printf("<%.2X>", dest[i]);			// 打印接收的数据
		}
	}
	return rc;
}



/* 
 * 读取设备标识。功能码： 0x2B
 * code：read dec id code（读取方式，协议标准01/02/03/04）
 * oi：  objec id（要读取的对象标识）
 * tips： 目前未兼容 tcp 模式
 */
int modbus_read_deviceInfo(modbus_t *ctx, uint8_t code, uint8_t oi, uint8_t *dest)
{
	int status = 0;
	
    zlog_info("\n\n bigin once modbus DI: \n");
    status = _modbus_read_deviceInfo(ctx, _FC_READ_DEVICE_INFO, code, oi, dest);

    // zlog_info(" Not define USE_FUNCTION_0X2B \n");

	return status;
}




