/*
 * Made in China
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif
#include <assert.h>

#include "zlog.h"

#include "modbus-private.h"
#include "modbus-asc.h"
#include "modbus-asc-private.h"

#if HAVE_DECL_TIOCSRS485
#include <sys/ioctl.h>
#include <linux/serial.h>
#endif

static inline uint8_t asc2hex(uint8_t asccode) {
	uint8_t ret = 0;

	if ('0' <= asccode && asccode <= '9')
		ret = asccode - '0';
	else if ('a' <= asccode && asccode <= 'f')
		ret = asccode - 'a' + 10;
	else if ('A' <= asccode && asccode <= 'F')
		ret = asccode - 'A' + 10;

	return ret;
}

_Bool _modbus_asc2hex(const uint8_t *from, uint8_t *to) {
	if (!from || !to)
		return false;

	*to = asc2hex(from[0]) << 4;
	*to += asc2hex(from[1]);

	return true;
}

_Bool _modbus_hex2asc(uint8_t *to, uint8_t from) {
	uint8_t l4, h4;
	if (!to)
		return false;

	h4 = from >> 4;
	l4 = from & 0x0f;

	if (h4 <= 9)
		to[0] = h4 + '0';
	else
		to[0] = h4 - 10 + 'A';

	if (l4 <= 9)
		to[1] = l4 + '0';
	else
		to[1] = l4 - 10 + 'A';

	return true;
}

/* Define the slave ID of the remote device to talk in master mode or set the
 * internal slave ID in slave mode */
static int _modbus_set_slave(modbus_t *ctx, int slave) {
	/* Broadcast address is 0 (MODBUS_BROADCAST_ADDRESS) */
	if (slave >= 0 && slave <= 247) {
		ctx->slave = slave;
	} else {
		errno = EINVAL;
		return -1;
	}

	return 0;
}

/* Builds a ASC request header */
static int _modbus_asc_build_request_basis(modbus_t *ctx, int function, int addr, int nb, uint8_t *req) {
	assert(ctx->slave != -1);
	req[0] = ctx->slave;
	req[1] = function;
	req[2] = addr >> 8;
	req[3] = addr & 0x00ff;
	req[4] = nb >> 8;
	req[5] = nb & 0x00ff;

	return _MODBUS_ASC_PRESET_REQ_LENGTH;
}

/* Builds a ASC response header */
static int _modbus_asc_build_response_basis(sft_t *sft, uint8_t *rsp) {
	/* In this case, the slave is certainly valid because a check is already
	 * done in _modbus_asc_listen */
	rsp[0] = sft->slave;
	rsp[1] = sft->function;

	return _MODBUS_ASC_PRESET_RSP_LENGTH;
}

static uint8_t crc8(uint8_t *buffer, uint16_t buffer_length) {
	uint8_t crc = 0;
	int i;

	for (i = 0; i < buffer_length; i++)
		crc += buffer[i];

	return -crc;
}

static uint8_t crc8_asc_slave(uint8_t *buffer, uint16_t buffer_length) {
	uint8_t crc = 0;
	int i;

	_modbus_asc2hex(buffer, &crc); // asc slave to hex
	for (i = 2; i < buffer_length; i++)
		crc += buffer[i];

	return -crc;
}

int _modbus_asc_prepare_response_tid(const uint8_t *req, int *req_length) {
	(*req_length) -= _MODBUS_ASC_CHECKSUM_LENGTH + 2;
	/* No TID */
	return 0;
}

int _modbus_asc_send_msg_pre(uint8_t *req, int req_length) {
	int i;
	uint8_t crc;
	uint8_t _req[req_length * 2 + 5]; // 2 * hex(1byte) + CRC(2byte) + start(1byte) + CR(1byte) + LF(1byte)

	hlog_trace(req, req_length);
	crc = crc8(req, req_length);
	_req[0] = _MODBUS_ASC_START;
	for (i = 0; i < req_length; i++)
		_modbus_hex2asc(&_req[i * 2 + 1], req[i]);

	_modbus_hex2asc(&_req[req_length * 2 + 1], crc);
	_req[req_length * 2 + 3] = _MODBUS_ASC_CR;
	_req[req_length * 2 + 4] = _MODBUS_ASC_LF;

	memmove(req, _req, req_length * 2 + 5);

	return req_length * 2 + 5;
}

#if defined(_WIN32)

/* This simple implementation is sort of a substitute of the select() call,
 * working this way: the win32_ser_select() call tries to read some data from
 * the serial port, setting the timeout as the select() call would. Data read is
 * stored into the receive buffer, that is then consumed by the win32_ser_read()
 * call.  So win32_ser_select() does both the event waiting and the reading,
 * while win32_ser_read() only consumes the receive buffer.
 */

static void win32_ser_init(struct win32_ser *ws) {
	/* Clear everything */
	memset(ws, 0x00, sizeof(struct win32_ser));

	/* Set file handle to invalid */
	ws->fd = INVALID_HANDLE_VALUE;
}

/* FIXME Try to remove length_to_read -> max_len argument, only used by win32 */
static int win32_ser_select(struct win32_ser *ws, int max_len,
		struct timeval *tv) {
	COMMTIMEOUTS comm_to;
	unsigned int msec = 0;

	/* Check if some data still in the buffer to be consumed */
	if (ws->n_bytes > 0) {
		return 1;
	}

	/* Setup timeouts like select() would do.
	 FIXME Please someone on Windows can look at this?
	 Does it possible to use WaitCommEvent?
	 When tv is NULL, MAXDWORD isn't infinite!
	 */
	if (tv == NULL) {
		msec = MAXDWORD;
	} else {
		msec = tv->tv_sec * 1000 + tv->tv_usec / 1000;
		if (msec < 1)
		msec = 1;
	}

	comm_to.ReadIntervalTimeout = msec;
	comm_to.ReadTotalTimeoutMultiplier = 0;
	comm_to.ReadTotalTimeoutConstant = msec;
	comm_to.WriteTotalTimeoutMultiplier = 0;
	comm_to.WriteTotalTimeoutConstant = 1000;
	SetCommTimeouts(ws->fd, &comm_to);

	/* Read some bytes */
	if ((max_len > PY_BUF_SIZE) || (max_len < 0)) {
		max_len = PY_BUF_SIZE;
	}

	if (ReadFile(ws->fd, &ws->buf, max_len, &ws->n_bytes, NULL)) {
		/* Check if some bytes available */
		if (ws->n_bytes > 0) {
			/* Some bytes read */
			return 1;
		} else {
			/* Just timed out */
			return 0;
		}
	} else {
		/* Some kind of error */
		return -1;
	}
}

static int win32_ser_read(struct win32_ser *ws, uint8_t *p_msg,
		unsigned int max_len) {
	unsigned int n = ws->n_bytes;

	if (max_len < n) {
		n = max_len;
	}

	if (n > 0) {
		memcpy(p_msg, ws->buf, n);
	}

	ws->n_bytes -= n;

	return n;
}
#endif

ssize_t _modbus_asc_send(modbus_t *ctx, const uint8_t *req, int req_length) {
#if defined(_WIN32)
	modbus_rtu_t *ctx_rtu = ctx->backend_data;
	DWORD n_bytes = 0;
	return (WriteFile(ctx_rtu->w_ser.fd, req, req_length, &n_bytes, NULL)) ? n_bytes : -1;
#else
	return write(ctx->s, req, req_length);
#endif
}

ssize_t _modbus_asc_recv(modbus_t *ctx, uint8_t *rsp, int rsp_length) {
#if defined(_WIN32)
	return win32_ser_read(&((modbus_rtu_t *)ctx->backend_data)->w_ser, rsp, rsp_length);
#else
	return read(ctx->s, rsp, rsp_length);
#endif
}

int _modbus_asc_recv_msg_post(uint8_t *rsq, int rsq_length) {
	int i;
	int offset = _MODBUS_ASC_HEADER_LENGTH;
	int check = _MODBUS_ASC_CHECKSUM_LENGTH;
	int len = (rsq_length - offset - check) / 2;
	uint8_t _rsp[rsq_length];

	for (i = 0; i < len; i++)
		_modbus_asc2hex(&rsq[offset + i * 2], &_rsp[i]);

	memmove(&rsq[offset], _rsp, len);
	memmove(&rsq[offset + len], &rsq[rsq_length - check], check);
	return rsq_length - len;
}

int _modbus_asc_flush(modbus_t *);

/* The check_crc8 function shall return the message length if the CRC is
 valid. Otherwise it shall return -1 and set errno to EMBADCRC. */
int _modbus_asc_check_integrity(modbus_t *ctx, uint8_t *msg, const int msg_length) {
	uint8_t crc_calculated;
	uint8_t crc_received = 0;

	crc_calculated = crc8_asc_slave(&msg[1], msg_length - _MODBUS_ASC_CHECKSUM_LENGTH - 1);
	_modbus_asc2hex(&msg[msg_length - _MODBUS_ASC_CHECKSUM_LENGTH], &crc_received);

	/* Check CRC¡¢CR and LF of msg */
	if (crc_calculated == crc_received && _MODBUS_ASC_CR == msg[msg_length - 2]
	        && _MODBUS_ASC_LF == msg[msg_length - 1]) {
		return msg_length;
	} else {
		if (ctx->debug) {
			zlog_debug("CRC received %0x != CRC calculated %0x", crc_received, crc_calculated);
		}
		if (ctx->error_recovery & MODBUS_ERROR_RECOVERY_PROTOCOL) {
			_modbus_asc_flush(ctx);
		}
		errno = EMBBADCRC;
		return -1;
	}
}

/* Sets up a serial port for ASC communications */
static int _modbus_asc_connect(modbus_t *ctx) {
#if defined(_WIN32)
	DCB dcb;
#else
	struct termios tios;
	speed_t speed;
#endif
	modbus_asc_t *ctx_asc = ctx->backend_data;

	if (ctx->debug) {
		zlog_info("Opening %s at %d bauds (%c, %d, %d)", ctx_asc->device, ctx_asc->baud, ctx_asc->parity,
		        ctx_asc->data_bit, ctx_asc->stop_bit);
	}

#if defined(_WIN32)
	/* Some references here:
	 * http://msdn.microsoft.com/en-us/library/aa450602.aspx
	 */
	win32_ser_init(&ctx_asc->w_ser);

	/* ctx_rtu->device should contain a string like "COMxx:" xx being a decimal
	 * number */
	ctx_asc->w_ser.fd = CreateFileA(ctx_asc->device,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			0,
			NULL);

	/* Error checking */
	if (ctx_asc->w_ser.fd == INVALID_HANDLE_VALUE) {
		zlog_debug("Can't open the device %s (%s)", ctx_asc->device, strerror(errno));
		return -1;
	}

	/* Save params */
	ctx_asc->old_dcb.DCBlength = sizeof(DCB);
	if (!GetCommState(ctx_asc->w_ser.fd, &ctx_asc->old_dcb)) {
		zlog_debug("Error getting configuration (LastError %d)", (int)GetLastError());
		CloseHandle(ctx_asc->w_ser.fd);
		ctx_asc->w_ser.fd = INVALID_HANDLE_VALUE;
		return -1;
	}

	/* Build new configuration (starting from current settings) */
	dcb = ctx_asc->old_dcb;

	/* Speed setting */
	switch (ctx_asc->baud) {
		case 110:
		dcb.BaudRate = CBR_110;
		break;
		case 300:
		dcb.BaudRate = CBR_300;
		break;
		case 600:
		dcb.BaudRate = CBR_600;
		break;
		case 1200:
		dcb.BaudRate = CBR_1200;
		break;
		case 2400:
		dcb.BaudRate = CBR_2400;
		break;
		case 4800:
		dcb.BaudRate = CBR_4800;
		break;
		case 9600:
		dcb.BaudRate = CBR_9600;
		break;
		case 19200:
		dcb.BaudRate = CBR_19200;
		break;
		case 38400:
		dcb.BaudRate = CBR_38400;
		break;
		case 57600:
		dcb.BaudRate = CBR_57600;
		break;
		case 115200:
		dcb.BaudRate = CBR_115200;
		break;
		default:
		dcb.BaudRate = CBR_9600;
		zlog_warn("Unknown baud rate %d for %s (B9600 used)", ctx_asc->baud, ctx_asc->device);
	}

	/* Data bits */
	switch (ctx_asc->data_bit) {
		case 5:
		dcb.ByteSize = 5;
		break;
		case 6:
		dcb.ByteSize = 6;
		break;
		case 7:
		dcb.ByteSize = 7;
		break;
		case 8:
		default:
		dcb.ByteSize = 8;
		break;
	}

	/* Stop bits */
	if (ctx_asc->stop_bit == 1)
	dcb.StopBits = ONESTOPBIT;
	else /* 2 */
	dcb.StopBits = TWOSTOPBITS;

	/* Parity */
	if (ctx_asc->parity == 'N') {
		dcb.Parity = NOPARITY;
		dcb.fParity = FALSE;
	} else if (ctx_asc->parity == 'E') {
		dcb.Parity = EVENPARITY;
		dcb.fParity = TRUE;
	} else {
		/* odd */
		dcb.Parity = ODDPARITY;
		dcb.fParity = TRUE;
	}

	/* Hardware handshaking left as default settings retrieved */

	/* No software handshaking */
	dcb.fTXContinueOnXoff = TRUE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;

	/* Binary mode (it's the only supported on Windows anyway) */
	dcb.fBinary = TRUE;

	/* Don't want errors to be blocking */
	dcb.fAbortOnError = FALSE;

	/* TODO: any other flags!? */

	/* Setup port */
	if (!SetCommState(ctx_asc->w_ser.fd, &dcb)) {
		zlog_debug("Error setting new configuration (LastError %d)", (int)GetLastError());
		CloseHandle(ctx_asc->w_ser.fd);
		ctx_asc->w_ser.fd = INVALID_HANDLE_VALUE;
		return -1;
	}
#else
	/* The O_NOCTTY flag tells UNIX that this program doesn't want
	 to be the "controlling terminal" for that port. If you
	 don't specify this then any input (such as keyboard abort
	 signals and so forth) will affect your process

	 Timeouts are ignored in canonical input mode or when the
	 NDELAY option is set on the file via open or fcntl */
	ctx->s = open(ctx_asc->device, O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL);
	if (ctx->s == -1) {
		zlog_debug("Can't open the device %s (%s)", ctx_asc->device, strerror(errno));
		return -1;
	}

	/* Save */
	tcgetattr(ctx->s, &(ctx_asc->old_tios));

	memset(&tios, 0, sizeof(struct termios));

	/* C_ISPEED     Input baud (new interface)
	 C_OSPEED     Output baud (new interface)
	 */
	switch (ctx_asc->baud) {
	case 110:
		speed = B110;
		break;
	case 300:
		speed = B300;
		break;
	case 600:
		speed = B600;
		break;
	case 1200:
		speed = B1200;
		break;
	case 2400:
		speed = B2400;
		break;
	case 4800:
		speed = B4800;
		break;
	case 9600:
		speed = B9600;
		break;
	case 19200:
		speed = B19200;
		break;
	case 38400:
		speed = B38400;
		break;
	case 57600:
		speed = B57600;
		break;
	case 115200:
		speed = B115200;
		break;
	default:
		speed = B9600;
		if (ctx->debug) {
			zlog_warn("Unknown baud rate %d for %s (B9600 used)", ctx_asc->baud, ctx_asc->device);
		}
	}

	/* Set the baud rate */
	if ((cfsetispeed(&tios, speed) < 0) || (cfsetospeed(&tios, speed) < 0)) {
		close(ctx->s);
		ctx->s = -1;
		return -1;
	}

	/* C_CFLAG      Control options
	 CLOCAL       Local line - do not change "owner" of port
	 CREAD        Enable receiver
	 */
	tios.c_cflag |= (CREAD | CLOCAL);
	/* CSIZE, HUPCL, CRTSCTS (hardware flow control) */

	/* Set data bits (5, 6, 7, 8 bits)
	 CSIZE        Bit mask for data bits
	 */
	tios.c_cflag &= ~CSIZE;
	switch (ctx_asc->data_bit) {
	case 5:
		tios.c_cflag |= CS5;
		break;
	case 6:
		tios.c_cflag |= CS6;
		break;
	case 7:
		tios.c_cflag |= CS7;
		break;
	case 8:
	default:
		tios.c_cflag |= CS8;
		break;
	}

	/* Stop bit (1 or 2) */
	if (ctx_asc->stop_bit == 1)
		tios.c_cflag &= ~ CSTOPB;
	else
		/* 2 */
		tios.c_cflag |= CSTOPB;

	/* PARENB       Enable parity bit
	 PARODD       Use odd parity instead of even */
	if (ctx_asc->parity == 'N') {
		/* None */
		tios.c_cflag &= ~ PARENB;
	} else if (ctx_asc->parity == 'E') {
		/* Even */
		tios.c_cflag |= PARENB;
		tios.c_cflag &= ~ PARODD;
	} else {
		/* Odd */
		tios.c_cflag |= PARENB;
		tios.c_cflag |= PARODD;
	}

	/* Read the man page of termios if you need more information. */

	/* This field isn't used on POSIX systems
	 tios.c_line = 0;
	 */

	/* C_LFLAG      Line options

	 ISIG Enable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals
	 ICANON       Enable canonical input (else raw)
	 XCASE        Map uppercase \lowercase (obsolete)
	 ECHO Enable echoing of input characters
	 ECHOE        Echo erase character as BS-SP-BS
	 ECHOK        Echo NL after kill character
	 ECHONL       Echo NL
	 NOFLSH       Disable flushing of input buffers after
	 interrupt or quit characters
	 IEXTEN       Enable extended functions
	 ECHOCTL      Echo control characters as ^char and delete as ~?
	 ECHOPRT      Echo erased character as character erased
	 ECHOKE       BS-SP-BS entire line on line kill
	 FLUSHO       Output being flushed
	 PENDIN       Retype pending input at next read or input char
	 TOSTOP       Send SIGTTOU for background output

	 Canonical input is line-oriented. Input characters are put
	 into a buffer which can be edited interactively by the user
	 until a CR (carriage return) or LF (line feed) character is
	 received.

	 Raw input is unprocessed. Input characters are passed
	 through exactly as they are received, when they are
	 received. Generally you'll deselect the ICANON, ECHO,
	 ECHOE, and ISIG options when using raw input
	 */

	/* Raw input */
	tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	/* C_IFLAG      Input options

	 Constant     Description
	 INPCK        Enable parity check
	 IGNPAR       Ignore parity errors
	 PARMRK       Mark parity errors
	 ISTRIP       Strip parity bits
	 IXON Enable software flow control (outgoing)
	 IXOFF        Enable software flow control (incoming)
	 IXANY        Allow any character to start flow again
	 IGNBRK       Ignore break condition
	 BRKINT       Send a SIGINT when a break condition is detected
	 INLCR        Map NL to CR
	 IGNCR        Ignore CR
	 ICRNL        Map CR to NL
	 IUCLC        Map uppercase to lowercase
	 IMAXBEL      Echo BEL on input line too long
	 */
	if (ctx_asc->parity == 'N') {
		/* None */
		tios.c_iflag &= ~INPCK;
	} else {
		tios.c_iflag |= INPCK;
	}

	/* Software flow control is disabled */
	tios.c_iflag &= ~(IXON | IXOFF | IXANY);

	/* C_OFLAG      Output options
	 OPOST        Postprocess output (not set = raw output)
	 ONLCR        Map NL to CR-NL

	 ONCLR ant others needs OPOST to be enabled
	 */

	/* Raw ouput */
	tios.c_oflag &= ~ OPOST;

	/* C_CC         Control characters
	 VMIN         Minimum number of characters to read
	 VTIME        Time to wait for data (tenths of seconds)

	 UNIX serial interface drivers provide the ability to
	 specify character and packet timeouts. Two elements of the
	 c_cc array are used for timeouts: VMIN and VTIME. Timeouts
	 are ignored in canonical input mode or when the NDELAY
	 option is set on the file via open or fcntl.

	 VMIN specifies the minimum number of characters to read. If
	 it is set to 0, then the VTIME value specifies the time to
	 wait for every character read. Note that this does not mean
	 that a read call for N bytes will wait for N characters to
	 come in. Rather, the timeout will apply to the first
	 character and the read call will return the number of
	 characters immediately available (up to the number you
	 request).

	 If VMIN is non-zero, VTIME specifies the time to wait for
	 the first character read. If a character is read within the
	 time given, any read will block (wait) until all VMIN
	 characters are read. That is, once the first character is
	 read, the serial interface driver expects to receive an
	 entire packet of characters (VMIN bytes total). If no
	 character is read within the time allowed, then the call to
	 read returns 0. This method allows you to tell the serial
	 driver you need exactly N bytes and any read call will
	 return 0 or N bytes. However, the timeout only applies to
	 the first character read, so if for some reason the driver
	 misses one character inside the N byte packet then the read
	 call could block forever waiting for additional input
	 characters.

	 VTIME specifies the amount of time to wait for incoming
	 characters in tenths of seconds. If VTIME is set to 0 (the
	 default), reads will block (wait) indefinitely unless the
	 NDELAY option is set on the port with open or fcntl.
	 */
	/* Unused because we use open with the NDELAY option */
	tios.c_cc[VMIN] = 0;
	tios.c_cc[VTIME] = 0;

	if (tcsetattr(ctx->s, TCSANOW, &tios) < 0) {
		close(ctx->s);
		ctx->s = -1;
		return -1;
	}
#endif

#if HAVE_DECL_TIOCSRS485
	/* The RS232 mode has been set by default */
	ctx_rtu->serial_mode = MODBUS_RTU_RS232;
#endif

	return 0;
}

int modbus_asc_set_serial_mode(modbus_t *ctx, int mode) {
	if (ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU) {
#if HAVE_DECL_TIOCSRS485
		modbus_rtu_t *ctx_rtu = ctx->backend_data;
		struct serial_rs485 rs485conf;
		memset(&rs485conf, 0x0, sizeof(struct serial_rs485));

		if (mode == MODBUS_RTU_RS485) {
			rs485conf.flags = SER_RS485_ENABLED;
			if (ioctl(ctx->s, TIOCSRS485, &rs485conf) < 0) {
				return -1;
			}

			ctx_rtu->serial_mode |= MODBUS_RTU_RS485;
			return 0;
		} else if (mode == MODBUS_RTU_RS232) {
			if (ioctl(ctx->s, TIOCSRS485, &rs485conf) < 0) {
				return -1;
			}

			ctx_rtu->serial_mode = MODBUS_RTU_RS232;
			return 0;
		}
#else
		if (ctx->debug) {
			zlog_trace("This function isn't supported on your platform");
		}
		errno = ENOTSUP;
		return -1;
#endif
	}

	/* Wrong backend and invalid mode specified */
	errno = EINVAL;
	return -1;
}

int modbus_asc_get_serial_mode(modbus_t *ctx) {
	if (ctx->backend->backend_type == _MODBUS_BACKEND_TYPE_RTU) {
#if HAVE_DECL_TIOCSRS485
		modbus_rtu_t *ctx_rtu = ctx->backend_data;
		return ctx_rtu->serial_mode;
#else
		if (ctx->debug) {
			zlog_trace("This function isn't supported on your platform");
		}
		errno = ENOTSUP;
		return -1;
#endif
	} else {
		errno = EINVAL;
		return -1;
	}
}

void _modbus_asc_close(modbus_t *ctx) {
	/* Closes the file descriptor in RTU mode */
	modbus_asc_t *ctx_asc = ctx->backend_data;

#if defined(_WIN32)
	/* Revert settings */
	if (!SetCommState(ctx_asc->w_ser.fd, &ctx_asc->old_dcb)) {
		zlog_debug("Couldn't revert to configuration (LastError %d)", (int)GetLastError());
	}

	if (!CloseHandle(ctx_asc->w_ser.fd)) {
		zlog_debug( "Error while closing handle (LastError %d)", (int)GetLastError());
	}
#else
	tcsetattr(ctx->s, TCSANOW, &(ctx_asc->old_tios));
	close(ctx->s);
#endif
}

int _modbus_asc_flush(modbus_t *ctx) {
#if defined(_WIN32)
	modbus_rtu_t *ctx_rtu = ctx->backend_data;
	ctx_rtu->w_ser.n_bytes = 0;
	return (FlushFileBuffers(ctx_rtu->w_ser.fd) == FALSE);
#else
	return tcflush(ctx->s, TCIOFLUSH);
#endif
}

int _modbus_asc_select(modbus_t *ctx, fd_set *rfds, struct timeval *tv, int length_to_read) {
	int s_rc;
#if defined(_WIN32)
	s_rc = win32_ser_select(&(((modbus_rtu_t*)ctx->backend_data)->w_ser),
			length_to_read, tv);
	if (s_rc == 0) {
		errno = ETIMEDOUT;
		return -1;
	}

	if (s_rc < 0) {
		return -1;
	}
#else
	while ((s_rc = select(ctx->s + 1, rfds, NULL, NULL, tv)) == -1) {
		if (errno == EINTR) {
			if (ctx->debug) {
				zlog_trace("A non blocked signal was caught");
			}
			/* Necessary after an error */
			FD_ZERO(rfds);
			FD_SET(ctx->s, rfds);
		} else {
			return -1;
		}
	}

	if (s_rc == 0) {
		/* Timeout */
		errno = ETIMEDOUT;
		return -1;
	}
#endif

	return s_rc;
}

int _modbus_asc_filter_request(modbus_t *ctx, int slave) {
	/* Filter on the Modbus unit identifier (slave) in RTU mode */
	if (slave != ctx->slave && slave != MODBUS_BROADCAST_ADDRESS) {
		/* Ignores the request (not for me) */
		if (ctx->debug) {
			zlog_trace("Request for slave %d ignored (not %d)", slave, ctx->slave);
		}
		return 1;
	} else {
		return 0;
	}
}

const modbus_backend_t _modbus_asc_backend = { _MODBUS_BACKEND_TYPE_RTU,
_MODBUS_ASC_HEADER_LENGTH,
_MODBUS_ASC_CHECKSUM_LENGTH,
MODBUS_ASC_MAX_ADU_LENGTH, _modbus_set_slave, _modbus_asc_build_request_basis,
        _modbus_asc_build_response_basis, _modbus_asc_prepare_response_tid, _modbus_asc_send_msg_pre,
        _modbus_asc_send, _modbus_asc_recv, _modbus_asc_recv_msg_post, _modbus_asc_check_integrity,
        NULL, _modbus_asc_connect, _modbus_asc_close, _modbus_asc_flush, _modbus_asc_select,
        _modbus_asc_filter_request };

modbus_t* modbus_new_asc(const char *device, int baud, char parity, int data_bit, int stop_bit) {
	modbus_t *ctx;
	modbus_asc_t *ctx_asc;
	size_t dest_size;
	size_t ret_size;

	ctx = (modbus_t *) malloc(sizeof(modbus_t));
	_modbus_init_common(ctx, _MODBUS_ASCII);

	ctx->backend = &_modbus_asc_backend;
	ctx->backend_data = (modbus_asc_t *) malloc(sizeof(modbus_asc_t));
	ctx_asc = (modbus_asc_t *) ctx->backend_data;

	dest_size = sizeof(ctx_asc->device);
	ret_size = strlcpy(ctx_asc->device, device, dest_size);
	if (ret_size == 0) {
		zlog_trace("The device string is empty");
		modbus_free(ctx);
		errno = EINVAL;
		return NULL;
	}

	if (ret_size >= dest_size) {
		zlog_trace("The device string has been truncated");
		modbus_free(ctx);
		errno = EINVAL;
		return NULL;
	}

	ctx_asc->baud = baud;
	if (parity == 'N' || parity == 'E' || parity == 'O') {
		ctx_asc->parity = parity;
	} else {
		modbus_free(ctx);
		errno = EINVAL;
		return NULL;
	}
	ctx_asc->data_bit = data_bit;
	ctx_asc->stop_bit = stop_bit;

	return ctx;
}
