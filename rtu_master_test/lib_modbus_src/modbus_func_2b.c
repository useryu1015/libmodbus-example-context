// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <errno.h>
// #include <limits.h>
// #include <time.h>

// #include <config.h>

// #include "modbus.h"
// #include "modbus-private.h"



// /*
//     ---------- Request     Indication ----------
//     | Client | ---------------------->| Server |
//     ---------- Confirmation  Response ----------
// */

// typedef enum {
//     /* Request message on the server side */
//     MSG_INDICATION,
//     /* Request message on the client side */
//     MSG_CONFIRMATION
// } msg_type_t;


// #define MAX_MESSAGE_LENGTH 260
// #define _MODBUS_RTU_PRESET_REQ_LENGTH  6

// /* Builds a RTU request header */
// static int _modbus_rtu_build_request_basis_deviceID(modbus_t *ctx, int function,
//                                            int addr, int nb,
//                                            uint8_t *req)
// {
//     assert(ctx->slave != -1);
//     req[0] = ctx->slave;
//     req[1] = function;
//     req[2] = addr >> 8;
//     req[3] = addr & 0x00ff;
//     req[4] = nb >> 8;
//     req[5] = nb & 0x00ff;

//     return _MODBUS_RTU_PRESET_REQ_LENGTH;
// }

// /* Reads the data from a remove device and put that data into an array */
// static int read_registers_deviceID(modbus_t *ctx, int function, int addr, int nb,
//                           uint16_t *dest)
// {
//     int rc;
//     int req_length;
//     uint8_t req[_MIN_REQ_LENGTH];
//     uint8_t rsp[MAX_MESSAGE_LENGTH];

//     if (nb > MODBUS_MAX_READ_REGISTERS) {
//         if (ctx->debug) {
//             fprintf(stderr,
//                     "ERROR Too many registers requested (%d > %d)\n",
//                     nb, MODBUS_MAX_READ_REGISTERS);
//         }
//         errno = EMBMDATA;
//         return -1;
//     }

//     // req_length = ctx->backend->build_request_basis(ctx, function, addr, nb, req);       // 构建报文头

//     rc = send_msg(ctx, req, req_length);
//     if (rc > 0) {
//         int offset;
//         int i;

//         rc = receive_msg(ctx, rsp, MSG_CONFIRMATION);
//         if (rc == -1)
//             return -1;

//         rc = check_confirmation(ctx, req, rsp, rc);
//         if (rc == -1)
//             return -1;

//         offset = ctx->backend->header_length;

//         for (i = 0; i < rc; i++) {
//             /* shift reg hi_byte to temp OR with lo_byte */
//             dest[i] = (rsp[offset + 2 + (i << 1)] << 8) |
//                 rsp[offset + 3 + (i << 1)];
//         }
//     }

//     return rc;
// }




// /* 
//  * 读取设备标识
//  * 功能码： 0x2B
//  * 
//  * tips： 目前未兼容 tcp 模式
//  */
// modbus_read_deviceID(modbus_t *ctx, int addr, int nb, uint16_t *dest)
// {
//     int status;

//     // if (nb > MODBUS_MAX_READ_REGISTERS) {
//     //     if (ctx->debug) {
//     //         fprintf(stderr,
//     //                 "ERROR Too many registers requested (%d > %d)\n",
//     //                 nb, MODBUS_MAX_READ_REGISTERS);
//     //     }
//     //     errno = EMBMDATA;
//     //     return -1;
//     // }

//     status = read_registers_deviceID(ctx, _FC_READ_DEVICE_ID,
//                             addr, nb, dest);
//     return status;


// }