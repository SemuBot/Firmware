/*
 * usb_dcd_transport.h
 *
 *  Created on: Feb 2, 2026
 *      Author: medved
 */

#ifndef __USB_CDC_TRANSPORT_H
#define __USB_CDC_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <uxr/client/transport.h>

extern bool cubemx_transport_open(struct uxrCustomTransport * transport);
extern bool cubemx_transport_close(struct uxrCustomTransport * transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
extern size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t * err);

#ifdef __cplusplus
}
#endif

#endif /* __USB_CDC_TRANSPORT_H */
