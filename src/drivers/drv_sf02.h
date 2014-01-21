
/**
 * @file drv_sf02.h
 *
 * SF02 driver interface.
 */

#ifndef _DRV_SF02_H
#define _DRV_SF02_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define SF02_DEFAULT_UART_PORT "/dev/ttyS3"

#define SF02_DEVICE_PATH	"/dev/sf02"


/*
 * ObjDev tag for GPS data.
 */
ORB_DECLARE(sf02);

/*
 * ioctl() definitions
 */
#define _SF02IOCBASE			(0x2800)            //TODO: arbitrary choice...
#define _SF02IOC(_n)		(_IOC(_SF02IOCBASE, _n))

#endif /* _DRV_SF02_H */
