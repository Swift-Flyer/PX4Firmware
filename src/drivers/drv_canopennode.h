/*
 * drv_canopennode.h
 *
 *  Created on: Nov 22, 2014
 *      Author: mstrindga
 */

#ifndef DRV_CANOPENNODE_H_
#define DRV_CANOPENNODE_H_

#define CANOPEN_NODE_PATH "/dev/canopennode"

#define _CANOPEN_BASE		0x6700

/** arm all servo outputs handle by this driver */
#define CANOPEN_ARM		_IOC(_CANOPEN_BASE, 0)

/** arm all esc's */
#define CANOPEN_ARM_ESC _IOC(_CANOPEN_BASE, 1)


#endif /* DRV_CANOPENNODE_H_ */
