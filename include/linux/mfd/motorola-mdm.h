/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _LINUX_MFD_MOTMDM_H
#define _LINUX_MFD_MOTMDM_H

#include <linux/kfifo.h>
#include <linux/serdev-gsm.h>

enum motmdm_dlci_nr {
	MOTMDM_DLCI1 = 1,
	MOTMDM_DLCI2,
	MOTMDM_DLCI3,
	MOTMDM_DLCI4,
	MOTMDM_DLCI5,
	MOTMDM_DLCI6,
	MOTMDM_DLCI7,
	MOTMDM_DLCI8,
	MOTMDM_DLCI9,
	MOTMDM_DLCI10,
	MOTMDM_DLCI11,
	MOTMDM_DLCI12,
	MOTMDM_DLCI13,
	MOTMDM_DLCI14,
	MOTMDM_DLCI15,
};

enum motmdm_state {
	MOTMDM_STATE_IDLE = 0,
	MOTMDM_STATE_DIAL = 1,
	MOTMDM_STATE_ANSWERING = 2,
	MOTMDM_STATE_CONNECTING = 3,
	MOTMDM_STATE_INCOMING = 4,
	MOTMDM_STATE_CONNECTED = 5,
	MOTMDM_STATE_HANGING_UP = 6,
	MOTMDM_STATE_DISCONNECTED = 7,
};

struct motmdm_dlci {
	struct gsm_serdev_dlci gsm_dlci;
	struct list_head node;
	wait_queue_head_t read_queue;
	struct kfifo read_fifo;
	int line;
	u16 id;
	int (*send_command)(struct device *dev, struct motmdm_dlci *mot_dlci,
			    unsigned long timeout_ms, const unsigned char *cmd,
			    size_t cmdlen,
			    unsigned char *rsp, size_t rsplen);
	int (*handle_command)(struct motmdm_dlci *mot_dlci, int id,
			      const unsigned char *buf, size_t len);
	int (*receive_data)(struct motmdm_dlci *mot_dlci,
			    const unsigned char *buf,
			    size_t len);
	int (*write)(struct device *dev, struct motmdm_dlci *mot_dlci,
		     int cmdid, const unsigned char *buf, size_t count);
	int (*notify)(struct motmdm_dlci *mot_dlci, enum motmdm_state);
	struct list_head list;
	void *privdata;		/* Do not use, internal data */
	void *drvdata;		/* Available for consumer drivers */
};

int motmdm_register_dlci(struct device *dev, struct motmdm_dlci *mot_dlci);
void motmdm_unregister_dlci(struct device *dev, struct motmdm_dlci *mot_dlci);

static inline
int motmdm_send_command(struct device *dev, struct motmdm_dlci *mot_dlci,
			unsigned long timeout_ms, const unsigned char *cmd,
			size_t cmdlen, unsigned char *rsp, size_t rsplen)
{
	if (mot_dlci && mot_dlci->send_command)
		return mot_dlci->send_command(dev, mot_dlci,
					      timeout_ms, cmd, cmdlen,
					      rsp, rsplen);
	else
		return -EINVAL;
}

static inline
int motmdm_write(struct device *dev, struct motmdm_dlci *mot_dlci,
		 const unsigned char *buf, size_t count)
{
	if (mot_dlci && mot_dlci->write)
		return mot_dlci->write(dev, mot_dlci, -1, buf, count);
	else
		return -EINVAL;
}

#endif	/* _LINUX_MFD_MOTMDM_H */
