/*
 * CAN driver for MazCAN Project MZC/CORTEX-M3
 *
 * Copyright (C) 2009-2011 Jesus Felipe Trujillo Rodriguez <elyeyus@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published
 * by the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/init.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>

#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>

MODULE_AUTHOR("Jesus Felipe Trujillo Rodriguez <elyeyus@gmail.com>");
MODULE_DESCRIPTION("CAN driver for MazCAN Project MZC/CORTEX-M3");
MODULE_LICENSE("GPL v2");

/* Mode register NXP LPC17xx CAN Controller */
#define LPC17xx_MOD_NORMAL 	0x00
#define LPC17xx_MOD_RM     	0x01
#define LPC17xx_MOD_LOM    	0x02
#define LPC17xx_MOD_STM    	0x03
#define LPC17xx_MOD_SLEEP  	0x10

/* Error register NXP LPC17xx CAN Controller */
#define LPC17xx_MASK_ERRBIT 	(0xF<<16)
#define LPC17xx_MASK_ERRDIR   	(1<<21)
#define LPC17xx_MASK_ERRC	(0x3<<22)
#define LPC17xx_MASK_ALCBIT 	(0xFF<<24)

#define LPC17xx_ERRDIR_RECEIVE	1
#define LPC17xx_ERRDIR_SEND	0
#define LPC17xx_ERRC_BIT	0
#define LPC17xx_ERRC_FORM	1
#define LPC17xx_ERRC_STUFF	2
#define LPC17xx_ERRC_OTHER	3

/* CAN Global Status Register register content */
#define LPC17xx_GSR_BS 		0x80
#define LPC17xx_GSR_ES 		0x40
#define LPC17xx_GSR_RXERR 	(0xFF<<16)
#define LPC17xx_GSR_TXERR 	(0xFF<<24)

/* CAN Bittiming Register */
#define LPC17xx_BTR_BRP_BITMASK 	0x00001FF
#define LPC17xx_BTR_SJW_BITMASK		0x000C000
#define LPC17xx_BTR_TESG1_BITMASK	0x00F0000
#define LPC17xx_BTR_TESG2_BITMASK	0x0700000
#define LPC17xx_BTR_SAM_BITMASK		0x0800000

/* CAN MOD Status Register */
#define LPC17xx_MOD_BITMASK 	0xFF

/* mzc_msg types */
#define MZC_MSG_TYPE_NOMSG	0x00	/* Messages */
#define MZC_MSG_TYPE_STD_DATA	0x01
#define MZC_MSG_TYPE_EXT_DATA	0x02
#define MZC_MSG_TYPE_STD_RTR	0x03
#define MZC_MSG_TYPE_EXT_RTR	0x04
#define MZC_MSG_TYPE_IDENT	0x10	/* Identification */
#define MZC_MSG_TYPE_PARAMS	0x11
#define MZC_MSG_TYPE_OVERRUN	0x20	/* Error */
#define MZC_MSG_TYPE_ERROR	0x40
#define MZC_MSG_TYPE_INIT	0x80	/* Set parameters messages */
#define MZC_MSG_TYPE_DEINIT	0x81
#define MZC_MSG_TYPE_SET_MOD	0x82	
#define MZC_MSG_TYPE_SET_BTR	0x83
#define MZC_MSG_TYPE_GET_PARAMS 0x84


/* Define these values to match your devices */
#define USB_MZCUSB_VENDOR_ID 		0xFFFF
#define USB_MZCUSB_PRODUCT_ID 		0x1337

// CAN Peripheral clock PCLK = 100000000 -> PCLK/2 = 50000000
#define MZC_USB_CAN_CLOCK 	50000000

/* This values matches the interfaces of the mzc usb */
#define IF1_ENDPOINT 		2
#define IF1_INTERFACE 		0
#define IF2_ENDPOINT		5
#define IF2_INTERFACE		1

struct mzc_msg {
	uint8_t type;
	uint8_t generic[15];
};

struct mzc_msg_can_frame {
	uint8_t type;
	uint8_t length;
	uint32_t id;
	uint8_t data[8];
};

struct mzc_msg_ident {
	uint8_t type;
	unsigned char string[12];
};

struct mzc_msg_params {
	u8 type;
	u8 mod;
	u32 gsr;
	u32 icr;
	u32 btr;
};

struct mzc_msg_error {
	u8 type;
	u8 txerr;
	u8 rxerr;
	u32 icr;
};

struct USB_MSG_Type {
	struct mzc_msg messages[4];
};

/*
 * Table of devices that work with this driver
 * NOTE: This driver supports only MZC v1 LPC17xx CORTEX-M3
 */
static struct usb_device_id mzc_usb_table[] = {
	{USB_DEVICE(USB_MZCUSB_VENDOR_ID, USB_MZCUSB_PRODUCT_ID)},
	{} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, mzc_usb_table);

/*
 * Size of the Reception endpoints of the MZC-USB interface EP2 and EP5 (IN)
 */
#define RX_BUFFER_SIZE      64
#define CONTROL_BUFFER_SIZE 16
#define RX_MAX_MESSAGES	    4

/*
 * Number of URB (USB Request Blocks) asigned to each reception endpoint
 */
#define MAX_RX_URBS 5

struct mzc_usb_device {
	struct net_device *netdev[2];
} mzc_usb_private;

struct mzc_usb;

struct mzc_usb {
	struct can_priv can; /* must be the first member */
	int open_time;

	struct usb_device *udev;
	struct net_device *netdev;

	struct usb_anchor rx_submitted;

	struct mzc_msg_params active_params; /* active controller parameters */
};

static void mzc_usb_rx_can_msg(struct mzc_usb *dev, struct mzc_msg_can_frame *msg)	// DONE
{
	struct can_frame *cf;
	struct sk_buff *skb;
	int i;
	struct net_device_stats *stats = &dev->netdev->stats;

	skb = alloc_can_skb(dev->netdev, &cf);
	if (skb == NULL)
		return;

	cf->can_id = le32_to_cpu(msg->id);
	cf->can_dlc = get_can_dlc(msg->length & 0xF);

	if (msg->type == MZC_MSG_TYPE_EXT_DATA ||
	    msg->type == MZC_MSG_TYPE_EXT_RTR)
		cf->can_id |= CAN_EFF_FLAG;

	if (msg->type == MZC_MSG_TYPE_STD_RTR ||
	    msg->type == MZC_MSG_TYPE_EXT_RTR) {
		cf->can_id |= CAN_RTR_FLAG;
	} else {
		for (i = 0; i < cf->can_dlc; i++)
			cf->data[i] = msg->data[i];
	}

	netif_rx(skb);

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
}

static void mzc_usb_rx_err(struct mzc_usb *dev, struct mzc_msg *msg)		//DONE
{
	struct can_frame *cf;
	struct sk_buff *skb;
	struct net_device_stats *stats = &dev->netdev->stats;
	struct mzc_msg_params *params_msg;
	struct mzc_msg_error *error_msg;

	skb = alloc_can_err_skb(dev->netdev, &cf);
	if (skb == NULL)
		return;

	if (msg->type == MZC_MSG_TYPE_PARAMS) {
		params_msg = (struct mzc_msg_params *)msg;
		u32 state = params_msg->gsr;

		if (state & LPC17xx_GSR_BS) {
			dev->can.state = CAN_STATE_BUS_OFF;
			cf->can_id |= CAN_ERR_BUSOFF;

			can_bus_off(dev->netdev);
		} else if (state & LPC17xx_GSR_ES) {
			dev->can.state = CAN_STATE_ERROR_WARNING;
			dev->can.can_stats.error_warning++;
		} else {
			dev->can.state = CAN_STATE_ERROR_ACTIVE;
			dev->can.can_stats.error_passive++;
		}
	} else if (msg->type == MZC_MSG_TYPE_ERROR) {
		error_msg = (struct mzc_msg_error *)msg;
		u32 icr = error_msg->icr;
		u8 txerr = error_msg->txerr;
		u8 rxerr = error_msg->rxerr;

		/* bus error interrupt */
		dev->can.can_stats.bus_error++;
		stats->rx_errors++;

		cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

		switch ( ((icr & LPC17xx_MASK_ERRC)>>22) ) {
		case LPC17xx_ERRC_BIT:
			cf->data[2] |= CAN_ERR_PROT_BIT;
			break;
		case LPC17xx_ERRC_FORM:
			cf->data[2] |= CAN_ERR_PROT_FORM;
			break;
		case LPC17xx_ERRC_STUFF:
			cf->data[2] |= CAN_ERR_PROT_STUFF;
			break;
		default:
			cf->data[2] |= CAN_ERR_PROT_UNSPEC;
			cf->data[3] = ((icr & LPC17xx_MASK_ERRC)>>22);
			break;
		}

		/* Error occured during transmission? */				
		if ((icr & LPC17xx_MASK_ERRDIR) == 0)
			cf->data[2] |= CAN_ERR_PROT_TX;

		if (dev->can.state == CAN_STATE_ERROR_WARNING ||
		    dev->can.state == CAN_STATE_ERROR_PASSIVE) {
			cf->data[1] = (txerr > rxerr) ?
			    CAN_ERR_CRTL_TX_PASSIVE : CAN_ERR_CRTL_RX_PASSIVE;
		}
	} else if (msg->type == MZC_MSG_TYPE_OVERRUN) {					
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;

		stats->rx_over_errors++;
		stats->rx_errors++;
	}

	netif_rx(skb);

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
}

/*
 * callback for bulk IN urb
 */
static void mzc_usb_read_bulk_callback(struct urb *urb)						// DONE
{
	struct mzc_usb *dev = urb->context;
	struct net_device *netdev;
	int retval, endpoint, device;

	// Get the urb device and endpoint number
	endpoint = usb_pipeendpoint(urb->pipe);
	device = usb_pipedevice(urb->pipe);

	// we select the netdev knowing the urb's endpoint
	if(endpoint == IF1_ENDPOINT) {
		netdev = mzc_usb_private.netdev[IF1_INTERFACE];
	} else if (endpoint == IF2_ENDPOINT) {
		netdev = mzc_usb_private.netdev[IF2_INTERFACE];
	} else {
		return;	
	}

	if (!netif_device_present(netdev))
		return;

	switch (urb->status) {
	case 0: /* success */
		break;

	case -ENOENT:
		return;

	default:
		dev_info(netdev->dev.parent, "Rx URB aborted (%d)\n",
			 urb->status);
		goto resubmit_urb;
	}

	// We do the processing only if at least we guarantee a packet
	if (urb->actual_length > 16) { 
		
		u8 *ibuf = urb->transfer_buffer;
		u8 indexes[] = {0,16,32,48,64};
		u8 pointer = 0;
		u8 start = indexes[pointer];

		while (pointer < RX_MAX_MESSAGES) {
			struct mzc_msg *generic_msg;
			struct mzc_msg_can_frame *frame_msg;
			u8 type = ibuf[start];
			

			switch (type) {
				case MZC_MSG_TYPE_PARAMS:
					generic_msg = (struct mzc_msg *)&ibuf[start];
					/* Process CAN state changes */
					mzc_usb_rx_err(dev, generic_msg);
					break;

				case MZC_MSG_TYPE_STD_DATA:
				case MZC_MSG_TYPE_EXT_DATA:
				case MZC_MSG_TYPE_STD_RTR:
				case MZC_MSG_TYPE_EXT_RTR:
					frame_msg = (struct mzc_msg_can_frame *)&ibuf[start];
					mzc_usb_rx_can_msg(dev, frame_msg);
					break;

				case MZC_MSG_TYPE_ERROR:
					generic_msg = (struct mzc_msg *)&ibuf[start];
					/* Process errorframe */
					mzc_usb_rx_err(dev, generic_msg);
					break;

				case MZC_MSG_TYPE_OVERRUN:
					generic_msg = (struct mzc_msg *)&ibuf[start];
					/* Message lost while receiving */
					mzc_usb_rx_err(dev, generic_msg);
					break;
			}

			pointer++;
			start = indexes[pointer];

			if (urb->transfer_buffer_length < 64) {
				dev_err(netdev->dev.parent, "format error received less than RX_BUFFER_SIZE\n");
				break;
			}
		}
	}

resubmit_urb:
	usb_fill_bulk_urb(urb, dev->udev, usb_rcvbulkpipe(dev->udev, endpoint),
			  urb->transfer_buffer, RX_BUFFER_SIZE,
			  mzc_usb_read_bulk_callback, dev);

	retval = usb_submit_urb(urb, GFP_ATOMIC);

	if (retval == -ENODEV)
		netif_device_detach(netdev);
	else if (retval)
		dev_err(netdev->dev.parent,
			"failed resubmitting read bulk urb: %d\n", retval);
}

/*
 * Send the given MZC command synchronously
 */
static int mzc_usb_command_msg(struct mzc_usb *dev, struct mzc_msg *msg)				// DONE
{
	int actual_length, endp, retval;
	u8 message_buf[16];

	struct net_device *netdev = dev->netdev;
	if (netdev == mzc_usb_private.netdev[IF1_INTERFACE]) {
		endp = IF1_ENDPOINT;
		dev_info(netdev->dev.parent,"sending usb command to interface 1 -> endpoint: %d\n",endp);
	} else if (netdev == mzc_usb_private.netdev[IF2_INTERFACE]) {
		endp = IF2_ENDPOINT;
		dev_info(netdev->dev.parent,"sending usb command to interface 2 -> endpoint: %d\n",endp);	
	} else {
		// No such interface
		return;
	}

	/* Copy payload */
	memcpy(&message_buf[0], msg, sizeof(struct mzc_msg));
	dev_info(netdev->dev.parent,"Command: %X %X %X %X\n",message_buf[0],message_buf[1],message_buf[2],message_buf[3]);

	retval = usb_bulk_msg(dev->udev, usb_sndbulkpipe(dev->udev, endp),
			    &message_buf[0],
			    CONTROL_BUFFER_SIZE,
			    &actual_length, HZ*10);
	dev_info(netdev->dev.parent,"command returned %d and sent %d bytes\n",retval,actual_length);
	return retval;
}

/*
 * Change CAN controllers' mode register
 */
static int mzc_usb_write_mode(struct mzc_usb *dev, u8 mode)					// NOT DONE
{
	dev->active_params.mod = mode;
	dev->active_params.type = MZC_MSG_TYPE_SET_MOD;

	return mzc_usb_command_msg(dev, (struct mzc_msg *)&dev->active_params);

}

/*
 * Start interface
 */
static int mzc_usb_start(struct mzc_usb *dev)									// DONE
{
	struct net_device *netdev = dev->netdev;
	int err, i, endp;

	if (netdev == mzc_usb_private.netdev[IF1_INTERFACE]) {
		dev_info(netdev->dev.parent, "mzc_usb_start: starting interface 1\n");
		endp = IF1_ENDPOINT;
	} else if (netdev == mzc_usb_private.netdev[IF2_INTERFACE]) {
		dev_info(netdev->dev.parent, "mzc_usb_start: starting interface 2\n");
		endp = IF2_ENDPOINT;	
	} else {
		dev_err(netdev->dev.parent, "mzc_usb_start: No such interface\n");
		return 1;
	}

	// Interface URBs
	for (i = 0; i < MAX_RX_URBS; i++) {
		struct urb *urb = NULL;
		u8 *buf = NULL;

		/* create a URB, and a buffer for it */
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			dev_err(netdev->dev.parent,
				"No memory left for URBs\n");
			return -ENOMEM;
		}

		buf = usb_alloc_coherent(dev->udev, RX_BUFFER_SIZE, GFP_KERNEL,
					 &urb->transfer_dma);
		if (!buf) {
			dev_err(netdev->dev.parent,
				"No memory left for USB buffer\n");
			usb_free_urb(urb);
			return -ENOMEM;
		}
		dev_info(netdev->dev.parent, "mzc_usb_start: Registering urb(%p) on endpoint %d\n",urb,endp);
		usb_fill_bulk_urb(urb, dev->udev, usb_rcvbulkpipe(dev->udev, endp),
				  buf, RX_BUFFER_SIZE,
				  mzc_usb_read_bulk_callback, dev);
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		usb_anchor_urb(urb, &dev->rx_submitted);								

		err = usb_submit_urb(urb, GFP_KERNEL);
		if (err) {
			if (err == -ENODEV)
				netif_device_detach(dev->netdev);

			usb_unanchor_urb(urb);
			usb_free_coherent(dev->udev, RX_BUFFER_SIZE, buf,
					  urb->transfer_dma);
			break;
		}

		/* Drop reference, USB core will take care of freeing it */
		usb_free_urb(urb);
	}


	/* Did we submit any URBs */
	if (i == 0) {
		dev_warn(netdev->dev.parent, "couldn't setup read URBs\n");
		return err;
	}

	/* Warn if we've couldn't transmit all the URBs */
	if (i < MAX_RX_URBS)
		dev_warn(netdev->dev.parent, "rx performance may be slow\n");


	// Mode config													
	if (dev->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		/* Put device into loopback mode */
		mzc_usb_write_mode(dev, LPC17xx_MOD_STM);
		dev_info(netdev->dev.parent, "the controller is starting in self test mode\n");
	} else if (dev->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		/* Put device into listen-only mode */
		mzc_usb_write_mode(dev, LPC17xx_MOD_LOM);
		dev_info(netdev->dev.parent, "the controller is starting in listen only mode\n");
	} else {
		/* Put device into normal mode */
		mzc_usb_write_mode(dev, LPC17xx_MOD_NORMAL);
		dev_info(netdev->dev.parent, "the controller is starting in normal mode\n");
	}

	dev->can.state = CAN_STATE_ERROR_ACTIVE;

	return 0;
}

static void unlink_all_urbs(struct mzc_usb *dev)							// DONE
{

	usb_kill_anchored_urbs(&dev->rx_submitted);

}

static int mzc_usb_open(struct net_device *netdev)							// DONE
{
	struct mzc_usb *dev = netdev_priv(netdev);
	int err;
	
	if (netdev == mzc_usb_private.netdev[IF1_INTERFACE]) {
		err = mzc_usb_write_mode(dev, LPC17xx_MOD_RM);
	} else if (netdev == mzc_usb_private.netdev[IF2_INTERFACE]) {
		err = mzc_usb_write_mode(dev, LPC17xx_MOD_RM);
	}
	if (err)
		return err;


	/* common open */
	err = open_candev(netdev);
	if (err)
		return err;

	/* finally start device */
	err = mzc_usb_start(dev);
	if (err) {
		if (err == -ENODEV)
			netif_device_detach(dev->netdev);

		dev_warn(netdev->dev.parent, "couldn't start device: %d\n",
			 err);

		close_candev(netdev);

		return err;
	}

	dev->open_time = jiffies;

	netif_start_queue(netdev);

	return 0;
}



static int mzc_usb_close(struct net_device *netdev)				// DONE
{
	struct mzc_usb *dev = netdev_priv(netdev);

	/* Stop polling */
	unlink_all_urbs(dev);

	netif_stop_queue(netdev);

	/* Set CAN controller to reset mode */
	if (mzc_usb_write_mode(dev, LPC17xx_MOD_RM))
		dev_warn(netdev->dev.parent, "couldn't stop device");

	close_candev(netdev);

	dev->open_time = 0;

	return 0;
}

static const struct net_device_ops mzc_usb_netdev_ops = {
	.ndo_open = mzc_usb_open,
	.ndo_stop = mzc_usb_close,
};

static struct can_bittiming_const mzc_usb_bittiming_const = {
	.name = "mzc_usb",
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,
};

static int mzc_usb_set_mode(struct net_device *netdev, enum can_mode mode)		//DONE
{
	struct mzc_usb *dev = netdev_priv(netdev);

	if (!dev->open_time)
		return -EINVAL;

	switch (mode) {
	case CAN_MODE_START:
		if (mzc_usb_write_mode(dev, LPC17xx_MOD_NORMAL))
			dev_warn(netdev->dev.parent, "couldn't start device");

		if (netif_queue_stopped(netdev))
			netif_wake_queue(netdev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int mzc_usb_set_bittiming(struct net_device *netdev)					//DONE
{
	struct mzc_usb *dev = netdev_priv(netdev);
	struct can_bittiming *bt = &dev->can.bittiming;
	u32 btr = 0;

	dev_info(netdev->dev.parent, "[Bittiming] Set request -> brp: %d sjw: %d prop_seg: %d phase_seg1: %d phase_seg2: %d\n",bt->brp,bt->sjw,bt->prop_seg,bt->phase_seg1,bt->phase_seg2);

	btr |= ((bt->brp-1)&LPC17xx_BTR_BRP_BITMASK);
	btr |= ((bt->sjw-1)<<14)&LPC17xx_BTR_SJW_BITMASK;
	btr |= ((bt->prop_seg + bt->phase_seg1 -1)<<16)&LPC17xx_BTR_TESG1_BITMASK;
	btr |= ((bt->phase_seg2-1)<<20)&LPC17xx_BTR_TESG2_BITMASK;

	if (dev->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		btr |= LPC17xx_BTR_SAM_BITMASK;

	dev_info(netdev->dev.parent, "setting BTR=0x%x\n",
		 btr);

	dev->active_params.btr = btr;
	dev->active_params.type = MZC_MSG_TYPE_SET_BTR;

	return mzc_usb_command_msg(dev, (struct mzc_msg *)&dev->active_params);
}

static void init_params_lpc17xx(struct mzc_msg_params *msg)
{

	// Dumb initialization the controller takes care of all	
	msg->type = MZC_MSG_TYPE_INIT;

	msg->mod |= LPC17xx_MOD_RM;
	msg->btr |= 0x00000000;

}

/*
 * probe function for new MZC-USB devices
 */
static int mzc_usb_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	struct net_device *netdev_if1;
	struct net_device *netdev_if2;	
	struct mzc_usb *dev_if1;
	struct mzc_usb *dev_if2;
	int i, err = -ENOMEM;

	netdev_if1 = alloc_candev(sizeof(struct mzc_usb), MAX_RX_URBS);
	mzc_usb_private.netdev[IF1_INTERFACE] = netdev_if1;
	dev_info(&intf->dev, "alloc_candev to netdev_if1 with (%p)\n",netdev_if1);
	if (!netdev_if1) {
		dev_err(&intf->dev, "mzc_usb: Couldn't alloc candev for interface 1\n");
		return -ENOMEM;
	}

	netdev_if2 = alloc_candev(sizeof(struct mzc_usb), MAX_RX_URBS);
	mzc_usb_private.netdev[IF2_INTERFACE] = netdev_if2;
	dev_info(&intf->dev, "alloc_candev to netdev_if2 with (%p)\n",netdev_if2);
	if (!netdev_if2) {
		dev_err(&intf->dev, "mzc_usb: Couldn't alloc candev for interface 2\n");
		return -ENOMEM;
	}

	dev_info(&intf->dev, "Initialized two instances of can device IF1(%p) IF2(%p)\n",mzc_usb_private.netdev[IF1_INTERFACE],mzc_usb_private.netdev[IF2_INTERFACE]);

	dev_if1 = netdev_priv(mzc_usb_private.netdev[IF1_INTERFACE]);
	dev_if2 = netdev_priv(mzc_usb_private.netdev[IF2_INTERFACE]);

	dev_if1->udev = interface_to_usbdev(intf);
	dev_if2->udev = interface_to_usbdev(intf);
	dev_info(&intf->dev, "usb device associated to each interface 1(%p) 2(%p)\n",dev_if1->udev,dev_if2->udev);
	dev_if1->netdev = netdev_if1;
	dev_if2->netdev = netdev_if2;

	dev_if1->can.state = CAN_STATE_STOPPED;
	dev_if1->can.clock.freq = MZC_USB_CAN_CLOCK;
	dev_if1->can.bittiming_const = &mzc_usb_bittiming_const;
	dev_if1->can.do_set_bittiming = mzc_usb_set_bittiming;
	dev_if1->can.do_set_mode = mzc_usb_set_mode;
	dev_if1->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES |
		CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY;

	dev_if2->can.state = CAN_STATE_STOPPED;
	dev_if2->can.clock.freq = MZC_USB_CAN_CLOCK;
	dev_if2->can.bittiming_const = &mzc_usb_bittiming_const;
	dev_if2->can.do_set_bittiming = mzc_usb_set_bittiming;
	dev_if2->can.do_set_mode = mzc_usb_set_mode;
	dev_if2->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES |
		CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY;

	netdev_if1->netdev_ops = &mzc_usb_netdev_ops;
	netdev_if2->netdev_ops = &mzc_usb_netdev_ops;

	netdev_if1->flags |= IFF_ECHO; /* we support local echo */
	netdev_if2->flags |= IFF_ECHO; /* we support local echo */

	init_usb_anchor(&dev_if1->rx_submitted);
	init_usb_anchor(&dev_if2->rx_submitted);

	/* Los datos privados de la interfaz usb seran una estructura que contenga las referencias a los dos
	 * dispositivos de red creados por el driver mzc_usb_private->netdev[0,1]
	 */
	usb_set_intfdata(intf, &mzc_usb_private);

	// El dispositivo parent de los netdev de cada interfaz de red sera el dispositivo usb
	SET_NETDEV_DEV(netdev_if1, &intf->dev);
	SET_NETDEV_DEV(netdev_if2, &intf->dev);

	init_params_lpc17xx(&dev_if1->active_params);
	init_params_lpc17xx(&dev_if2->active_params);

	dev_info(netdev_if1->dev.parent, "sending params via usb to the controller dev_if1(%p) active_params(%p)\n",dev_if1,&dev_if1->active_params);
	err = mzc_usb_command_msg(dev_if1, (struct mzc_msg *)&dev_if1->active_params);
	if (err) {
		dev_err(netdev_if1->dev.parent,
			"couldn't initialize controller if1: %d \n", err);
	}

	dev_info(netdev_if2->dev.parent, "sending params via usb to the controller dev_if2(%p) active_params(%p)\n",dev_if2,&dev_if2->active_params);
	err = mzc_usb_command_msg(dev_if2, (struct mzc_msg *)&dev_if2->active_params);
	if (err) { 
		dev_err(netdev_if2->dev.parent,
			"couldn't initialize controller if2: %d \n", err);
	}

	err = register_candev(netdev_if1);
	if (err) {
		dev_err(netdev_if1->dev.parent,
			"couldn't register CAN1 device: %d\n", err);
	}

	err = register_candev(netdev_if2);
	if (err) {
		dev_err(netdev_if2->dev.parent,
			"couldn't register CAN2 device: %d\n", err);
	}

	return 0;

cleanup_candev:
	free_candev(netdev_if1);
	free_candev(netdev_if2);

	return err;
}

/*
 * called by the usb core when the device is removed from the system
 */
static void mzc_usb_disconnect(struct usb_interface *intf)				// DONE
{
	struct mzc_usb *dev_1 = netdev_priv(mzc_usb_private.netdev[IF1_INTERFACE]);
	struct mzc_usb *dev_2 = netdev_priv(mzc_usb_private.netdev[IF2_INTERFACE]);

	usb_set_intfdata(intf, NULL);

	if (dev_1) {
		unregister_netdev(mzc_usb_private.netdev[IF1_INTERFACE]);
		free_candev(mzc_usb_private.netdev[IF1_INTERFACE]);
		unlink_all_urbs(dev_1);
	}

	if (dev_2) {
		unregister_netdev(mzc_usb_private.netdev[IF2_INTERFACE]);
		free_candev(mzc_usb_private.netdev[IF2_INTERFACE]);
		unlink_all_urbs(dev_2);
	}
}

/* usb specific object needed to register this driver with the usb subsystem */
static struct usb_driver mzc_usb_driver = {
	.name = "mzc_usb",
	.probe = mzc_usb_probe,
	.disconnect = mzc_usb_disconnect,
	.id_table = mzc_usb_table,
};

static int __init mzc_usb_init(void)							//DONE
{
	int err;

	printk(KERN_INFO "MazCAN Project kernel driver loaded\n");
	printk(KERN_INFO "Jesus F. Trujillo 2009-2011\n");

	/* register this driver with the USB subsystem */
	err = usb_register(&mzc_usb_driver);

	if (err) {
		err("usb_register failed. Error number %d\n", err);
		return err;
	}

	return 0;
}

static void __exit mzc_usb_exit(void)							//DONE
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&mzc_usb_driver);
}

module_init(mzc_usb_init);
module_exit(mzc_usb_exit);
