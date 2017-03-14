/*
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.

 * This is a port of snull driver from the book "Linux Device Drivers" by
 * Jonathan Corbet, Alessandro Rubini and Greg Kroah-Hartman, published by 
 * O'Reilly & Associates.
 *  
 * Port from 2.6 Linux kernel to 3.19 Linux Kernel.

 * This was done purely for my own practice and understanding.
 * No warranty is attached. I cannot take responsibility for errors or
 * fitness for use.
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/in.h>

#include "snull.h"

//Transmitter lockup simulation
//normally disabled.
static int lockup = 0;
module_param( lockup, int, 0 );

static int timeout = SNULL_TIMEOUT;
module_param( timeout, int, 0 );

//variable that determines running in NAPI MODE or NOT:
static int use_napi = 0;
module_param( use_napi, int, 0 );

//Structure: In-Flight Packet:
struct snull_packet {
	struct snull_packet *next;
	struct net_device *dev;
	int datalen;
	u8 data[ETH_DATA_LEN];
};

int pool_size = 8;
module_param( pool_size, int, 0 );

//Structure: Private To Each Device. Used To Pass Packets In/Out
//Place for Packet:
struct snull_priv {
	struct net_device_stats stats;
	int status;

	struct napi_struct napi;

	struct snull_packet *ppool;
	//List Of Incoming Packets:
	struct snull_packet *rx_queue;
	int rx_int_enabled;
	int tx_packetlen;
	u8 *tx_packetdata;
	struct sk_buff *skb;
	spinlock_t lock;
};

//SNULL Interrupt Callback:
static void (*snull_interrupt)(int, void *, struct pt_regs *);

//SNULL setup a devices POOL:
static void snull_setup_pool( struct net_device *dev ) {
	struct snull_priv *priv = netdev_priv( dev );
	int i;
	struct snull_packet *pkt;

	priv->ppool = NULL;
	for( i = 0; i < pool_size; i++ ) {
		pkt = kmalloc( sizeof( struct snull_packet ), GFP_KERNEL );
		if( pkt == NULL ) {
			printk(KERN_NOTICE "snull: ran out of memory allocating packet pool \n");
			return ;
		}

		pkt->dev = dev;
		pkt->next = priv->ppool;
		priv->ppool = pkt;
	}	

	return ;
}

void snull_teardown_pool( struct net_device *dev ) {
	struct snull_priv *priv = netdev_priv( dev );
	struct snull_packet *pkt;

	while( ( pkt = priv->ppool ) ) {
		priv->ppool = pkt->next;
		kfree( pkt );
	}

	return ;
}

void snull_release_buffer( struct snull_packet *pkt ) {
	unsigned long flags;
	struct snull_priv *priv = netdev_priv( pkt->dev );

	spin_lock_irqsave( &priv->lock, flags );
	
	pkt->next = priv->ppool;
	priv->ppool = pkt;
	
	spin_unlock_irqrestore( &priv->lock, flags );
	
	if (netif_queue_stopped(pkt->dev) && pkt->next == NULL) {
		netif_wake_queue(pkt->dev);
	}
}

//SNULL RX:
//Receive a packet: retrieve
//encapsulate
//and pass over to upper levels
void snull_rx( struct net_device *dev, struct snull_packet *pkt ) {
	struct sk_buff *skb;
	struct snull_priv *priv = netdev_priv( dev );

	
	//The packet has been retrieved from the transmission
	//medium. Build an skb around it, so upper layers can handle it
	skb = dev_alloc_skb( pkt->datalen + 2 );
	if( !skb ) {
		if (printk_ratelimit())
			printk(KERN_NOTICE "snull rx: low on mem - packet dropped\n");
		priv->stats.rx_dropped++;
		goto out;
	}

	//Align IP on 16B boundary:
	skb_reserve( skb, 2 );
	memcpy( skb_put( skb, pkt->datalen ), pkt->data, pkt->datalen);

	//Write Metadata:
	//And pass to the receive level:
	skb->dev =dev;
	skb->protocol = eth_type_trans( skb, dev );
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	priv->stats.rx_packets++;
	priv->stats.rx_bytes += pkt->datalen;
	netif_rx( skb );
out:
	return ;
}

//enable/disable receive interrupts:
static void snull_rx_ints( struct net_device *dev, int enable ) {
	struct snull_priv *priv = netdev_priv( dev );
	priv->rx_int_enabled = enable;
}

static void snull_napi_interrupt( int irq, void *dev_id, struct pt_regs *regs ) {
	int statusword;
	struct snull_priv *priv;

	struct net_device *dev = ( struct net_device * ) dev_id;
	
	if( !dev ) {
		return ;
	}

	priv = netdev_priv( dev );

	//Lock The Device:
	spin_lock( &priv->lock );
	
	//retrieve statusword: real netdevices use I/O instructions
	statusword = priv->status;
	priv->status = 0;
	
	if (statusword & SNULL_RX_INTR) {
		snull_rx_ints(dev, 0);  /* Disable further interrupts */
		//netif_rx_schedule(dev);
		__napi_schedule(&priv->napi);
	}

	if (statusword & SNULL_TX_INTR) {
        	/* a transmission is over: free the skb */
		priv->stats.tx_packets++;
		priv->stats.tx_bytes += priv->tx_packetlen;
		dev_kfree_skb(priv->skb);
	}

	//Unlock The Device:
	spin_unlock( &priv->lock );

	return ;
}

static void snull_regular_interrupt( int irq, void *dev_id, struct pt_regs *regs ) {
	int statusword;
	struct snull_priv *priv;
	struct snull_packet *pkt = NULL;

	//Check device pointer if it's really interrupting:
	//Then assign struct net_device *dev

	struct net_device *dev = ( struct net_device * ) dev_id;
	//Check with hardware, if it's really ours:

	if( !dev ) {
		return ;
	}

	priv = netdev_priv( dev );

	//Lock The Device:
	spin_lock( &priv->lock );

	//retrieve statusword: real netdevices use I/O instructions
	statusword = priv->status;
	priv->status = 0;

	if( statusword & SNULL_RX_INTR ) {
		//Send it to snull_rx for handling:
		pkt = priv->rx_queue;
		if (pkt ) {
			priv->rx_queue = pkt->next;
			snull_rx( dev, pkt );
		}
	}

	if( statusword & SNULL_TX_INTR ) {
		//Transmission over:
		//Free the skb:
		priv->stats.tx_packets++;
		priv->stats.tx_bytes += priv->tx_packetlen;
		dev_kfree_skb(priv->skb);
	}

	//Unlock The Device:
	spin_unlock( &priv->lock );

	if( pkt ) {
		snull_release_buffer( pkt );
	}

	return ;
}

//SNULL Open Method:
int snull_open( struct net_device *dev ) {
	//request_region(...)
	//request_irq(...)
	//Generally in Open:
	//1. Request IRQ
	//2. HW INIT
	//3. Buffer Allocation

	//Assign HW Address of the Board:
	//use '\0SNULx', where x is 0 or 1
	//First Byte is '\0' to avoid being a Multicast Address
	//First Byte of MUlticast Address is Odd:

	memcpy( dev->dev_addr, "\0SNUL0", ETH_ALEN );
	if( dev == snull_devs[1] ) {
		dev->dev_addr[ETH_ALEN - 1]++;
	}

	//Start the interfaces xmit queue:
	//(Allowing it to accept packets for transmission)
	//Once it is ready to start sending Data:
	netif_start_queue( dev );

	return 0;
}

//SNULL Release Method:
int snull_release( struct net_device *dev ) {
	//release ports, irq etc.

	netif_stop_queue( dev );

	return 0;
}

//SNULL set config:
int snull_config( struct net_device *dev, struct ifmap *map ) {
	//Can't act on running interface:
	//If interface UP, return -EBUSY
	if( dev->flags & IFF_UP ) {
		return -EBUSY;
	}

	//Not allowed to change the I/O Address:
	if( map->base_addr != dev->base_addr ) {
		printk( KERN_WARNING "snull: Cannot change I/O Address" );
		return -EOPNOTSUPP;
	}

	//Changing IRQ allowed:
	if( map->irq != dev->irq ) {
		dev->irq = map->irq;
	}

	//Ignore other fields:

	return 0;
}

void snull_enqueue_buf( struct net_device *dev, struct snull_packet *pkt ) {
	unsigned long flags;
	struct snull_priv *priv = netdev_priv(dev);

	spin_lock_irqsave( &priv->lock, flags );
	pkt->next = priv->rx_queue;  //FIXME - misorders packets
	priv->rx_queue = pkt;
	spin_unlock_irqrestore( &priv->lock, flags );	

	return ;
}

//Buffer, Pool Management:
struct snull_packet *snull_get_tx_buffer(struct net_device *dev) {
	struct snull_priv *priv = netdev_priv( dev );
	unsigned long flags;
	struct snull_packet *pkt;

	spin_lock_irqsave( &priv->lock, flags );
	
	pkt = priv->ppool;
	priv->ppool = pkt->next;

	if( priv->ppool == NULL ) {
		printk( KERN_INFO "snull: Pool Empty \n");
		netif_stop_queue( dev );
	}

	spin_unlock_irqrestore( &priv->lock, flags );

	return pkt;
}

//Device Specific Send Function:
//Transmit a packet:
//Low Level Interface:
static void snull_hw_tx( char *buf, int len, struct net_device *dev ) {
	//This function deals with hw details. This interface loops
	//back the packet to the other snull interface (if any).
	//In other words, this function implements the snull behaviour,
	//while all other procedures are rather device-independent
	struct iphdr *ih;
	u32 *saddr, *daddr;
	struct net_device *dest;
	struct snull_priv *priv;
	struct snull_packet *tx_buffer;

	//If packet less than ethernet header + ip header:
	if( len < ( sizeof( struct ethhdr ) + sizeof( struct iphdr ) ) ) {
		printk("snull: packet too short (%i octets) \n", len);
		return ;
	}

	//Enable this to look at the Data:
	if(0) {
		int i;
		PDEBUG("len is %i \n" KERN_DEBUG "data: ", len);
		for( i = 14; i < len; i++ ) {
			printk("%02x", ( buf[i] & 0xFF ) );
		}
		printk("\n");
	}

	//Ethhdr is 14 bytes, but the kernel arranges for iphdr
	//to be aligned (i.e., ethhdr is unaligned)
	ih = ( struct iphdr * )( buf + sizeof( struct ethhdr ) );
	saddr = &ih->saddr;
	daddr = &ih->daddr;

	//Change the third octet (class C):
	((u8 *)saddr)[2] ^= 1;
	((u8 *)daddr)[2] ^= 1;

	//and rebuild the checksum (ip needs it):
	ih->check = 0;
	ih->check = ip_fast_csum((unsigned char *)ih,ih->ihl);

	printk(KERN_ALERT "snull ih->ihl: %d \n", ih->ihl);

	if (dev == snull_devs[0])
		PDEBUGG("%08x:%05i --> %08x:%05i\n",
				ntohl(ih->saddr),ntohs(((struct tcphdr *)(ih+1))->source),
				ntohl(ih->daddr),ntohs(((struct tcphdr *)(ih+1))->dest));
	else
		PDEBUGG("%08x:%05i <-- %08x:%05i\n",
				ntohl(ih->daddr),ntohs(((struct tcphdr *)(ih+1))->dest),
				ntohl(ih->saddr),ntohs(((struct tcphdr *)(ih+1))->source));

	//Ok, now the packet is ready for transmission: first simulate a
	//receive interrupt on the twin device, then  a
	//transmission-done on the transmitting device:

	dest = snull_devs[dev == snull_devs[0] ? 1 : 0];
	priv = netdev_priv(dest);
	tx_buffer = snull_get_tx_buffer(dev);
	tx_buffer->datalen = len;
	memcpy(tx_buffer->data, buf, len);
	snull_enqueue_buf(dest, tx_buffer);
	//RX:
	if (priv->rx_int_enabled) {
		priv->status |= SNULL_RX_INTR;
		snull_interrupt(0, dest, NULL);
	}

	//TX:
	priv = netdev_priv( dev );
	priv->tx_packetlen = len;
	priv->tx_packetdata = buf;
	priv->status |= SNULL_TX_INTR;
	if (lockup && ((priv->stats.tx_packets + 1) % lockup) == 0) {
        	//Simulate a dropped transmit interrupt
		netif_stop_queue(dev);
		PDEBUG("Simulate lockup at %ld, txp %ld\n", jiffies,
				(unsigned long) priv->stats.tx_packets);
	}
	else
		snull_interrupt(0, dev, NULL);
}

//SNULL TX:
int snull_tx( struct sk_buff *skb, struct net_device *dev ) {
	int len;
	char *data, shortpkt[ETH_ZLEN];
	struct snull_priv *priv = netdev_priv( dev );

	data = skb->data;
	len = skb->len;

	//If short packet:
	//Make it of appropriate size and pad
	//[ETH_ZLEN -> Min. octets in frame sans FCS]
	if( len < ETH_ZLEN ) {
		printk(KERN_ALERT "snull: len < ETH_ZLEN \n");
		memset( shortpkt, 0, ETH_ZLEN );
		memcpy( shortpkt, skb->data, skb->len );
		len = ETH_ZLEN;
		data = shortpkt;	
	}

	//Save the timestamp:
	dev->trans_start = jiffies;

	//Remember the skb:
	//To free at interrupt time:
	priv->skb = skb;

	//Call device specific function to send Data:
	snull_hw_tx( data, len , dev );

	return 0;
}

//SNULL IOCTL:
int snull_ioctl( struct net_device *dev, struct ifreq *rq, int cmd ) {
	PDEBUG("ioctl \n");

	return 0;
}

//SNULL get stats:
//return statistics to caller:
struct net_device_stats *snull_stats( struct net_device *dev ) {
	struct snull_priv *priv = netdev_priv( dev );

	return &priv->stats;
}

//SNULL change MTU:
//Usually not needed:
int snull_change_mtu( struct net_device *dev, int new_mtu ) {
	unsigned long flags;
	struct snull_priv *priv = netdev_priv( dev );
	spinlock_t *lock = &priv->lock;

	//check ranges:
	if( ( new_mtu < 68 ) && ( new_mtu > 1500 ) ) {
		return -EINVAL;
	}

	spin_lock_irqsave( lock, flags );
	dev->mtu = new_mtu;
	spin_unlock_irqrestore( lock, flags );

	return 0;
}

//SNULL TX timeout:
//Deals with transmit timeout:
void snull_tx_timeout( struct net_device *dev ) {
	struct snull_priv *priv = netdev_priv(dev);

	PDEBUG("Transmit timeout at %ld, latency %ld\n",
		jiffies, jiffies - dev->trans_start);

        //Simulate a transmission interrupt:
	priv->status = SNULL_TX_INTR;
	snull_interrupt(0, dev, NULL);

	//Increment error stats:
	priv->stats.tx_errors++;

	netif_wake_queue(dev);

	return ;
}

//SNULL dequeue buf:
struct snull_packet *snull_dequeue_buf( struct net_device *dev ) {
	struct snull_priv *priv = netdev_priv( dev );
	struct snull_packet *pkt;
	unsigned long flags;

	spin_lock_irqsave( &priv->lock, flags );
	pkt = priv->rx_queue;
	if( pkt != NULL ) {
		priv->rx_queue = pkt->next;
	}
	spin_unlock_irqrestore( &priv->lock, flags );
	
	return pkt;
}

//SNULL poll:
static int snull_poll( struct napi_struct *napi, int budget ) {
	struct snull_priv *priv = container_of( napi, struct snull_priv, napi );
	struct snull_packet *pkt;
	struct sk_buff *skb;
	int npackets = 0;
	//int quota = min(dev->quota, *budget);
	int quota = budget;

	while( ( npackets < quota ) && ( priv->rx_queue ) ) {
		pkt = snull_dequeue_buf( napi->dev );
		skb = dev_alloc_skb( pkt->datalen + 2 );
		if( !skb ) {
			if (printk_ratelimit())
				printk(KERN_NOTICE "snull: packet dropped\n");
			priv->stats.rx_dropped++;
			snull_release_buffer(pkt);
			continue;
		}

		//Align IP on 16B boundary
		skb_reserve(skb, 2);
		memcpy(skb_put(skb, pkt->datalen), pkt->data, pkt->datalen);
		skb->dev = napi->dev;
		skb->protocol = eth_type_trans(skb, napi->dev);
		skb->ip_summed = CHECKSUM_UNNECESSARY; //don't check it
		netif_receive_skb(skb);

		//Maintain stats
		npackets++;
		priv->stats.rx_packets++;
		priv->stats.rx_bytes += pkt->datalen;
		snull_release_buffer(pkt);
	}

	//Processed all packets
	//Tell the Kernel
	//Reenable ints
	budget -= npackets;
	//dev->quota -= npackets;
	if (! priv->rx_queue) {
		//netif_rx_complete(napi->dev);
		napi_complete(napi);
		snull_rx_ints(napi->dev, 1);
		return 0;
	}

	return 1;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
//SNULL poll controller implementation:
static void snull_netpoll( struct net_device *dev ) {

	return ;
}
#endif	

//SNULL create header:
static int snull_create_header( struct sk_buff *skb, struct net_device *dev,
			 unsigned short type, const void *daddr,
			 const void *saddr, unsigned int len) {
	struct ethhdr *eth = ( struct ethhdr * ) skb->data;

	eth->h_proto = htons( type );
	memcpy(eth->h_source, saddr ? saddr : dev->dev_addr, dev->addr_len);
	memcpy(eth->h_dest,   daddr ? daddr : dev->dev_addr, dev->addr_len);
	//dest is us xor 1
	eth->h_dest[ETH_ALEN - 1] ^= 0x01;

	return (dev->hard_header_len);
}

//net_device_ops structure
//Callback for various operations on device(open, release, etc.)
static const struct net_device_ops snull_netdev_ops = {
	.ndo_open = snull_open,
	.ndo_stop = snull_release,
	.ndo_set_config = snull_config,
	.ndo_start_xmit = snull_tx,
	.ndo_do_ioctl = snull_ioctl,
	.ndo_get_stats = snull_stats,
	.ndo_change_mtu = snull_change_mtu,
	//rebuild_header
	//hard_header
	.ndo_tx_timeout = snull_tx_timeout,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = snull_netpoll,
#endif	
};

//header_ops structure
static const struct header_ops snull_header_ops = {
	.create = snull_create_header,
	//Disable Caching:
	.cache = NULL,
};

//init/probe function
//Callback from register_netdev(...)
void snull_init( struct net_device *dev ) {
	struct snull_priv *priv;

	ether_setup( dev );

	dev->netdev_ops = &snull_netdev_ops;
	dev->header_ops = &snull_header_ops;
	dev->watchdog_timeo = timeout;

	dev->flags |= IFF_NOARP;
	
	//*****Cannot find alternative for this:******
	//dev->features |= NETIF_F_NO_CSUM;

	//Initialize the private field:
	//Contains Statistics and few private fields:
	priv = netdev_priv( dev );
	memset(priv, 0, sizeof( struct snull_priv ) );

	netif_napi_add( dev, &priv->napi, snull_poll, SNULL_NAPI_WEIGHT );

	//Initialize spinlock:
	spin_lock_init( &priv->lock );

	//enable receive interrupts:
	snull_rx_ints( dev, 1 );

	//setup POOL:
	snull_setup_pool( dev );
}

//Emulated Devices:
//sn0 and sn1
struct net_device *snull_devs[2];

//Unload The Module:
static void __exit snull_cleanup_module(void) {
	int i;

	printk(KERN_ALERT "snull module unloaded ...");

	for( i =0; i < 2; i++ ) {
		if( snull_devs[i] ) {
			unregister_netdev( snull_devs[i] );
			snull_teardown_pool( snull_devs[i] );
			free_netdev( snull_devs[i] );
		}
	}

	return ;
}

//Load The Module:
static int __init snull_init_module(void) {
	int i, result, ret = -ENOMEM;

	snull_interrupt = use_napi ? snull_napi_interrupt : snull_regular_interrupt;

	//Allocate The Devices:
	
	//Device 1: sn0
	snull_devs[0] = alloc_netdev(
				     sizeof( struct snull_priv ),
				     "sn%d",
				     NET_NAME_UNKNOWN,
				     snull_init
				    );

	//Device 2: sn1
	snull_devs[1] = alloc_netdev(
				     sizeof( struct snull_priv ),
				     "sn%d",
				     NET_NAME_UNKNOWN,
			  	     snull_init
				    );

	if( snull_devs[0] == NULL || snull_devs[1] == NULL ) {
		goto out;
	}

	ret = -ENODEV;
	for( i = 0; i < 2; i++ ) {
		if( ( result = register_netdev(snull_devs[i]) ) ) {
			printk("snull: error %i registering device \"%s\" \n",
				result, snull_devs[i]->name);
		} else {
			ret = 0;
		}
	}

out:	
	if( ret ) {
		snull_cleanup_module();	
	}

	printk(KERN_ALERT "snull module loaded ...");

	return ret;
}

module_init(snull_init_module);
module_exit(snull_cleanup_module);

MODULE_LICENSE("GPL");
