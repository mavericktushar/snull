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

#undef PDEBUG

#ifdef SNULL_DEBUG
#  ifdef __KERNEL__
     //This one if debugging is on, and kernel space
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG "snull: " fmt, ## args)
#  else
     //This one for user space
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) //not debugging: nothing
#endif

#undef PDEBUGG
#define PDEBUGG(fmt, args...) //nothing: it's a placeholder

//NAPI WEIGHT:
#define SNULL_NAPI_WEIGHT 2

//Flags in statusword:
#define SNULL_RX_INTR 0x0001
#define SNULL_TX_INTR 0x0002

//Default Timeout Period:
//In jiffies:
#define SNULL_TIMEOUT	5

extern struct net_device *snull_devs[2];
