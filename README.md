# snull

This is a port of snull driver from the book "Linux Device Drivers" by
Jonathan Corbet, Alessandro Rubini and Greg Kroah-Hartman, published by 
O'Reilly & Associates which is based on 2.6 Linux kernel to 3.19 Linux
Kernel

This was done purely for my own practice and understanding. No warranty
is attached. I cannot take responsibility for errors or fitness for use.

Functionality:
A) Add the following line in /etc/hosts:
   192.168.0.1 local0
   192.168.0.2 remote0
   192.168.1.2 local1
   192.168.1.1 remote1

B) Add the following lines in /etc/networks:
   snullnet0	192.168.0.0
   snullnet1	192.168.1.0

C) Configure IP Addresses on sn0 and sn1 interfaces:
   ifconfig sn0 local0
   ifconfig sn1 local1

D) Test with ping utility:
   ping -c 4 remote0
 
   ping -c 4 remote1
   

Changes required to move from 2.6 to 3.19 kernel:

1) alloc_netdev api change:

   From taking 3 parameters in 2.6 kernel:
   struct net_device *alloc_netdev( int sizeof_priv,
		 		    const char *name,
				    void (*setup)(struct net_device *)
				  );

   To taking 4 parameters in 3.19 kernel:
   struct net_device *alloc_netdev( int sizeof_priv,
				    const char *name,
				    unsigned char name_assign_type,
				    void (*setup)(struct net_device *)
				  );

2) Device operations assignment:
   
   2.6 kernel:
   device operations assigned directly to struct net_device *dev

   3.19 kernel:
   device oprations assigned to struct net_device_ops *snull_netdev_ops
   which is assigned to struct net_device *dev
				    
3) Added struct header_ops:
   
   2.6 kernel:
   struct net_device *dev;

   dev->rebuild_header  = snull_rebuild_header;
   dev->hard_header     = snull_header;

   3.19 kernel:
   struct net_device *dev;
   struct header_ops snull_header_ops;

   .create = snull_create_header;

   dev->header_ops = &snull_header_ops;

4) Cannot find alternative for NETIF_F_NO_CSUM
   (Assuming Default)

5) For Poll Functionality: use struct napi_struct:
   
   2.6 kernel:
   poll function directly added to .poll device operation:
   dev->poll        = snull_poll;

   3.19 kernel:
   Done through struct napi_struct:

   struct napi_struct added to snull_priv:
  
   In snull_init, add:
   netif_napi_add( dev, &priv->napi, snull_poll, SNULL_NAPI_WEIGHT );

   changed interrupt function: snull_napi_interrupt accordingly
