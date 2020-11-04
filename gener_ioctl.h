#ifndef PS_MON_IOCTL_H
#define PS_MON_IOCTL_H
#include <linux/ioctl.h>
 
typedef struct
{
    unsigned long long int nsec[2];
} gener_arg_t;
 
#define GENER_GET_VARIABLES _IOR('q', 1, gener_arg_t *)
#define GENER_CLR_VARIABLES _IO('q', 2)
#define GENER_SET_VARIABLES _IOW('q', 3, gener_arg_t *)

#endif
