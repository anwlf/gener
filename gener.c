/*****************************************************************/
/*                                                               */
/*              Frequency Generator Module. Rev. 1.0.0.1         */
/*              Volkovs, Andrejs, GPL, 2018-2020                 */
/*                                                               */
/*****************************************************************/
#include <linux/init.h>           ///< Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/fs.h>             ///< Header for the Linux file system support
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/time.h>           ///< timespec
#include <asm/div64.h>            ///< for fast div
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>

#include "gener_ioctl.h"

/* device definitions */
#define  DEVICE_NAME "gen"              ///< The device will appear at /dev/genXX using this value
#define  CLASS_NAME  "gen"              ///< The device class -- this is a character device driver
#define  MAX_DEV_CNT 10                 ///< Max number of devices allowed
#define  SEC_NS 1000000000              ///< ns in one second

MODULE_LICENSE("GPL");                  ///< The license type -- this affects available functionality
MODULE_AUTHOR("Volkovs, Andrejs");      ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("A Linux char driver for frequency generation on GPIO output");  ///< The description -- see modinfo
MODULE_VERSION("0.1");                  ///< A version number to inform users

static int    majorNumber;              ///< Stores the device number -- determined automatically
static char   message[2256] = {""};     ///< Memory for the string that is passed from userspace
static short  size_of_message=22;       ///< Used to remember the size of the string stored
static struct class*  genClass  = NULL; ///< The device-driver class struct pointer
static struct device* genDevice[MAX_DEV_CNT] = {NULL};  ///< The device-driver device struct pointers array

static int started[MAX_DEV_CNT] = {0,0};///< Generator was started flag

static int count=2;                     ///< Default number of devices, if no module parameters specified

/* The prototype functions for the character driver -- must come before the struct definition */
static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int dev_ioctl(struct inode *i, struct file *f, unsigned int cmd, unsigned long arg);
#else
static long dev_ioctl(struct file *f, unsigned int cmd, unsigned long arg);
#endif


/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 *  /linux/fs.h lists the callback functions that you wish to associated with your file operations
 *  using a C99 syntax structure. char devices usually implement open, read, write and release calls
 */
static struct file_operations fops =
{
   .open = dev_open,
   .read = dev_read,
   .write = dev_write,
   .release = dev_release,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
    .ioctl = dev_ioctl
#else
    .unlocked_ioctl = dev_ioctl
#endif
};

///< outputs array
static unsigned int gpio[MAX_DEV_CNT]={66,70};
module_param_array(gpio,int,&count,0660);

#define NSD16 62500000

static int gpio_init(char* name, int n) {
    int rc = gpio_request(gpio[n], name);
    if( rc < 0 ) {
        printk(KERN_ERR "gener: gpio_request for pin %u (%s) failed with error %d\n", gpio[n], name, rc );
        return rc;
    } else {
        rc = gpio_direction_output(gpio[n],0);
        if( rc < 0 ) {
            printk(KERN_ERR "gener: gpio_out_direction_request for pin %u (%s) failed with error %d\n", gpio[n], name, rc );
            return rc;
        }
    }
    return rc;
}

///< Frequency meter structure
struct hmeter {
     struct hrtimer timer;   ///< Frequency timers
     ktime_t kt_periode[2];  ///< Period and pulse holders
     unsigned char per;      ///< Step
     unsigned char index;    ///< GPIO index
};

static struct hmeter meter[MAX_DEV_CNT];

///< Timer handler
static enum hrtimer_restart timer_function(struct hrtimer *timerp)
{
    gpio_set_value(gpio[((struct hmeter*)timerp)->index],((struct hmeter*)timerp)->per);
#ifdef _DEBUG
    printk(KERN_INFO "gener: timer index %d\n",((struct hmeter*)timerp)->index);
#endif
    hrtimer_forward_now(timerp, ((struct hmeter*)timerp)->kt_periode[((struct hmeter*)timerp)->per]);
    ((struct hmeter*)timerp)->per = (((struct hmeter*)timerp)->per+1)&0x01;
    return HRTIMER_RESTART;
}

static void gpio_exit(unsigned int num) {
    gpio_free(gpio[num]);
}

static int __init dev_init(void)
{
    int i;
    char str[50];

    printk(KERN_INFO "gener module: init\n");

    /* Try to dynamically allocate a major number for the device -- more difficult but worth it */
    majorNumber = register_chrdev(0, DEVICE_NAME , &fops);
    if (majorNumber<0){
        printk(KERN_ALERT "gener: failed to register a major number\n");
        return majorNumber;
    }
    printk(KERN_INFO "gener: registered correctly with major number %d\n", majorNumber);

    /* Register the device class */
    genClass = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(genClass)){                ///< Check for error and clean up if there is
        printk(KERN_ALERT "Failed to register device class\n");
        return PTR_ERR(genClass);          ///< Correct way to return an error on a pointer
    }
    printk(KERN_INFO "gener: device class registered correctly\n");

    /* Register the device driver */
    for (i=0; i<count; i++) {
        meter[i].index=i;
        meter[i].per=0;
        sprintf(str,"%s%d",DEVICE_NAME,gpio[i]);
        genDevice[i] = device_create(genClass, NULL, MKDEV(majorNumber, i), NULL,/* DEVICE_NAME*/str);
        if (IS_ERR(genDevice)){
            /* Clean up if there is an error */
            class_destroy(genClass);
            /* Repeated code but the alternative is goto statements */
            unregister_chrdev(majorNumber, str);
            printk(KERN_ALERT "Failed to create the device\n");
            return PTR_ERR(genDevice);
        } else printk(KERN_INFO "gener: %s created correctly\n",str);
    }
    /* Made it! device was initialized */
    printk(KERN_INFO "gener: device class created correctly\n");
    return 0;
}

static void __exit dev_exit(void)
{
    int i;
    char str[50];
    printk(KERN_INFO "gener: gpio exit\n" );

    /* exit device */
    printk(KERN_INFO "gener: exit\n" );
    for (i=0; i<count; i++) {
        sprintf(str,"%s%d",DEVICE_NAME,gpio[i]);
        /* remove the device */
        device_destroy(genClass, MKDEV(majorNumber, i));
        /* unregister the major number */
        unregister_chrdev(majorNumber, str); 
        if (started[i]) {
            hrtimer_cancel(&meter[i].timer);
            gpio_exit(i);
        }
    }
    class_unregister(genClass);                              ///< unregister the device class
    class_destroy(genClass);                                 ///< remove the device class
    printk(KERN_INFO "gener: device unregistered\n");
}

/* device functions */

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep){
#ifdef _DEBUG
    int minor = MINOR(filep->f_path.dentry->d_inode->i_rdev);
    printk(KERN_INFO "gener: dev%u device successfully opened.\n",gpio[minor]);
#endif
    return 0;
}

/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case is uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset if required
 */
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){

    int error_count = 0;

    int minor = MINOR(filep->f_path.dentry->d_inode->i_rdev);

    size_of_message=sprintf(message,"gen%u period:pulse %llu:%llu\n",gpio[minor],meter[minor].kt_periode[0]+meter[minor].kt_periode[1],meter[minor].kt_periode[1]);

    if (*offset>=size_of_message) return 0;

   /* copy_to_user has the format ( * to, *from, size) and returns 0 on success */
   error_count = copy_to_user(buffer, message, size_of_message);

   if (error_count==0){            // if true then have success
#ifdef _DEBUG
        printk(KERN_INFO "gener: sent %d characters to the user\n", size_of_message);
#endif
        *offset+=size_of_message;
        error_count=size_of_message;
        size_of_message=0;
        return error_count;
   }
   else {
        printk(KERN_ERR "gener: Failed to send %d characters to the user\n", error_count);
        return -EFAULT;              // Failed -- return a bad address message (i.e. -14)
   }
}

/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
    int error_count = 0;
    unsigned long long int nsec[2],p[2],s[2],ns[2];

    int minor = MINOR(filep->f_path.dentry->d_inode->i_rdev);

    error_count = copy_from_user(message, buffer, len);
    message[len]=0;
    size_of_message = strlen(message);                 // store the length of the stored message
#ifdef _DEBUG
    printk(KERN_INFO "gen%u: Received %u characters from the user: %s\n",gpio[minor], len, message);
#endif
    sscanf(message,"%llu:%llu",&nsec[0],&nsec[1]);
    p[0]=(nsec[0]>nsec[1])?nsec[0]-nsec[1]:nsec[1]-nsec[0];
    p[1]=(nsec[0]>nsec[1])?nsec[1]:nsec[0];
    if (nsec[0] == 0) {
        if (started[minor]) hrtimer_cancel(&meter[minor].timer);
        started[minor] = 0;
        gpio_exit(minor);
    }
    else {
#ifdef _DEBUG
        printk(KERN_INFO "gen%u period:pulse %llu:%llu ns\n",gpio[minor], nsec[0],nsec[1]);
#endif
        s[0]=div64_u64_rem(p[0],SEC_NS,&ns[0]);
        s[1]=div64_u64_rem(p[1],SEC_NS,&ns[1]);
        meter[minor].kt_periode[0] = ktime_set(s[0], ns[0]); //seconds,nanoseconds
        meter[minor].kt_periode[1] = ktime_set(s[1], ns[1]); //seconds,nanoseconds
        if (!started[minor]) {
            gpio_init("generator",minor);
#ifdef _DEBUG
            printk(KERN_INFO "gen%u timer start init period:pulse %lu:%lu ns\n",minor,nsec[0],nsec[1]);
#endif
            hrtimer_init (&meter[minor].timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
            meter[minor].timer.function = timer_function;
            hrtimer_start(&meter[minor].timer, meter[minor].kt_periode[0], HRTIMER_MODE_ABS);
            started[minor]=1;
#ifdef _DEBUG
            printk(KERN_INFO "gen%u timer started period:pulse %lu:%lu ns\n",minor,nsec[0],nsec[1]);
#endif
        }
    }
    return len;
}

/* ioctl */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int dev_ioctl(struct inode *i, struct file *filep, unsigned int cmd, unsigned long arg)
#else
static long dev_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
#endif
{
    int minor = MINOR(filep->f_path.dentry->d_inode->i_rdev);
    gener_arg_t _arg;
    switch (cmd) {
        case GENER_GET_VARIABLES:
            _arg.nsec[0]=meter[minor].kt_periode[0];
            _arg.nsec[1]=meter[minor].kt_periode[1];
            if (copy_to_user((gener_arg_t *)arg, &_arg, sizeof(gener_arg_t)))
            {
                return -EACCES;
            }
            break;
        case GENER_CLR_VARIABLES:
            meter[minor].kt_periode[0]=0;
            meter[minor].kt_periode[1]=0;
            if (started[minor]) hrtimer_cancel(&meter[minor].timer);
            started[minor] = 0;
            gpio_exit(minor);
            break;
        case GENER_SET_VARIABLES:
            if (copy_from_user(&_arg, (gener_arg_t *)arg, sizeof(gener_arg_t)))
            {
                return -EACCES;
            }
            meter[minor].kt_periode[0]=_arg.nsec[0];
            meter[minor].kt_periode[1]=_arg.nsec[1];
            break;
        default:
            return -EINVAL;
    }
    return 0;
}


/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep){
    int minor = MINOR(filep->f_path.dentry->d_inode->i_rdev);
    printk(KERN_INFO "gener: dev%u device successfully closed\n",gpio[minor]);
    return 0;
}

module_init(dev_init);
module_exit(dev_exit);

/*
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("A.V.");
MODULE_DESCRIPTION("Generator Driver");
*/