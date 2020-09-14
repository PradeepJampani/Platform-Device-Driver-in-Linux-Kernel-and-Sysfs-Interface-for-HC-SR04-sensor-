#include "IOCTL.h"
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/timekeeping.h>
#include <linux/math64.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#define DEVICE_NAME1 "HCSR0"
#define DEVICE_NAME2 "HCSR1"
#define MAX_HCSR_DEV 2
#define DRIVER_NAME "PLT_HCSR"
#define CLASS_NAME "HCSR"

int check_num_devices=0;
static struct device *sys_device;
static struct class *sys_class;

static int check_flag = 1;


static const struct platform_device_id P_id_table[] = {
         { DEVICE_NAME1, 0 },
         { DEVICE_NAME2, 0 },
};

/* ping configurations and Parameters buffer*/
struct pin_mux_buf
{
    int trigger_pin;
    int echo_pin;
    };

struct mode_buf
{
    int n_samples;
    int period;
    int times_to_measure;
};

int stop=0;
typedef struct {
	__u64 TSC;
}data, *Pdata;
struct buf {
	Pdata ptr;
	int   write_p;
	int   read_p;
	unsigned long count;
	unsigned long loss;
};
static int base_minor_num;

typedef struct HCSRdevice {

		char 			*name;
		int			dev_no;
		struct platform_device 	plf_dev;

}*Pchip_hcsrdevice;
static LIST_HEAD(device_list);


struct hcsr04_dev
{
    	Pchip_hcsrdevice phcsr_device;

	struct miscdevice miscdev;
	char *dev_name;
	int dev_busy;
	struct pin_mux_buf cfg;
	struct mode_buf params;
	struct pin_mux_buf l_pin;
	int device_trigger[4];
	int device_echo[4];
	unsigned int irq;
    struct buf *pring;
    int write_c;
	struct semaphore distance_sem;
	struct semaphore trigger_sem;
		struct semaphore read_sem;
        int distance;
		int measurement_state;      //To check the status of measurement in read
    spinlock_t m_Lock;
    int state;
	struct task_struct *distance_measure_thread;
		struct list_head device_entry;
    int temp;
	struct task_struct *trig_thread;
	__u64 dist1;
	__u64 dist2;
};


irq_handler_t gpio_irq_handler(unsigned int irq, void *data)
{

    struct hcsr04_dev *hcsr = (struct hcsr04_dev *)data;

    if (irq_get_trigger_type(irq) == IRQF_TRIGGER_RISING)
    {

        hcsr->dist1 = ktime_get_ns();

        irq_set_irq_type(irq, IRQF_TRIGGER_FALLING);
    }
    else if (irq_get_trigger_type(irq) == IRQF_TRIGGER_FALLING)
    {
        hcsr->dist2 = ktime_get_ns();

        up(&(hcsr->distance_sem));

        irq_set_irq_type(irq, IRQF_TRIGGER_RISING);
    }
    return (irq_handler_t)IRQ_HANDLED;
}

int l_gpio[4],v_val[4];
void pin_setup(int pin){
int i;

    if(pin==0)
    {
        int pins_gpio[4] = {11, 32, -1, -1};
        int vals[4] = {0, 0, -1, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==1)
       {
        int pins_gpio[4] = {12, 28, 45, -1};
        int vals[4] = {0, 0, 0, -1};
       for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==2)
    {
        int pins_gpio[4] = {13, 34, 77, -1};
        int vals[4] = {0, 0, 0, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==3)
    {
        int pins_gpio[4] = {14, 16, 76, 64};
        int vals[4] = {0, 0, 0, 0};
       for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }

    }
   if(pin==4)
    {
        int pins_gpio[4] = {6, 36, -1, -1};
        int vals[4] = {0, 0, -1, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==5)
    {
        int pins_gpio[4] = {0, 18, 66, -1};
        int vals[4] = {0, 0, 0, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==6)
    {
        int pins_gpio[4] = {1, 20, 68, -1};
        int vals[4] = {0, 0, 0, -1};
       for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==7)
    {
        int pins_gpio[4] = {38, -1, -1, -1};
        int vals[4] = {0, -1, -1, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==8)
    {
        int pins_gpio[4] = {40, -1, -1, -1};
        int vals[4] = {0, -1, -1, -1};
       for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==9)
    {
        int pins_gpio[4] = {4, 22, 70, -1};
        int vals[4] = {0, 0, 0, -1};
    for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==10)
    {
        int pins_gpio[4] = {10, 26, 74, -1};
        int vals[4] = {0, 0, 0, -1};
       for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
   if(pin==11)
    {
        int pins_gpio[4] = {5, 24, 44, 72};
        int vals[4] = {0, 0, 0, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==12)
    {
        int pins_gpio[4] = {15, 42, -1, -1};
        int vals[4] = {0, 0, -1, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    if(pin==13)
    {
        int pins_gpio[4] = {7, 30, 46, -1};
        int vals[4] = {0, 0, 0, -1};
        for(i=0;i<4;i++){
        l_gpio[i]=pins_gpio[i];
        v_val[i]=vals[i];
        }
    }
    }

int pin_init(int pin, struct hcsr04_dev **hcsr, int isEchopin)
{
    int rc = 0, i = 0, irq = -1;
    pin_setup(pin);

    for (i = 0; i < 4; i++)
    {
        if (isEchopin)
            (*hcsr)->device_echo[i] = l_gpio[i];
        else
            (*hcsr)->device_trigger[i] = l_gpio[i];

        if (l_gpio[i] < 0)
            continue;

        gpio_request(l_gpio[i], NULL);

        if (i >=2)
            gpio_set_value_cansleep(l_gpio[i], v_val[i]);

        else
        {
                 if (isEchopin)
                {
                rc = gpio_direction_input(l_gpio[i]);

                if (i == 0)
                {
                    irq = gpio_to_irq(l_gpio[i]);
                    rc = request_irq(irq, (irq_handler_t)gpio_irq_handler, IRQF_TRIGGER_RISING, NULL, (void *)(*hcsr));
                    (*hcsr)->irq = irq;
                }
                    }
                     else  gpio_direction_output(l_gpio[i], v_val[i]);
               }
               }

    if (isEchopin)
        (*hcsr)->l_pin.echo_pin = l_gpio[0];
    else
        (*hcsr)->l_pin.trigger_pin = l_gpio[0];

    return 0;

}

int n_sens = 2;
module_param(n_sens, int, 0000);

typedef struct hcsr04_dev *hcsr_devp;
typedef struct hcsr04_dev shcsr_devp;


void one_shot(struct hcsr04_dev *hcsr ){
gpio_set_value_cansleep(hcsr->device_trigger[0], 1);
                    udelay(4);
                                gpio_set_value_cansleep(hcsr->device_trigger[0], 0);

}


static int distance_measure_thread_fn(void *data)
{
            unsigned long int flags;


    int count = 0, i = 0;
    struct hcsr04_dev *hcsr = (struct hcsr04_dev *)data;
    const int n_samples = hcsr->params.n_samples;
    const int delay_msecs = hcsr->params.period;
   // const int n_write=hcsr->write_c;
    __u64 distance = 0, max = 0, min = __UINT64_MAX__;
    __u64 total_time[n_samples + 2];

    while(stop==0){
        one_shot(hcsr);

        count = 0, i = 0,distance = 0, max = 0, min = __UINT64_MAX__;
    while (count < (n_samples + 2))
    {

        down_interruptible(&(hcsr->distance_sem));

        total_time[count] = hcsr->dist2 - hcsr->dist1;

        if (total_time[count] < min)
            min = total_time[count];
        if (total_time[count] > max)
            max = total_time[count];



        count++;


        if (count < (n_samples + 2))
        {
        up(&(hcsr->trigger_sem));
            msleep(delay_msecs);


        }
        }



    for (i = 0; i < (n_samples + 2); i++)
    {
        if (total_time[i] == min || total_time[i] == max)
            continue;


        distance = distance + total_time[i];
    }

    distance = div_u64(distance * 340, (__u64)(10000000 * n_samples * 2));
        spin_lock_irqsave(&hcsr->m_Lock, flags );
			       	hcsr->pring->ptr[hcsr->pring->read_p].TSC = distance;
			       	 hcsr->distance=distance;

                                   hcsr->pring->read_p = (hcsr->pring->read_p + 1) % 5;


        udelay(40);

       }


        return 0;
}

static int trig_thread_fn(void *data)
{
    int count = 0;
    struct hcsr04_dev *hcsr = (struct hcsr04_dev *)data;
    const int n_samples = hcsr->params.n_samples;



   while(stop==0){
    count=0;
    while (count < (n_samples + 2))
    {

       down_interruptible(&hcsr->trigger_sem);

            gpio_set_value_cansleep(hcsr->device_trigger[0], 0);

        gpio_set_value_cansleep(hcsr->device_trigger[0], 1);

        udelay(11);

        gpio_set_value_cansleep(hcsr->device_trigger[0], 0);


        count++;

    }

   }

    return 0;

    }



int hcsr04_driver_open(struct inode *inode, struct file *file)
{
    const struct file_operations *new_fops = NULL;
	int dev_minor= iminor(inode);
    hcsr_devp hcsr_p;
   // int i = 0, minor;

    dev_minor = iminor(inode);


    printk("minor number in open = %d\n", dev_minor);

	list_for_each_entry(hcsr_p, &device_list, device_entry)
	{
		if(hcsr_p->miscdev.minor == dev_minor)
		{
			new_fops = fops_get(hcsr_p->miscdev.fops);
			break;
		}
	}
	check_num_devices++;
		printk("minor number is %d\n", dev_minor);
    	file->private_data = hcsr_p;



    return 0;
}


int hcsr04_driver_release(struct inode *inode, struct file *file)
{
        struct hcsr04_dev *hcsr;
	      hcsr = file->private_data;
    






    return 0;
}



/* Read function*/
ssize_t hcsr04_driver_read(struct file *file, char *buf, size_t check_count, loff_t *ppos)
{
                unsigned long int flags;

    struct hcsr04_dev *hcsr;
    int var;
    hcsr = file->private_data; //opening the file descriptor
            //check_count=(check_count+1);
            //atomicaly getting the write_p of the ring buffer
            spin_lock_irqsave(&hcsr->m_Lock, flags );
            var= hcsr->pring->write_p;
            spin_unlock_irqrestore(&hcsr->m_Lock, flags);
            //Checking if the buffer has recored a measurement , if not block and start a new measurement if there is no on-going measurement
            while(1){
            if((hcsr->pring->ptr[var].TSC)==-1){
                if(hcsr->measurement_state==0){
                 hcsr->measurement_state=1;
                 hcsr->params.times_to_measure=1;//starting the measurement only once for exceptions like this

                 //Start the measurement by intiating the distance and trigger thread
                 hcsr->distance_measure_thread = kthread_run(distance_measure_thread_fn, (void *)hcsr, "%s_distance_measure_thread", hcsr->dev_name);
                 hcsr->trig_thread = kthread_run(trig_thread_fn, (void *)hcsr, "%s_trig_thread", hcsr->dev_name);
                 udelay(40);
                }
                //If there is an ongoing measurement wait untill its done
                else if (hcsr->measurement_state==1){
                            down_interruptible(&(hcsr->read_sem));
                            }

            }
            else break;

            }
            //Sending the data to the user
            if (copy_to_user((Pdata)buf, &(hcsr->pring->ptr[var]) , sizeof(data)))
                {
                printk("unable to copy to  the user");
                return -EFAULT;
                        }
                //updating the write_p of the ring buffer
                hcsr->pring->write_p = (hcsr->pring->write_p + 1) % 5;



    return 0;
}



/* The write function to start the write operation */
ssize_t hcsr04_driver_write(struct file *file, const char *buf, size_t check_count, loff_t *ppos)
{
    struct hcsr04_dev *hcsr;
    int user_int, rc = 0,i;
    hcsr = file->private_data; //opening the private file

    //If there is any on-going measurement, return invalid
    if (hcsr->dev_busy)
        return -EINVAL;
    //get the mode from user
    rc = copy_from_user(&user_int, buf, check_count);
    if (rc)
        return -EINVAL;

    //If the value is an non-zero value stop the operation and clear the ring buffer
    if (user_int)
    {
      stop=1;
      //clearing the ring buffer
     hcsr->pring->read_p=0;
     for( i=0;i<5;i++){
      hcsr->pring->ptr[hcsr->pring->read_p].TSC=-1;
        hcsr->pring->read_p=(hcsr->pring->read_p+1)%5;
    }
    /* IF the mode from user is non-zero value start the measurement*/
    }
    else
    {
        stop=0;
        hcsr->measurement_state=1; //flag to store the measurment state
       hcsr->dev_busy = 1;      //flag to store the sttaus of device

        /*Starting the distance and trigger threads*/
        hcsr->distance_measure_thread = kthread_run(distance_measure_thread_fn, (void *)hcsr, "%s_distance_measure_thread", hcsr->dev_name);
        hcsr->trig_thread = kthread_run(trig_thread_fn, (void *)hcsr, "%s_trig_thread", hcsr->dev_name);
        }


    return 0;
}



long hcsr04_driver_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int rc = 0;
    struct hcsr04_dev *hcsr;
    struct pin_mux_buf cfg;
    struct mode_buf params;
    hcsr = file->private_data;

    switch (cmd)
    {
    case CONFIG_PINS:
    {
        rc = copy_from_user(&cfg, (struct pin_mux_buf *)arg, sizeof(struct pin_mux_buf));

        hcsr->cfg = cfg;

        pin_init(hcsr->cfg.trigger_pin, &hcsr, 0);
         pin_init(hcsr->cfg.echo_pin, &hcsr, 1);

        break;
    }
    case SET_PARAMETERS:
    {
        rc = copy_from_user(&params, (struct mode_buf *)arg, sizeof(struct mode_buf));

        hcsr->params = params;
        break;
    }

    }

    return 0;
}

static struct file_operations hcsr04_fops = {
    .owner = THIS_MODULE,
    .open = hcsr04_driver_open,
    .read = hcsr04_driver_read,
    .write = hcsr04_driver_write,
    .unlocked_ioctl = hcsr04_driver_ioctl,
    .release = hcsr04_driver_release,
};

hcsr_devp init_device(void* Pchip_handle)
{
    int ret=0;
    int  j = 0;
    Pchip_hcsrdevice pdevice = (Pchip_hcsrdevice)Pchip_handle;
	hcsr_devp hcsr_p;
       hcsr_p = kmalloc(sizeof(shcsr_devp), GFP_KERNEL);
	printk("device kmalloc buf\n");
	if (!hcsr_p) {
		printk("Bad Kmalloc\n");

	}
printk("malloc hcsr_p\n");

	memset(hcsr_p, 0, sizeof( shcsr_devp));

	hcsr_p->phcsr_device = pdevice;

	if (pdevice->dev_no ==1)
	{
		hcsr_p->dev_name = DEVICE_NAME1;
		hcsr_p->miscdev.name = DEVICE_NAME1;


	}

	if (pdevice->dev_no ==2)
	{
		hcsr_p->dev_name = DEVICE_NAME2;
		hcsr_p->miscdev.name = DEVICE_NAME2;

	}

	hcsr_p->miscdev.minor = ++base_minor_num;
	hcsr_p->miscdev.fops = &hcsr04_fops;
	ret = misc_register(&hcsr_p->miscdev);

	printk("inside probe  minor = %d", hcsr_p->miscdev.minor);

        hcsr_p->dev_busy = 0;
        for (j = 0; j < 4; j++)
        {
            hcsr_p->device_echo[j] = -1;
            hcsr_p->device_trigger[j] = -1;
        }
       hcsr_p->irq = -1;
        sema_init(&(hcsr_p->distance_sem), 0);
        sema_init(&(hcsr_p->trigger_sem), 1);
                sema_init(&(hcsr_p->read_sem), 0);

        		hcsr_p->pring = kmalloc(sizeof(struct buf), GFP_KERNEL);

        if (!hcsr_p->pring)
		{
			printk("Bad Kmalloc\n");

		}

		if (!(hcsr_p->pring->ptr = kmalloc(sizeof(data)* 5, GFP_KERNEL)))
		{
			printk("Bad Kmalloc\n");

		}
		hcsr_p->pring->write_p = 0;
		hcsr_p->pring->read_p = 0;


    INIT_LIST_HEAD(&hcsr_p->device_entry) ;
    list_add(&hcsr_p->device_entry, &device_list );


    return hcsr_p;
}





static ssize_t trigger_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	hcsr_devp hcsr = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n",hcsr->cfg.trigger_pin);
}



static ssize_t trigger_pin_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count)
{
	hcsr_devp hcsr = dev_get_drvdata(dev);

        sscanf(buf, "%d", &hcsr->cfg.trigger_pin);
        pin_init(hcsr->cfg.trigger_pin, &hcsr, 0);
	return count;


}

static ssize_t echo_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{


	hcsr_devp hcsr = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n",hcsr->cfg.echo_pin);
}

static ssize_t echo_pin_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count)
{
    	hcsr_devp hcsr = dev_get_drvdata(dev);

        sscanf(buf, "%d", &hcsr->cfg.echo_pin);
            pin_init(hcsr->cfg.echo_pin, &hcsr, 1);

                    one_shot(hcsr);

	return count;


}

static ssize_t sampling_period_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	hcsr_devp hcsr = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n",hcsr->params.period);
}
static ssize_t sampling_period_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count)
{
    hcsr_devp hcsr = dev_get_drvdata(dev);

        sscanf(buf, "%d", &hcsr->params.period);
return count;

}


static ssize_t enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{

        return snprintf(buf, PAGE_SIZE, "%d\n",2);
}


static ssize_t enable_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count)
{


    	hcsr_devp hcsr = dev_get_drvdata(dev);
    sscanf(buf, "%d", &hcsr->temp);
    if(hcsr->temp==1){

       stop=0;
    hcsr->distance_measure_thread = kthread_run(distance_measure_thread_fn, (void *)hcsr, "%s_distance_measure_thread", hcsr->dev_name);

        hcsr->trig_thread = kthread_run(trig_thread_fn, (void *)hcsr, "%s_trig_thread", hcsr->dev_name);
        }

    if(hcsr->temp==0){
    stop=1;

    }


return count;

}


static ssize_t number_samples_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	hcsr_devp hcsr = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n",hcsr->params.n_samples);
}
static ssize_t number_samples_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count)
{
    hcsr_devp hcsr = dev_get_drvdata(dev);

        sscanf(buf, "%d", &hcsr->params.n_samples);
        return count;

}
static ssize_t distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	hcsr_devp hcsr = dev_get_drvdata(dev);
        return snprintf(buf, PAGE_SIZE, "%d\n",hcsr->distance);
}
static ssize_t distance_store(struct device *dev, struct device_attribute *attr,const char *buf,size_t count)
{
return count;

}



static DEVICE_ATTR(trigger_pin, S_IRUSR | S_IWUSR ,trigger_pin_show,trigger_pin_store);
static DEVICE_ATTR(echo_pin, S_IRUSR | S_IWUSR ,echo_pin_show,echo_pin_store);
static DEVICE_ATTR(number_samples, S_IRUSR | S_IWUSR ,number_samples_show,number_samples_store);
static DEVICE_ATTR(sampling_period, S_IRUSR | S_IWUSR ,sampling_period_show,sampling_period_store);
static DEVICE_ATTR(enable, S_IRUSR | S_IWUSR ,enable_show,enable_store);
static DEVICE_ATTR(distance, S_IRUSR | S_IWUSR ,distance_show,distance_store);






static int P_driver_probe(struct platform_device *dev_found)
{
    	hcsr_devp hcsr_p;
int rval;
Pchip_hcsrdevice pdevice;

	pdevice = container_of(dev_found, struct HCSRdevice, plf_dev);

	printk(KERN_ALERT "Found the device -- %s  %d \n", pdevice->name, pdevice->dev_no);
        if(!(hcsr_p =init_device((void*)pdevice)))
{
	printk("device initialisation failed\n");
}

if(check_flag == 1)
{
 sys_class = class_create(THIS_MODULE, CLASS_NAME);
        if (IS_ERR(sys_class)) {
                printk( " cant create class %s\n", CLASS_NAME);
               // goto class_err;

        }
check_flag =0;
}

sys_device = device_create(sys_class, hcsr_p->miscdev.this_device, MKDEV(0,0), hcsr_p, hcsr_p->dev_name);
        if (IS_ERR(sys_device)) {

                printk( " cant create device %s\n", hcsr_p->dev_name);
               // goto device_err;
        }

rval= device_create_file(sys_device, &dev_attr_trigger_pin);
        if (rval < 0) {
                printk(" cant create device attribute %s %s\n",
                       hcsr_p->dev_name, dev_attr_trigger_pin.attr.name);
        }
rval= device_create_file(sys_device, &dev_attr_echo_pin);
        if (rval < 0) {
                printk(" cant create device attribute %s %s\n",
                       hcsr_p->dev_name, dev_attr_echo_pin.attr.name);
        }

rval= device_create_file(sys_device, &dev_attr_number_samples);
        if (rval < 0) {
                printk(" cant create device attribute %s %s\n",
                       hcsr_p->dev_name, dev_attr_number_samples.attr.name);
        }

rval= device_create_file(sys_device, &dev_attr_sampling_period);
        if (rval < 0) {
                printk(" cant create device attribute %s %s\n",
                       hcsr_p->dev_name, dev_attr_sampling_period.attr.name);
        }
rval= device_create_file(sys_device, &dev_attr_enable);
        if (rval < 0) {
                printk(" cant create device attribute %s %s\n",
                       hcsr_p->dev_name, dev_attr_enable.attr.name);
        }
rval= device_create_file(sys_device, &dev_attr_distance);
        if (rval < 0) {
                printk(" cant create device attribute %s %s\n",
                       hcsr_p->dev_name, dev_attr_distance.attr.name);
        }
return 0;
/*
device_err:
        device_destroy(sys_class, gko_dev);
class_err:
        class_unregister(sys_class);
        class_destroy(sys_class);*/
return -EFAULT;



}


static int P_driver_remove(struct platform_device *pdev)
{
    hcsr_devp  hcsr_p;
list_for_each_entry(hcsr_p, &device_list, device_entry)
	{
	   	printk("removing a device\n");
		if(hcsr_p->dev_name == pdev->name)
		{
		    --check_num_devices;
		list_del(&hcsr_p->device_entry);
	   device_destroy(sys_class, hcsr_p->miscdev.minor);
		kfree(hcsr_p->pring->ptr);
   	    
	    kfree(hcsr_p->pring);
   	    
	    misc_deregister(&hcsr_p->miscdev);
   	  
	    kfree(hcsr_p);
			break;
		}
	}

check_flag=1;




if (check_num_devices == 0)
{
class_unregister(sys_class);
        class_destroy(sys_class);
}

	return 0;
};



static struct platform_driver P_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= P_driver_probe,
	.remove		= P_driver_remove,
	.id_table	= P_id_table,
};


module_platform_driver(P_driver);


MODULE_LICENSE("GPL v2");
