#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/stat.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/mutex.h>
#include <linux/timer.h>


#include <asm/uaccess.h>
#include <asm/delay.h>


#define	RASPBERRYPI2
#undef RASPBERRYPI1


MODULE_AUTHOR("RT Corporation");
MODULE_LICENSE("GPL");
MODULE_VERSION("1:1.0") ;
MODULE_DESCRIPTION("Raspberry pi Car device driver");


#define NUM_DEV_MOTORRAWR	1
#define NUM_DEV_MOTORRAWL	1
#define NUM_DEV_MOTOREN		1
#define NUM_DEV_MOTOR		1


#define NUM_DEV_TOTAL ( NUM_DEV_MOTORRAWR + NUM_DEV_MOTORRAWL + NUM_DEV_MOTOREN )

#define DEVNAME_MOTORRAWR	"rtmotor_raw_r"
#define DEVNAME_MOTORRAWL	"rtmotor_raw_l"
#define DEVNAME_MOTOREN		"rtmotoren"
#define DEVNAME_MOTOR		"rtmotor"

#define	DEV_MAJOR 0
#define	DEV_MINOR 0

static int _major_motorrawr = DEV_MAJOR;
static int _minor_motorrawr = DEV_MINOR;

static int _major_motorrawl = DEV_MAJOR;
static int _minor_motorrawl = DEV_MINOR;

static int _major_motoren = DEV_MAJOR;
static int _minor_motoren = DEV_MINOR;

static int _major_motor = DEV_MAJOR;
static int _minor_motor = DEV_MINOR;

static struct cdev *cdev_array = NULL;
static struct class *class_motorrawr = NULL;
static struct class *class_motorrawl = NULL;
static struct class *class_motoren = NULL;
static struct class *class_motor = NULL;

static volatile void __iomem *pwm_base;
static volatile void __iomem *clk_base;
static volatile uint32_t *gpio_base;

static volatile int cdev_index = 0;
static volatile int open_counter = 0;

/*
 * Mutex Lock
 */
static struct mutex lock;

#define R_AD_CH		3
#define L_AD_CH		0
#define RF_AD_CH	2
#define LF_AD_CH	1

#define MOTCLK_L_BASE	12
#define MOTDIR_L_BASE	16

#define MOTCLK_R_BASE	13
#define MOTDIR_R_BASE	6

#define MOTEN_BASE	5

#define PWM_ORG0_BASE	40
#define PWM_ORG1_BASE	45


/* レジスタアドレス */
//base addr
#ifdef RASPBERRYPI1
	#define RPI_REG_BASE	0x20000000
#else
	#define RPI_REG_BASE	0x3f000000
#endif

//gpio addr
#define RPI_GPIO_OFFSET	0x200000
#define RPI_GPIO_SIZE	0xC0
#define RPI_GPIO_BASE	(RPI_REG_BASE + RPI_GPIO_OFFSET)
#define	REG_GPIO_NAME	"RPi mouse GPIO"

//pwm addr
#define RPI_PWM_OFFSET	0x20C000
#define RPI_PWM_SIZE	0xC0
#define RPI_PWM_BASE	(RPI_REG_BASE + RPI_PWM_OFFSET)
#define	REG_PWM_NAME	"RPi mouse PWM"

//clock addr
#define RPI_CLK_OFFSET	0x101000
#define RPI_CLK_SIZE	0x100
#define RPI_CLK_BASE	(RPI_REG_BASE + RPI_CLK_OFFSET)
#define	REG_CLK_NAME	"RPi mouse CLK"

//clock offsets
#define CLK_PWM_INDEX		0xa0
#define CLK_PWMDIV_INDEX	0xa4

/* GPIO PUPD select */
#define GPIO_PULLNONE 0x0
#define GPIO_PULLDOWN 0x1
#define GPIO_PULLUP   0x2

/* GPIO Functions	*/
#define	RPI_GPF_INPUT	0x00
#define	RPI_GPF_OUTPUT	0x01
#define	RPI_GPF_ALT0	0x04
#define	RPI_GPF_ALT5	0x02

/* GPIOレジスタインデックス */
#define RPI_GPFSEL0_INDEX	0
#define RPI_GPFSEL1_INDEX	1
#define RPI_GPFSEL2_INDEX	2
#define RPI_GPFSEL3_INDEX	3

#define RPI_GPSET0_INDEX	7
#define RPI_GPCLR0_INDEX	10

//PWM インデックス
#define RPI_PWM_CTRL	0x0
#define RPI_PWM_STA		0x4
#define RPI_PWM_DMAC	0x8
#define RPI_PWM_RNG1	0x10
#define RPI_PWM_DAT1	0x14
#define RPI_PWM_FIF1	0x18
#define RPI_PWM_RNG2	0x20
#define RPI_PWM_DAT2	0x24

#define PWM_BASECLK	9600000

/* GPIO Mask */
#define RPI_GPIO_P1MASK	(uint32_t) ((0x01<<2) | (0x01<<3) | (0x01<<4) | \
									(0x01<<7) | (0x01<<8) | (0x01<<9) | \
									(0x01<<10)| (0x01<<11)| (0x01<<14)| \
									(0x01<<15)| (0x01<<17)| (0x01<<18)| \
									(0x01<<22)| (0x01<<23)| (0x01<<24)| \
									(0x01<<25)| (0x01<<27)\
								   )
#define RPI_GPIO_P2MASK (uint32_t)0xffffffff
static void set_motor_r_freq(int freq);
static void set_motor_l_freq(int freq);

#define MAX_BUFLEN 64
unsigned char sw_buf[ MAX_BUFLEN ];

#define MOTOR_MOTION	0

#if MOTOR_MOTION

typedef struct
{
	signed int r_hz;
	signed int l_hz;
	unsigned int time;
}t_motor_motion;

#define MAX_MOTORBUFLEN 16
static t_motor_motion motor_motion[MAX_MOTORBUFLEN];
static unsigned int motor_motion_head = 0, motor_motion_tail = 0;

/*

*/
static int motor_motion_push(int r_hz, int l_hz, int time)
{
	unsigned int next_tail = motor_motion_tail + 1;
	
	if(next_tail >= MAX_MOTORBUFLEN)
	{
		next_tail = 0;
	}
	
	if(next_tail == motor_motion_head)
	{
		return -1;
	}
	
	motor_motion[motor_motion_tail].r_hz = r_hz;
	motor_motion[motor_motion_tail].l_hz = l_hz;
	motor_motion[motor_motion_tail].time = time;
	
	motor_motion_tail = next_tail;
	
	return 0;
}

/*

*/
static int motor_motion_pop(t_motor_motion **ret)
{
	unsigned int next_head = motor_motion_head + 1;

	if(motor_motion_tail == motor_motion_head)
	{
		return -1;
	}
	
	if(next_head >= MAX_MOTORBUFLEN)
	{
		next_head = 0;
	}
	
	*ret = (motor_motion + motor_motion_head);
	
	motor_motion_head = next_head;
	
	return 0;
}
#endif



/*
 *    getPWMCount
 */
static int getPWMCount(int freq)
{
	if(freq < 1)
	{
		freq = 1;
	}
	else if(freq > 10000)
	{
		freq = 10000;
	}
	return  PWM_BASECLK / freq;
}


/*
    GPIO operation: set function 
 */
static int rpi_gpio_function_set(int pin, uint32_t func)
{
	int index = RPI_GPFSEL0_INDEX + pin / 10;
	uint32_t mask = ~(0x7 << ((pin % 10) * 3));

	gpio_base[index] = (gpio_base[index] & mask) | ((func & 0x7) << ((pin % 10) * 3));
	
	return 1;
}

/*
   GPIO operation: set mask and value

 */
static void rpi_gpio_set32( uint32_t mask, uint32_t val )
{
	gpio_base[RPI_GPSET0_INDEX] = val & mask;
}

/*
   GPIO operation: clear mask and value
 */
static void rpi_gpio_clear32( uint32_t mask, uint32_t val )
{
	gpio_base[RPI_GPCLR0_INDEX] = val & mask;
}

/*
  
 */
static void rpi_pwm_write32(uint32_t offset, uint32_t val)
{
	iowrite32(val, pwm_base + offset);
}

/*

 */
static void set_motor_l_freq(int freq)
{
	int dat;
	
	if(freq == 0)
	{
		rpi_gpio_function_set(MOTCLK_L_BASE, RPI_GPF_OUTPUT);
		return;
	}
	else
	{
		rpi_gpio_function_set(MOTCLK_L_BASE, RPI_GPF_ALT0);
	}
	
	if(freq > 0)
	{
		rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << MOTDIR_L_BASE);
	}
	else
	{
		rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << MOTDIR_L_BASE);
		freq = -freq;
	}
	
	dat = getPWMCount(freq);
	
	rpi_pwm_write32(RPI_PWM_RNG1, dat);
	rpi_pwm_write32(RPI_PWM_DAT1, dat >> 1);
	
	return;
}

/*

 */
static void set_motor_r_freq(int freq)
{
	int dat;
	
	if(freq == 0)
	{
		rpi_gpio_function_set(MOTCLK_R_BASE, RPI_GPF_OUTPUT);
		return;
	}
	else
	{
		rpi_gpio_function_set(MOTCLK_R_BASE, RPI_GPF_ALT0);
	}
	
	if(freq > 0)
	{
		rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << MOTDIR_R_BASE);
	}
	else
	{
		rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << MOTDIR_R_BASE);
		freq = -freq;
	}
	
	dat = getPWMCount(freq);
	
	rpi_pwm_write32(RPI_PWM_RNG2, dat);
	rpi_pwm_write32(RPI_PWM_DAT2, dat >> 1);
	
	return;
}
static int led_gpio_map(void)
{
	static int clk_status = 1;
	
	if(gpio_base == NULL)
	{
		gpio_base = ioremap_nocache(RPI_GPIO_BASE, RPI_GPIO_SIZE);
	}
	
	if(pwm_base == NULL)
	{
		pwm_base = ioremap_nocache(RPI_PWM_BASE, RPI_PWM_SIZE);
	}
	
	if(clk_base == NULL)
	{
		clk_base = ioremap_nocache(RPI_CLK_BASE, RPI_CLK_SIZE);
	}

	//kill
	if(clk_status == 1)
	{
		iowrite32(0x5a000000 | (1 << 5), clk_base + CLK_PWM_INDEX);
		udelay(1000);

		//clk set
		iowrite32(0x5a000000 | (2 << 12), clk_base + CLK_PWMDIV_INDEX);
		iowrite32(0x5a000011, clk_base + CLK_PWM_INDEX);

		udelay(1000);	//1mS wait
		
		clk_status = 0;
	}

	return 0;
}

/*
    Open device
 */
static int dev_open(struct inode *inode, struct file *filep)
{
	int retval;
	int *minor = (int *)kmalloc(sizeof(int),GFP_KERNEL);
	int major = MAJOR(inode->i_rdev);
	*minor = MINOR(inode->i_rdev);

	printk(KERN_INFO "open request major:%d minor: %d \n", major, *minor);
	
	filep->private_data = (void *)minor;

	retval = led_gpio_map();
	if( retval != 0 )
	{
		printk(KERN_ERR "Can not open led.\n" );
		return retval;
	}
	
	if(_major_motor == major)
	{
		printk(KERN_INFO"motor write\n" );
	}

	open_counter++;
	return 0;
}

/*
    Unmap GPIO addresses
 */
static int gpio_unmap(void)
{
	iounmap(gpio_base);
	iounmap(pwm_base);
	iounmap(clk_base);

	gpio_base = NULL;
	pwm_base = NULL;
	clk_base = NULL;
	return 0;
}

/*
    Close device
 */
static int dev_release(struct inode *inode, struct file *filep)
{
	kfree(filep->private_data);
	
	open_counter--;
	if(open_counter <= 0)
	{
		gpio_unmap();
	}
	return 0;
}


/*
    Parse Frequency 
 */
static int parseFreq(const char __user *buf, size_t count, int *ret)
{
	char cval;
	int error = 0, i = 0, tmp, bufcnt = 0, freq;
	size_t readcount = count;
	int sgn = 1;
	
	char *newbuf = kmalloc(sizeof(char)*count, GFP_KERNEL);

	while(readcount > 0)
	{
		if(copy_from_user(&cval, buf + i, sizeof(char)))
		{
			kfree(newbuf);
			return -EFAULT;
		}
		
		if(cval == '-')
		{
			if(bufcnt == 0)
			{
				sgn = -1;
			}
		}
		else if(cval < '0' || cval > '9')
		{
			newbuf[bufcnt] = 'e';
			error = 1;
		}
		else
		{
			newbuf[bufcnt] = cval;
		}
		
		i++;
		bufcnt++;
		readcount--;
		
		if(cval == '\n')
		{
			break;
		}
	}

	freq = 0;
	for(i = 0, tmp = 1; i < bufcnt; i++)
	{
		char c = newbuf[bufcnt - i - 1];
		
		if( c >= '0' && c <= '9')
		{
			freq += ( newbuf[bufcnt - i - 1]  -  '0' ) * tmp;
			tmp *= 10;
		}
	}
	
	*ret = sgn * freq; 
	
	kfree(newbuf);
	
	return bufcnt;
}

/*
   Paese motor command....

 */
static int parseMotorCmd(const char __user *buf, size_t count, int *ret)
{
	int r_motor_val, l_motor_val, time_val;
	char *newbuf = kmalloc(sizeof(char)*count, GFP_KERNEL);

	if(copy_from_user(newbuf, buf, sizeof(char) * count))
	{
		kfree(newbuf);
		return -EFAULT;
	}
	
	sscanf(newbuf,"%d%d%d\n",&l_motor_val, &r_motor_val, &time_val);
	
	kfree(newbuf);

	mutex_lock(&lock);	

	set_motor_l_freq(l_motor_val);
	set_motor_r_freq(r_motor_val);

	msleep_interruptible(time_val);
	
	set_motor_l_freq(0);
	set_motor_r_freq(0);
	
	mutex_unlock(&lock);

	return count;
}


/*
   Write function of /dev/rtmotor_raw_l
      Output frequency to the left motor
 */
static ssize_t rawmotor_l_write( struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	int freq, bufcnt;
	bufcnt = parseFreq(buf, count, &freq);
	
	set_motor_l_freq(freq);
	
	return bufcnt;
} 

/*
   Write function of /dev/rtmotor_raw_r
      Output frequency to the right motor
 */
static ssize_t rawmotor_r_write( struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	int freq, bufcnt;
	bufcnt = parseFreq(buf, count, &freq);
	
	set_motor_r_freq(freq);
	
	return bufcnt;
} 

/*
   Write function of /dev/rtmotoren

 */
static ssize_t motoren_write( struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	char cval;
	
	if(count>0)
	{
		if(copy_from_user(&cval, buf, sizeof(char)))
		{
			return -EFAULT;
		}
		
		switch(cval)
		{
			case '1':
				rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << MOTEN_BASE);
				break;
			case '0':
				rpi_gpio_clear32(RPI_GPIO_P2MASK, 1 << MOTEN_BASE);
				break;
		}
		return sizeof(char);
	}
	return 0;
} 

/*
   Write function of /dev/rtmotor

 */
static ssize_t motor_write( struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	int tmp;
	int bufcnt;
	bufcnt = parseMotorCmd(buf,count,&tmp);
	
	return bufcnt;
} 


/*
    define device operations
      /dev/rtmotor_raw_r
 */
static struct file_operations motorrawr_fops = {
		.open      = dev_open,
		.write      = rawmotor_r_write,
		.release   = dev_release,
};

/*
    define device operations
      /dev/rtmotor_raw_l
*/
static struct file_operations motorrawl_fops = {
		.open      = dev_open,
		.write      = rawmotor_l_write,
		.release   = dev_release,
};

/*
    define device operations
      /dev/rtmotoren
 */
static struct file_operations motoren_fops = {
		.open      = dev_open,
		.write      = motoren_write,
		.release   = dev_release,
};

/*
    define device operations
      /dev/rtmotor
 */
static struct file_operations motor_fops = {
		.open      = dev_open,
		.write      = motor_write,
		.release   = dev_release,
};


/*
   Register device drive and create device file
     /dev/rtmotor_raw_r0

 */
static int motorrawr_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;
	
	/* 空いているメジャー番号を使ってメジャー&
	   マイナー番号をカーネルに登録する */
	retval =  alloc_chrdev_region(
		&dev,			/* 結果を格納するdev_t構造体 */
		DEV_MINOR,		/* ベースマイナー番号 */
		NUM_DEV_MOTORRAWR,	/* デバイスの数 */
		DEVNAME_MOTORRAWR	/* デバイスドライバの名前 */
	);
	
	if( retval < 0 ) 
	{
		printk(KERN_ERR "alloc_chrdev_region failed.\n" );
		return retval;
	}
	_major_motorrawr = MAJOR(dev);
	
	/* デバイスクラスを作成する */
	class_motorrawr = class_create(THIS_MODULE,DEVNAME_MOTORRAWR);
	 devno = MKDEV(_major_motorrawr, _minor_motorrawr);

	/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
	cdev_init(&(cdev_array[cdev_index]), &motorrawr_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if( cdev_add( &(cdev_array[cdev_index]), devno, 1) < 0 )
	{
		/* 登録に失敗した */
		printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_motorrawr );
	}
	else
	{
		/* デバイスノードの作成 */
		device_create(
				class_motorrawr,
				NULL,
				devno,
				NULL,
				DEVNAME_MOTORRAWR"%u",
				_minor_motorrawr
		);
	}
	
	cdev_index++;

	return 0;
}

/*
   Register device drive and create device file
     /dev/rtmotor_raw_r0

*/
static int motorrawl_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;
	
	/* 空いているメジャー番号を使ってメジャー&
	   マイナー番号をカーネルに登録する */
	retval =  alloc_chrdev_region(
		&dev,			/* 結果を格納するdev_t構造体 */
		DEV_MINOR,		/* ベースマイナー番号 */
		NUM_DEV_MOTORRAWL,	/* デバイスの数 */
		DEVNAME_MOTORRAWL	/* デバイスドライバの名前 */
	);
	
	if( retval < 0 )
	{
		printk(KERN_ERR "alloc_chrdev_region failed.\n" );
		return retval;
	}
	_major_motorrawl = MAJOR(dev);
	
	/* デバイスクラスを作成する */
	class_motorrawl = class_create(THIS_MODULE,DEVNAME_MOTORRAWL);

	 devno = MKDEV(_major_motorrawl, _minor_motorrawl);
	/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
	cdev_init(&(cdev_array[cdev_index]), &motorrawl_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if( cdev_add( &(cdev_array[cdev_index]), devno, 1) < 0 )
	{
		/* 登録に失敗した */
		printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_motorrawl );
	}
	else
	{
		/* デバイスノードの作成 */
		device_create(
				class_motorrawl,
				NULL,
				devno,
				NULL,
				DEVNAME_MOTORRAWL"%u",
				_minor_motorrawl
		);
	}

	cdev_index++;

	return 0;
}

/*
   Register device drive and create device file
     /dev/rtmotoren0

*/
static int motoren_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;
		
	/* 空いているメジャー番号を使ってメジャー
 		&マイナー番号をカーネルに登録する */
	retval =  alloc_chrdev_region(
		&dev,			/* 結果を格納するdev_t構造体 */
		DEV_MINOR,		/* ベースマイナー番号 */
		NUM_DEV_MOTOREN,	/* デバイスの数 */
		DEVNAME_MOTOREN		/* デバイスドライバの名前 */
	);
	
	if( retval < 0 )
	{
		printk(KERN_ERR "alloc_chrdev_region failed.\n" );
		return retval;
	}
	_major_motoren = MAJOR(dev);
	
	/* デバイスクラスを作成する */
	class_motoren = class_create(THIS_MODULE,DEVNAME_MOTOREN);
	if(IS_ERR(class_motoren))
	{
		return PTR_ERR(class_motoren);
	}

	devno = MKDEV(_major_motoren, _minor_motoren);
	/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
	cdev_init(&(cdev_array[cdev_index]), &motoren_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if( cdev_add( &(cdev_array[cdev_index]), devno, 1) < 0 )
	{
		/* 登録に失敗した */
		printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_motoren);
	}
	else
	{
		/* デバイスノードの作成 */
		device_create(
				class_motoren,
				NULL,
				devno,
				NULL,
				DEVNAME_MOTOREN"%u",
				_minor_motoren
		);
	}

	cdev_index++;

	return 0;
}

/*
   Register device drive and create device file
     /dev/rtmotor0

 */
static int motor_register_dev(void)
{
	int retval;
	dev_t dev;
	dev_t devno;
		
	/* 空いているメジャー番号を使ってメジャー
		&マイナー番号をカーネルに登録する */
	retval =  alloc_chrdev_region(
		&dev,		/* 結果を格納するdev_t構造体 */
		DEV_MINOR,	/* ベースマイナー番号 */
		NUM_DEV_MOTOR,	/* デバイスの数 */
		DEVNAME_MOTOR	/* デバイスドライバの名前 */
	);
	
	if( retval < 0 )
	{
		printk(KERN_ERR "alloc_chrdev_region failed.\n" );
		return retval;
	}
	_major_motor = MAJOR(dev);
	
	/* デバイスクラスを作成する */
	class_motor = class_create(THIS_MODULE,DEVNAME_MOTOR);
	if(IS_ERR(class_motor))
	{
		return PTR_ERR(class_motor);
	}

	devno = MKDEV(_major_motor, _minor_motor);
	/* キャラクタデバイスとしてこのモジュールをカーネルに登録する */
	cdev_init(&(cdev_array[cdev_index]), &motor_fops);
	cdev_array[cdev_index].owner = THIS_MODULE;
	if( cdev_add( &(cdev_array[cdev_index]), devno, 1) < 0 )
	{
		/* 登録に失敗した */
		printk(KERN_ERR "cdev_add failed minor = %d\n", _minor_motor);
	}
	else
	{
		/* デバイスノードの作成 */
		device_create(
				class_motor,
				NULL,
				devno,
				NULL,
				DEVNAME_MOTOR"%u",
				_minor_motor
		);
	}

	cdev_index++;

	return 0;
}

/*
    Register up driver module 
 */
int dev_init_module(void)
{
	int retval;
	size_t size;
	

	/* GPIOレジスタがマップ可能か調べる */
	retval = led_gpio_map();
	if( retval != 0 )
	{
		printk( KERN_ALERT "Can not use GPIO registers.\n");
		return -EBUSY;
	}

	/* GPIO初期化 */
	rpi_gpio_function_set( MOTDIR_L_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( MOTDIR_R_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( MOTEN_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( MOTCLK_L_BASE, RPI_GPF_OUTPUT );
	rpi_gpio_function_set( MOTCLK_L_BASE, RPI_GPF_OUTPUT );
	
	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << MOTEN_BASE);
	rpi_gpio_set32(RPI_GPIO_P2MASK, 1 << MOTDIR_L_BASE);

	/* cdev構造体の用意 */
	size = sizeof(struct cdev) * NUM_DEV_TOTAL;
	cdev_array =  (struct cdev*)kmalloc(size, GFP_KERNEL);
	
	/* デバイスドライバをカーネルに登録 */
	
	retval = motorrawr_register_dev();
	if( retval != 0 )
	{
		printk( KERN_ALERT " motor driver register failed.\n");
		return retval;
	}
	
	retval = motorrawl_register_dev();
	if( retval != 0 )
	{
		printk( KERN_ALERT " motor driver register failed.\n");
		return retval;
	}
	
	retval = motoren_register_dev();
	if( retval != 0 )
	{
		printk( KERN_ALERT " motor driver register failed.\n");
		return retval;
	}
	
	retval = motor_register_dev();
	if( retval != 0 )
	{
		printk( KERN_ALERT " motor driver register failed.\n");
		return retval;
	}
	
	printk( KERN_INFO "rtmouse driver register sccessed.\n");
		
	/* GPIOレジスタのアンマップ */
	gpio_unmap();

	/* Inirialize mutex lock.... */
	mutex_init(&lock);	

	printk("module being installed at %lu\n",jiffies);
	return 0;
}

/*
    Clean up driver module 
 */
void dev_cleanup_module(void)
{
	int i;
	dev_t devno;

	/* キャラクタデバイスの登録解除 */
	for(i = 0; i < NUM_DEV_TOTAL; i++)
	{
		cdev_del(&(cdev_array[i]));
	}
	
        /// Right Motor driver (raw)
	// /dev/rtmotor_raw_r0
	devno = MKDEV(_major_motorrawr,_minor_motorrawr); 
	device_destroy(class_motorrawr, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTORRAWR);
	
        /// Left Motor driver (raw)
	// /dev/rtmotor_raw_l0
	devno = MKDEV(_major_motorrawl,_minor_motorrawl); 
	device_destroy(class_motorrawl, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTORRAWL);
		
        ///  Motor softswitch
	// /dev/rtmotoren0
	devno = MKDEV(_major_motoren,_minor_motoren); 
	device_destroy(class_motoren, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTOREN);
	
        ///  Motor driver
	// /dev/rtmotor0
	devno = MKDEV(_major_motor,_minor_motor); 
	device_destroy(class_motor, devno);
	unregister_chrdev_region(devno, NUM_DEV_MOTOR);
	
	/* デバイスノードを取り除く */
	class_destroy( class_motorrawr );
	class_destroy( class_motorrawl );
	class_destroy( class_motoren );
	class_destroy( class_motor );
	
	kfree(cdev_array);
	printk("module being removed at %lu\n",jiffies);
}

/*

 */
module_init(dev_init_module);
module_exit(dev_cleanup_module);

