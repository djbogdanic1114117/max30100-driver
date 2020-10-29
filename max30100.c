#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <asm/mach/map.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>                 //kmalloc()
#include <linux/uaccess.h>              //copy_to/from_user()
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/string.h>

///**** adrese registara iz datasheet-a ***///


#define INT_STATUS     0x00  // Which interrupts are tripped
#define INT_ENABLE     0x01  // Which interrupts are active
#define FIFO_WR_PTR    0x02  // Where data is being written
#define OVRFLOW_CTR    0x03  // Number of lost samples
#define FIFO_RD_PTR    0x04  // Where to read from
#define FIFO_DATA      0x05  // Ouput data buffer
#define MODE_CONFIG    0x06  // Control register
#define SPO2_CONFIG    0x07  // Oximetry settings
#define LED_CONFIG     0x09  // Pulse width and power of LEDs
//RESERVED 0x0A-0x15 address
#define TEMP_INTG      0x16  // Temperature value, whole number
#define TEMP_FRAC      0x17  // Temperature value, fraction
#define REV_ID         0xFE  // Part revision
#define PART_ID        0xFF  // Part ID, normally 0x11
#define DEV_ADDRESS    0x57  // 8bit address converted to 7bit --i2cdetect -y 1
#define DEV_NAME       "max30100" 


#define MODNAME        "max30100"
///*********************************************************************///


struct max30100_device{

  struct cdev cdev;
  struct kobject *kobj_ref;
  int major;
  struct i2c_client *client;
 
};

//struktura koja opisuje uredjaj
static struct max30100_device max30100_device;


/*ako platforma nema stablo uredjaja ova struktura dozvoljava opisivanje kako je 
je uredja povezan na plocu*/
static struct i2c_board_info max30100_i2c_board_info={
      //pomocni makro sa kojim opisujemo datu strukturu
	  //argumenti:ime uredjaja i slave adresa uredjaja na magistrali
      I2C_BOARD_INFO(DEV_NAME,DEV_ADDRESS)
};


//struktura koja predstavlja uredja i2c_bus magistrali 
static struct i2c_client *max30100_client;




/*************** Driver Fuctions **********************/
int max30100_init(void);
void max30100_exit(void);
static ssize_t max30100_open(struct inode *inode, struct file *file);
static ssize_t max30100_release(struct inode *inode, struct file *file);
static ssize_t max30100_read(struct file *filp,
                char *buf, size_t len,loff_t * off);
static ssize_t max30100_write(struct file *filp,
                const char *buf, size_t len, loff_t * off);
                
static struct file_operations max30100_fops= {

   .owner   = THIS_MODULE,
   .open    = max30100_open,
   .release = max30100_release,
   .read    = max30100_read,
   .write   = max30100_write,
};







volatile int ac_values[2] = { 0 };
volatile int mode_values = 0;



#define BUF_LEN 4
char* max30100_buffer;
int max30100_major;






static ssize_t max30100_open(struct inode *inode, struct file *file)
{
       
        return 0;
}
 
 
 
static ssize_t max30100_release(struct inode *inode, struct file *file)
{
        
        return 0;
}
 
 
 




static ssize_t max30100_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
        printk(KERN_INFO "Write function\n");
        return 0;
}








static const struct i2c_device_id max30100_id[] = {
   {
          DEV_NAME,
          DEV_ADDRESS
   },
   {}
};

MODULE_DEVICE_TABLE(i2c,max30100_id);



/*Resetuje MODE_CONFIG registar*/
void hr_res(void )
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  
  temp = i2c_smbus_read_byte_data(dev->client, MODE_CONFIG);
  temp |= (1<<6);
  ret = i2c_smbus_write_byte_data(dev->client, MODE_CONFIG, temp);
}



/*Funkcija koja setuje/omogucava mjerenje temperature*/
void hr_temp_enable(int enable)
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  temp = i2c_smbus_read_byte_data(dev->client, MODE_CONFIG);
  printk(KERN_INFO"Temperature control:%d ", temp);
   
  if ( enable != 0 )
    temp |= (1<<3);
  else
    temp &= ~(1<<3);

  ret=i2c_smbus_write_byte_data(dev->client, MODE_CONFIG, temp);
  printk(KERN_INFO"Temperature control:%d ", temp);
}



/* Funkcija koja podesava mod rada senzora.*/
int8_t set_mode(uint8_t mode)
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  
  if(mode > 7)
    return -1;
  
  temp = i2c_smbus_read_byte_data(dev->client, MODE_CONFIG);
  
  temp |= mode;
  
  ret = i2c_smbus_write_byte_data(dev->client, MODE_CONFIG, temp);
  return 0;
}



/*Setuje mod visoke rezolucije.*/
void spo2_high_resolution_enable(int enable)
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  
  temp = i2c_smbus_read_byte_data(dev->client, SPO2_CONFIG);
  
  if(enable == 1)
    temp |= (1<<6);
  else
    temp &= ~(1<<6);
    
  ret = i2c_smbus_write_byte_data(dev->client, SPO2_CONFIG, temp);
}



/*Setuje sample rate*/
int8_t spo2_set_sample_rate(uint8_t sample_rate)
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  
  if(sample_rate > 7)
    return -1;
  
  temp = i2c_smbus_read_byte_data(dev->client, SPO2_CONFIG);
  
  temp |= (sample_rate << 2);
  
  ret = i2c_smbus_write_byte_data(dev->client, SPO2_CONFIG, temp);
  
  return 0;
}



/*Setuje sirinu pulsa*/
int8_t set_led_pulse_width(uint8_t value)
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  
  if(value > 3)
    return -1;
  
  temp = i2c_smbus_read_byte_data(dev->client, SPO2_CONFIG);
  
  temp |= value;
  
  ret = i2c_smbus_write_byte_data(dev->client, SPO2_CONFIG, temp);
  
  return 0;
}



/*Setuje RED strujni nivo*/
int8_t set_red_current_level(uint8_t level)
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  
  if(level > 15)
    return -1;
  
  temp = i2c_smbus_read_byte_data(dev->client, LED_CONFIG);
  
  temp |= (level << 4);
  
  ret = i2c_smbus_write_byte_data(dev->client, LED_CONFIG, temp);
  
  return 0;
}



/*Setuje IR strujni nivo*/
int8_t set_ir_current_level(uint8_t level)
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  
  if(level > 15)
    return -1;
  
  temp = i2c_smbus_read_byte_data(dev->client, LED_CONFIG);
  
  temp |= level;
  
  ret = i2c_smbus_write_byte_data(dev->client, LED_CONFIG, temp);
  
  return 0;
}



/*Prekid - FIFO bafer skoro pun*/
void fifo_almost_full_int_enable(int enable)
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  
  temp = i2c_smbus_read_byte_data(dev->client, INT_ENABLE);
  
  if(enable == 1)
    temp |= (1<<7);
  else
    temp &= ~(1<<7);
    
  ret = i2c_smbus_write_byte_data(dev->client, INT_ENABLE, temp);
}



/*Funkcija omogucuje prekid kod procitane TEMP.*/
void temp_ready_int_enable(int enable)
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  
  temp = i2c_smbus_read_byte_data(dev->client, INT_ENABLE);
  
  if(enable == 1)
    temp |= (1<<6);
  else
    temp &= ~(1<<6);
    
  ret = i2c_smbus_write_byte_data(dev->client, INT_ENABLE, temp);
}



/*Funkcija omogucuje prekid za spremnost podataka 
  kod citanja podataka o HR.*/
void heartrate_data_ready_int_enable(int enable)
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  
  temp = i2c_smbus_read_byte_data(dev->client, INT_ENABLE);
  
  if(enable == 1)
    temp |= (1<<5);
  else
    temp &= ~(1<<5);
    
  ret = i2c_smbus_write_byte_data(dev->client, INT_ENABLE, temp);
}



/*Funkcija omogucuje prekid za spremnost podataka 
  kod citanja podataka o SPO2.*/
void spo2_data_ready_int_enable(int enable)
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  
  temp = i2c_smbus_read_byte_data(dev->client, INT_ENABLE);
  
  if(enable == 1)
    temp |= (1<<4);
  else
    temp &= ~(1<<4);
    
  ret = i2c_smbus_write_byte_data(dev->client, INT_ENABLE, temp);
}


/*Funkcija za gasenje senzora*/
void hr_shut_down(int enable)
{
  struct max30100_device *dev=&max30100_device;
  uint8_t temp;
  uint8_t ret;
  
  //heartrate1_i2c_hal_read( MODE_CONFIG, 1, &temp );
  temp = i2c_smbus_read_byte_data(dev->client, MODE_CONFIG);

  if ( enable != 0 )
    temp |= (1<<7);
  else
    temp &= ~(1<<7);

    //heartrate1_i2c_hal_write(MODE_CONFIG,1, &temp);
  ret = i2c_smbus_write_byte_data(dev->client, MODE_CONFIG, temp);
  
}


static int max30100_probe(struct i2c_client *client,const struct i2c_device_id *id)
{

    //inicijalizacija uredjaja
	
	
    
    int err,devno;
    struct max30100_device *dev;
    
     //ako client pokupi adresu koja ne odgovara adresi mapiranoj adresi u nasem slucaju je to 0x57
    if(client->addr !=id->driver_data){
      
      printk(KERN_INFO MODNAME ":wrong address (is %d)\n",client->addr);
      return -ENODEV;
  
   }
   

   memset(&max30100_device,0,sizeof(max30100_device));
   dev =&max30100_device;
   //registracija u radni okvir jezgra(framework)
   i2c_set_clientdata(client,dev);

   dev->client=client;
   
   devno= MKDEV(dev->major,0);





    return 0;
}



static int max30100_remove(struct i2c_client *client){

    struct max30100_device *dev=i2c_get_clientdata(client);
    int devno;
    if(dev){
      kobject_put(dev->kobj_ref);
      
      i2c_set_clientdata(client,NULL);
      cdev_del(&dev->cdev);
      devno=MKDEV(dev->major,0);
      unregister_chrdev_region(devno,1);

    }
  
   return 0;
    
}


//struktura koja definise i2c podsitem u okviru modela uredjaja
//nasljedjuje struct device_driver
//predstavlja driver koji opsluzuje max30100 da datoj magistrali
static struct i2c_driver max30100_driver = {

    .driver = {

         .name= DEV_NAME,
         .owner= THIS_MODULE,
     },
     .probe    = max30100_probe,
     .remove   = max30100_remove,
     .id_table = max30100_id 
};


int max30100_init(void)
{
  int result =-1;
  
  printk(KERN_INFO "Inserting max30100 module \n");
  
  /* Registering  char device. */
    result = register_chrdev(0, "max30100", &max30100_fops);
    if (result < 0)
    {
        printk(KERN_INFO "max30100: cannot obtain major number %d\n", max30100_major);
        return result;
    }

    max30100_major = result;
    printk(KERN_INFO "max30100 major number is %d\n", max30100_major);

    max30100_buffer = kmalloc(BUF_LEN, GFP_KERNEL);
    if (!max30100_buffer)
    {
        result = -ENOMEM;
        goto fail_no_mem;
    }
    
    memset(max30100_buffer, 0, BUF_LEN);
    
	//dodavanje novog kontrolera i2c_adapter 
	//i2c_adapter struktura predstavlja kontroler koji identifikuje uredjaj na magistrali
    struct i2c_adapter *adapter = i2c_get_adapter(1);
	int ret;

	if (!adapter) {
		printk(KERN_INFO MODNAME ":ERROR getting i2c adapeter\n");
		ret = -ENODEV;
		goto exit1;

	}
    
	//instanciranje datog uredjaja u nasem slucaju max30100 na magistralu	
	max30100_client = i2c_new_device(adapter, &max30100_i2c_board_info);
	if (!max30100_client) {

		printk(KERN_INFO MODNAME ":ERROR registering i2c device\n");
		ret = -ENODEV;
		goto exit2;
	}

    //instanciranje infrastrukture drivera
	ret = i2c_add_driver(&max30100_driver);
	if (ret < 0) {

		goto exit3;

	}
    


	hr_res();
	
	set_mode(3);
	hr_temp_enable(1); 
	spo2_high_resolution_enable(1);
	spo2_set_sample_rate(1);
	set_led_pulse_width(3);
	set_red_current_level(15);
	set_ir_current_level(14);
	heartrate_data_ready_int_enable(1);
	spo2_data_ready_int_enable(1);
	temp_ready_int_enable(1);

	return 0;
    
  exit3:
    i2c_unregister_device(max30100_client);

  exit2:
    i2c_put_adapter(adapter);

  exit1:
    return ret;
    
  fail_no_mem:
    /* Freeing the major number. */
    unregister_chrdev(max30100_major, "max30100");
	
    return result;
}  


void max30100_exit(void)
{
    hr_shut_down(1);
    printk(KERN_INFO "Removing gpio_driver module\n");
    
    i2c_del_driver(&max30100_driver);
    i2c_unregister_device(max30100_client);  
    
    if (max30100_buffer)
    {
        kfree(max30100_buffer);
    }

    /* Freeing the major number. */
    unregister_chrdev(max30100_major, "gpio_driver");
}

union converter
{
  uint16_t source;
  char tgt[sizeof(uint16_t)]
};

static ssize_t max30100_read(struct file *filp, char  *buf, size_t len, loff_t *off)
{
        
    struct max30100_device *dev = &max30100_device;
	int tmp1, tmp2;
	uint8_t i, sampleNumber = 0;
	uint8_t wrPtr, rdPtr;
	uint8_t fifoptr;
	uint16_t samplesIR, samplesRED;
	int temperatura=0;

	

	
	tmp1 = i2c_smbus_read_byte_data(dev->client, TEMP_INTG);
	tmp2 = i2c_smbus_read_byte_data(dev->client, TEMP_FRAC);
    temperatura= tmp1 + tmp2; //ovako se racuna temperatura, TEMP_INTF + TEMP_FRAC
	
	printk(KERN_INFO "Procitana temperatura: %d [C]", temperatura);

	
//---------------------------------------------------------------------------------------------------------------------------	

	wrPtr = i2c_smbus_read_byte_data(dev->client, FIFO_WR_PTR);

	fifoptr = i2c_smbus_read_byte_data(dev->client, FIFO_DATA);

	rdPtr = i2c_smbus_read_byte_data(dev->client, FIFO_RD_PTR);

	sampleNumber = abs(16 + wrPtr - rdPtr) % 16;



	if (sampleNumber >= 1)
	{
		samplesIR = i2c_smbus_read_word_swapped(dev->client, FIFO_DATA);
		samplesRED = i2c_smbus_read_word_swapped(dev->client, FIFO_DATA); 

		

    union converter conv;
    if (*off == 0)
    { 
    memset(max30100_buffer, 0, BUF_LEN);
    
    conv.source=samplesIR;
    max30100_buffer[0]=conv.tgt[0];
    max30100_buffer[1]=conv.tgt[1];
    
    conv.source=samplesRED;
    max30100_buffer[2]=conv.tgt[0];
    max30100_buffer[3]=conv.tgt[1];
    
    copy_to_user(buf, max30100_buffer, len);
    }
  }
    return (sampleNumber - 1);
}


module_init(max30100_init);
module_exit(max30100_exit);



MODULE_LICENSE("GPL");
MODULE_AUTHOR("PURV 2020");
MODULE_DESCRIPTION("A simple device driver max30100 ");



