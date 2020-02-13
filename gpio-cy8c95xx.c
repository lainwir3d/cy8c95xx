#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include "cy8c95xx.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Richard Rondu <rondu.richard@gmail.com>");
MODULE_DESCRIPTION("CY8C95XX GPIO expander kernel module.");
MODULE_VERSION("0.9");

#define PORT_OUTPUT	0x0	
#define PORT_INPUT	0x1	

#define PWM_CLK_32KHZ 0x00
#define PWM_CLK_24MHZ 0x01
#define PWM_CLK_1_5MHZ 0x02
#define PWM_CLK_93_75KHZ 0x03
#define PWM_CLK_367_6_HZ 0x04
#define PWM_CLK_PREVIOUS 0x05

#define PWM_DISABLED 0x0
#define PWM_ENABLED 0x1

#define INVERSION_DISABLED 0x0
#define INVERSION ENABLED 0x1

#define INPUT_REG_BASE 0x00
#define OUTPUT_REG_BASE 0x08
#define INTERRUPT_REG_BASE 0x10

#define PORT0_OFFSET 0x00
#define PORT1_OFFSET 0x01
#define PORT2_OFFSET 0x02
#define PORT3_OFFSET 0x03
#define PORT4_OFFSET 0x04
#define PORT5_OFFSET 0x05
#define PORT6_OFFSET 0x06
#define PORT7_OFFSET 0x07

#define PORT_CONFIG_REG_BASE 0x18

#define PORT_SELECT_OFFSET 0x00
#define INTERRUPT_MASK_OFFSET 0x01
#define PWM_OUTPUT_OFFSET 0x02
#define PORT_INVERSION_OFFSET 0x03
#define PIN_DIRECTION_OFFSET 0x04
#define DRIVE_PULLUP_OFFSET 0x05
#define DRIVE_PULLDOWN_OFFSET 0x06
#define DRIVE_OPENDRAIN_HIGH 0x07
#define DRIVE_OPENDRAIN_LOW 0x08
#define DRIVE_STRONG 0x09
#define DRIVE_SLOW_STRONG 0x0A
#define DRIVE_HIGHZ 0x0B


#define CONFIG_REG_BASE 0x28

#define PWM_SELECT_OFFSET 0x00
#define PWM_CONFIG_OFFSET 0x01
#define PWM_PERIOD_OFFSET 0x02
#define PWM_PULSE_WIDTH_OFFSET 0x03
#define PWM_PROG_DIVIDER 0x04
#define WDE_EEE_EERO_OFFSET 0x05
#define DEVICE_ID_STATUS_OFFSET 0x06
#define WATCHDOG_OFFSET 0x07
#define COMMAND_OFFSET 0x08


enum {
	CY8C95XX
};

static const struct i2c_device_id cy8c95xx_id[] = {
	{ "cy8c95xx", CY8C95XX },
	{ },
};

#ifdef CONFIG_OF
static const struct of_device_id cy8c95xx_of_table[] = {
	{ .compatible = "cypress,cy8c95xx" },
	{ }
};
MODULE_DEVICE_TABLE(of, cy8c95xx_of_table);
#endif

MODULE_DEVICE_TABLE(i2c, cy8c95xx_id);

struct cy8c95xx_chip {
	struct gpio_chip gpio_chip;

	struct i2c_client *client;	/* "main" client */


	struct mutex	lock;
	uint8_t		reg_out[2];
	
	uint8_t		inReg_shadow[8];
	uint8_t		outReg_shadow[8];
	uint8_t		irqMaskReg_shadow[8];
	uint8_t		dirReg_shadow[8];
	
	uint8_t		pullUpReg_shadow[8];
	uint8_t		pullDownReg_shadow[8];
	uint8_t		DrainHighReg_shadow[8];
	uint8_t		DrainLowReg_shadow[8];
	uint8_t		StrongReg_shadow[8];
	uint8_t		SlowStrongReg_shadow[8];
	uint8_t		HighZReg_shadow[8];

#ifdef CONFIG_GPIO_CY8C95XX_IRQ
	struct mutex		irq_lock;
	uint8_t			irq_mask_cur[8];
	uint8_t			irq_trig_raise[8];
	uint8_t			irq_trig_fall[8];
#endif
};

static int cy8c95xx_resetReg(struct cy8c95xx_chip *chip)
{
	struct i2c_client *client;
	int ret;
	
	struct i2c_msg * msg;
	uint8_t buf[1];

	client = chip->client;
	
	
	msg = kmalloc(sizeof(struct i2c_msg), GFP_KERNEL);
	if (!msg)
			return -ENOMEM;
	
	buf[0] = 0x00;
	
	*msg = (struct i2c_msg) {
		.addr = chip->client->addr,
		.flags = 0,
		.len = 1,
		.buf = buf
	};
	
	ret = i2c_transfer(client->adapter, msg, 1);
	
	if (ret < 0) {
		dev_err(&client->dev, "failed writing\n");
		return ret;
	}

	return 0;
}

static int cy8c95xx_writeReg(struct cy8c95xx_chip *chip, uint8_t reg_addr, uint8_t reg_off, uint8_t data)
{
	struct i2c_client *client;
	int ret;
	
	struct i2c_msg * msg;
	uint8_t buf[2];

	client = chip->client;
	
	
	msg = kmalloc(sizeof(struct i2c_msg), GFP_KERNEL);
	if (!msg)
			return -ENOMEM;
	
	buf[0] = reg_addr + reg_off;
	buf[1] = data;
	
	*msg = (struct i2c_msg) {
		.addr = chip->client->addr,
		.flags = 0,
		.len = 2,
		.buf = buf
	};
	
	ret = i2c_transfer(client->adapter, msg, 1);
	
	if (ret < 0) {
		dev_err(&client->dev, "failed writing\n");
		return ret;
	}

	return 0;
}

static int cy8c95xx_readReg(struct cy8c95xx_chip *chip, uint8_t reg_addr, uint8_t reg_off, uint8_t * data, uint8_t len)
{
	struct i2c_client *client;
	int ret;
	
	struct i2c_msg * msg;
	uint8_t buf[1];

	client = chip->client;
	
	msg = kmalloc_array(2, sizeof(struct i2c_msg), GFP_KERNEL);
	if (!msg)
			return -ENOMEM;
	
	
	buf[0] = reg_addr + reg_off;
	
	*msg = (struct i2c_msg) {
		.addr = chip->client->addr,
		.flags = 0,
		.len = 1,
		.buf = buf
	};
	
	*(msg+1) = (struct i2c_msg) {
		.addr = chip->client->addr,
		.flags = I2C_M_RD,
		.len = len,
		.buf = data
	};
	
	ret = i2c_transfer(client->adapter, msg, 2);
	
	if (ret < 0) {
		dev_err(&client->dev, "failed writing\n");
		return ret;
	}

	return 0;
}

static int cy8c95xx_writeReadReg(struct cy8c95xx_chip *chip, uint8_t reg_addr, uint8_t reg_off, uint8_t *writeData, uint8_t writeLen, uint8_t * readData, uint8_t readLen)
{
	struct i2c_client *client;
	int ret;
	
	struct i2c_msg * msg;
	uint8_t * buf = kmalloc_array(writeLen+1, sizeof(uint8_t), GFP_KERNEL);

	client = chip->client;
	
	
	msg = kmalloc_array(2, sizeof(struct i2c_msg), GFP_KERNEL);
	if (!msg)
			return -ENOMEM;
	
	
	*(buf) = reg_addr + reg_off;
	memcpy(buf+1, writeData, writeLen);
	
	*msg = (struct i2c_msg) {
		.addr = chip->client->addr,
		.flags = 0,
		.len = writeLen+1,
		.buf = buf
	};
	
	*(msg+1) = (struct i2c_msg) {
		.addr = chip->client->addr,
		.flags = I2C_M_RD,
		.len = readLen,
		.buf = readData
	};
	
	ret = i2c_transfer(client->adapter, msg, 2);
	
	if (ret < 0) {
		dev_err(&client->dev, "failed writing\n");
		return ret;
	}

	return 0;
}

__inline__ static void cy8c95xx_gpio_offset2port(uint8_t off, int8_t * port, int8_t * number)
{
		if(!port && !number){
			return;
		}
		
		*port = off / 8;
		*number = off % 8;
		
		// TODO verify coherence more efficiently
		if(*port > 7) *port = -1;
}


int _cy8c95xx_gpio_set_config(struct gpio_chip *gc, unsigned off, enum pin_config_param param, int argument)
{
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);
	
	int8_t port = -1;
	int8_t number = -1;
	int ret = 0;
	
	cy8c95xx_gpio_offset2port(off, &port, &number);
	
	uint8_t mask = 1 << number;
	
#ifdef DEBUG
		dev_warn(&(chip->client)->dev, "_set_config  param=%x    arg=%x    port=%x    mask=%x", param, argument, port, mask);
#endif
    
	mutex_lock(&chip->lock);
	
	switch (param) {
	case PIN_CONFIG_BIAS_HIGH_IMPEDANCE:
	{
		uint8_t portSelect = port;
		
		uint8_t finalDrive = chip->HighZReg_shadow[port];
		finalDrive = finalDrive | mask;
		
		ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, PORT_SELECT_OFFSET, portSelect);
		if (ret){
			goto out;
		}
		
		ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, DRIVE_HIGHZ, finalDrive);
		if (ret){
			goto out;
		}
		
		chip->pullUpReg_shadow[port] &= ~mask; 
		chip->pullDownReg_shadow[port] &= ~mask; 
		chip->DrainHighReg_shadow[port] &= ~mask; 
		chip->DrainLowReg_shadow[port] &= ~mask; 
		chip->StrongReg_shadow[port] &= ~mask; 
		chip->SlowStrongReg_shadow[port] &= ~mask; 
		chip->HighZReg_shadow[port] = finalDrive;
        
#ifdef DEBUG
		dev_warn(&(chip->client)->dev, "_set_config  HighZ");
#endif
		
		break;
	}
	case PIN_CONFIG_BIAS_PULL_DOWN:
	{
		uint8_t portSelect = port;
		
		uint8_t finalDrive = chip->pullDownReg_shadow[port];
		finalDrive = finalDrive | mask;
		
		ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, PORT_SELECT_OFFSET, portSelect);
		if (ret){
			goto out;
		}
		
		ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, DRIVE_PULLDOWN_OFFSET, finalDrive);
		if (ret){
			goto out;
		}
		
		chip->pullUpReg_shadow[port] &= ~mask; 
		chip->pullDownReg_shadow[port] = finalDrive; 
		chip->DrainHighReg_shadow[port] &= ~mask; 
		chip->DrainLowReg_shadow[port] &= ~mask; 
		chip->StrongReg_shadow[port] &= ~mask; 
		chip->SlowStrongReg_shadow[port] &= ~mask; 
		chip->HighZReg_shadow[port] &= ~mask;
		
#ifdef DEBUG
		dev_warn(&(chip->client)->dev, "_set_config  Pull down");
#endif
		
		break;
	}
	case PIN_CONFIG_BIAS_PULL_UP:
	{
		uint8_t portSelect = port;
		
		uint8_t finalDrive = chip->pullUpReg_shadow[port];
		finalDrive = finalDrive | mask;
		
		ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, PORT_SELECT_OFFSET, portSelect);
		if (ret){
			goto out;
		}
		
		ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, DRIVE_PULLUP_OFFSET, finalDrive);
		if (ret){
			goto out;
		}
		
		chip->pullUpReg_shadow[port] = finalDrive; 
		chip->pullDownReg_shadow[port] &= ~mask; 
		chip->DrainHighReg_shadow[port] &= ~mask; 
		chip->DrainLowReg_shadow[port] &= ~mask; 
		chip->StrongReg_shadow[port] &= ~mask; 
		chip->SlowStrongReg_shadow[port] &= ~mask; 
		chip->HighZReg_shadow[port] &= ~mask;
		
#ifdef DEBUG
		dev_warn(&(chip->client)->dev, "_set_config  Pull up");
#endif
		
		break;
	}
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
#ifdef DEBUG
		dev_warn(&(chip->client)->dev, "_set_config  Open drain");
#endif
		break;
	case PIN_CONFIG_DRIVE_OPEN_SOURCE:
#ifdef DEBUG
		dev_warn(&(chip->client)->dev, "_set_config  Open source");
#endif
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
	{
		uint8_t portSelect = port;
		
		uint8_t finalDrive = chip->StrongReg_shadow[port];
		finalDrive = finalDrive | mask;
		
		ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, PORT_SELECT_OFFSET, portSelect);
		if (ret){
			goto out;
		}
		
		ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, DRIVE_STRONG, finalDrive);
		if (ret){
			goto out;
		}
		
		chip->pullUpReg_shadow[port] &= ~mask; 
		chip->pullDownReg_shadow[port] &= ~mask; 
		chip->DrainHighReg_shadow[port] &= ~mask; 
		chip->DrainLowReg_shadow[port] &= ~mask; 
		chip->StrongReg_shadow[port] = finalDrive; 
		chip->SlowStrongReg_shadow[port] &= ~mask; 
		chip->HighZReg_shadow[port] &= ~mask;
		
#ifdef DEBUG
		dev_warn(&(chip->client)->dev, "_set_config  Strong / push pull");
#endif
		
		break;
	}
	default:
#ifdef DEBUG
		dev_warn(&(chip->client)->dev, "_set_config  default");
#endif
		ret = -ENOTSUPP;
		break;
	}
	
	
out:
	mutex_unlock(&chip->lock);
	
	return ret;
}

int cy8c95xx_gpio_set_config(struct gpio_chip *gc, unsigned off, unsigned long config)
{
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);
	
	int8_t port = -1;
	int8_t number = -1;
	
	cy8c95xx_gpio_offset2port(off, &port, &number);
	
#ifdef DEBUG
	dev_warn(&(chip->client)->dev, "set_config %d", config);
#endif
	
	enum pin_config_param param = pinconf_to_config_param(config);
	int argument = pinconf_to_config_argument(config);
	
	return _cy8c95xx_gpio_set_config(gc, off, param, argument);
}



static int cy8c95xx_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);
	int ret;

	int8_t port = -1;
	int8_t number = -1;
	
	cy8c95xx_gpio_offset2port(off, &port, &number);
	
	uint8_t mask = 1 << number;
	
	mutex_lock(&chip->lock);
	
#ifdef DEBUG
	dev_warn(&(chip->client)->dev, "get_value   port=%x    mask=%x  intMask=%x", port, mask, chip->irqMaskReg_shadow[port]);
#endif
	
#ifdef IRQ_VALUE_PREFETCH
	if(chip->irqMaskReg_shadow[port] & mask)	// if interrupt disabled for this pin, get value from i2c, else return shadow value which should be up to date
#endif
	{		
		
		uint8_t data = 0x00;
		ret = cy8c95xx_readReg(chip, INPUT_REG_BASE, port, &data, 1);
		if(ret){
			goto out;
		}
		
#ifdef DEBUG
		dev_warn(&(chip->client)->dev, "get_value from i2c: 0x%x", data);
#endif
		chip->inReg_shadow[port] = data;
	}
		
	ret = chip->inReg_shadow[port];
	ret = (ret & mask) >> number;
		
out:
	mutex_unlock(&chip->lock);
	
	if(ret < 0) dev_err(&(chip->client)->dev, "failed reading. Keeping old value\n");
	return ret;
}


static void cy8c95xx_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	int ret;
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);
	uint8_t finalVal = 0;
	
	// TODO verify gpio is set as output
	
	int8_t port = -1;
	int8_t number = -1;
	uint8_t mask = 0;
	
	cy8c95xx_gpio_offset2port(off, &port, &number);
	
	mask = 1 << number;
	
#ifdef DEBUG
	dev_err(&(chip->client)->dev, "%s , %d:%d\n", "gpio_set_value", port, number);
#endif
	
	//if(port == -1) goto out_failed;  // should not be necessary
	
	mutex_lock(&chip->lock);
	
	finalVal = chip->outReg_shadow[port]; 
	finalVal = (val == 1) ? (finalVal | mask) : (finalVal & ~(mask));
	
	ret = cy8c95xx_writeReg(chip, OUTPUT_REG_BASE, port, finalVal);
	if (ret){
		dev_err(&(chip->client)->dev, "%s failed, %d\n", "gpio_set_value", ret);
	}else{
		chip->outReg_shadow[port] = finalVal;
	}
	
	mutex_unlock(&chip->lock);
	
	return;
}

static void cy8c95xx_gpio_set_multiple(struct gpio_chip *gc, unsigned long *mask, unsigned long *bits)
{
	int i = 0;
	
	uint8_t portMask;
	uint8_t portBits;
		
	uint8_t finalVal;
	
	int ret;
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);
	
#ifdef DEBUG
	dev_err(&(chip->client)->dev, "%s\n", "gpio_set_value_multiple");
#endif
	
	for(i=0; i < 8; i++){
		portMask = (*mask >> (8*i));
		portBits = (*bits >> (8*i));
		
		if(portMask){
			finalVal = chip->outReg_shadow[i] & (~portMask);
			finalVal = finalVal | portBits;
			
#ifdef DEBUG
			dev_err(&(chip->client)->dev, "%s , port %d mask 0x%x bits 0x%x  finalVal 0x%x\n", "gpio_set_value_multiple", i, portMask, portBits, finalVal);
#endif
			
			mutex_lock(&chip->lock);
			
			ret = cy8c95xx_writeReg(chip, OUTPUT_REG_BASE, i, finalVal);
			if (ret){
				dev_err(&(chip->client)->dev, "%s failed, port %d mask 0x%x bits 0x%x  finalVal 0x%x, %d\n", "gpio_set_value_multiple", i, portMask, portBits, finalVal, ret);
			}else{
				chip->outReg_shadow[i] = finalVal;
			}
			
			mutex_unlock(&chip->lock);
		}
	}	
}

static int cy8c95xx_gpio_get_direction(struct gpio_chip *gc, unsigned off)
{
	
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);

	int8_t port = -1;
	int8_t number = -1;
	
	cy8c95xx_gpio_offset2port(off, &port, &number);
	
	uint8_t mask = 1 << number;
	
	uint8_t rawDir = (chip->dirReg_shadow[port] & mask) >> number;
	
#ifdef DEBUG
	dev_warn(&(chip->client)->dev, "get_direction  port=%x    mask=%x  rawdir=%x", port, mask, rawDir);
#endif
	
	return rawDir;
}


static int cy8c95xx_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);
	
	int8_t port = -1;
	int8_t number = -1;
	uint8_t finalDir = 0;
	int ret = 0;
	
	cy8c95xx_gpio_offset2port(off, &port, &number);
	
	uint8_t mask = 1 << number;
	
#ifdef DEBUG
	dev_warn(&(chip->client)->dev, "set_direction_input  port=%x    mask=%x", port, mask);
#endif

	/*
	ret = _cy8c95xx_gpio_set_config(gc, off, PIN_CONFIG_BIAS_HIGH_IMPEDANCE, 0);
	if (ret){
		goto out;
	}
	*/
	
	mutex_lock(&chip->lock);
	
	finalDir = chip->dirReg_shadow[port]; 
	finalDir = (finalDir | mask);
	
	ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, PORT_SELECT_OFFSET, port);
	if (ret){
		goto out;
	}
	
	ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, PIN_DIRECTION_OFFSET, finalDir);
	if (ret){
		goto out;
	}
	
	chip->dirReg_shadow[port] = finalDir;

	ret = 0; // everything went accordingly
	
out:	
	mutex_unlock(&chip->lock);
	
	if(ret) dev_err(&(chip->client)->dev, "%s failed, %d\n", "gpio_set_value", ret);
	
	return ret;
}

static int cy8c95xx_gpio_direction_output(struct gpio_chip *gc, unsigned off, int val)
{
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);
	
	int8_t port = -1;
	int8_t number = -1;
	uint8_t finalDir = 0;
	int ret = 0;
	
	cy8c95xx_gpio_offset2port(off, &port, &number);
	
	uint8_t mask = 1 << number;

#ifdef DEBUG
	dev_warn(&(chip->client)->dev, "set_direction_output  port=%x    mask=%x", port, mask);
#endif
	
	/*
	ret = _cy8c95xx_gpio_set_config(gc, off, PIN_CONFIG_DRIVE_PUSH_PULL, 0);
	if (ret){
		goto out;
	}
	*/
	
	mutex_lock(&chip->lock);
	
	finalDir = chip->dirReg_shadow[port]; 
	finalDir = (finalDir & ~(mask));
	
	ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, PORT_SELECT_OFFSET, port);
	if (ret){
		goto out;
	}
	
	ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, PIN_DIRECTION_OFFSET, finalDir);
	if (ret){
		goto out;
	}
	
	chip->dirReg_shadow[port] = finalDir;
	
	ret = 0;
	
out:	
	mutex_unlock(&chip->lock);
	
	if(ret) dev_err(&(chip->client)->dev, "%s failed, %d\n", "gpio_set_value", ret);
	return ret;
}

static struct cy8c95xx_platform_data *of_gpio_cy8c95xx(struct device *dev)
{
	struct cy8c95xx_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->gpio_base = -1;

	return pdata;
}

#ifdef CONFIG_GPIO_CY8C95XX_IRQ

static void cy8c95xx_irq_update_mask(struct cy8c95xx_chip *chip)
{
	mutex_lock(&chip->lock);
	int ret = 0;
	
	for(int i=0; i<8; i++){
		if(chip->irqMaskReg_shadow[i] != chip->irq_mask_cur[i]){
			ret = cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, PORT_SELECT_OFFSET, i);
			ret += cy8c95xx_writeReg(chip, PORT_CONFIG_REG_BASE, INTERRUPT_MASK_OFFSET, chip->irq_mask_cur[i]);
			if(ret == 0){
				chip->irqMaskReg_shadow[i] = chip->irq_mask_cur[i];
			}else{
				dev_err(&(chip->client)->dev, "%s failed, port %d, %d\n", "interrupt mask updating", i, ret);
			}
		}
	}

	mutex_unlock(&chip->lock);
}

static void cy8c95xx_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);
	
	uint8_t port = -1;
	int8_t number = -1;
	
	cy8c95xx_gpio_offset2port(d->hwirq, &port, &number);
	
	uint8_t mask = 1 << number;

	chip->irq_mask_cur[port] |= mask;
}

static void cy8c95xx_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);
	
	uint8_t port = -1;
	int8_t number = -1;
	
	cy8c95xx_gpio_offset2port(d->hwirq, &port, &number);
	
	uint8_t mask = 1 << number;

	chip->irq_mask_cur[port] &= ~(mask);
}

static void cy8c95xx_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->irq_lock);
	memcpy(chip->irq_mask_cur, chip->irqMaskReg_shadow, 8);
}

static void cy8c95xx_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);

	cy8c95xx_irq_update_mask(chip);

	mutex_unlock(&chip->irq_lock);
}

static int cy8c95xx_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct cy8c95xx_chip *chip = gpiochip_get_data(gc);
	uint16_t off = d->hwirq;
	
	
	int8_t port = -1;
	int8_t number = -1;
	uint8_t mask = 0;
	
	cy8c95xx_gpio_offset2port(off, &port, &number);
	
	mask = 1 << number;

	if (!(type & IRQ_TYPE_EDGE_BOTH)) {
		dev_err(&chip->client->dev, "irq %d: unsupported type %d\n",
			d->irq, type);
		return -EINVAL;
	}

	if (type & IRQ_TYPE_EDGE_FALLING)
		chip->irq_trig_fall[port] |= mask;
	else
		chip->irq_trig_fall[port] &= ~mask;

	if (type & IRQ_TYPE_EDGE_RISING)
		chip->irq_trig_raise[port] |= mask;
	else
		chip->irq_trig_raise[port] &= ~mask;

	return 0;
}

static int cy8c95xx_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct cy8c95xx_chip *chip = irq_data_get_irq_chip_data(data);

	irq_set_irq_wake(chip->client->irq, on);
	return 0;
}

static struct irq_chip cy8c95xx_irq_chip = {
	.name			= "cy8c95xx",
	.irq_mask		= cy8c95xx_irq_mask,
	.irq_unmask		= cy8c95xx_irq_unmask,
	.irq_bus_lock		= cy8c95xx_irq_bus_lock,
	.irq_bus_sync_unlock	= cy8c95xx_irq_bus_sync_unlock,
	.irq_set_type		= cy8c95xx_irq_set_type,
	.irq_set_wake		= cy8c95xx_irq_set_wake,
};

static void cy8c95xx_irq_pending(struct cy8c95xx_chip *chip, uint8_t pending[8])
{
	int ret;

	uint8_t irqStatus[8] = {0};
#ifdef FAST_INTERRUPT
    int nbRegReq = 0;
    for(int i=0; i<8; i++) if(chip->irqMaskReg_shadow[i] != 0xff) nbRegReq += 1;
    
    if(nbRegReq > 2){
        dev_warn(&(chip->client)->dev, "Using optimized path for interrupt\n");
        ret = cy8c95xx_readReg(chip, INTERRUPT_REG_BASE, 0, irqStatus, 8);
        if(ret){
            return;
        }
    }else{
        for(int i=0; i<8; i++){
            if(chip->irqMaskReg_shadow[i] != 0xff){ // if there is some interrupt activated on this port
                ret = cy8c95xx_readReg(chip, INTERRUPT_REG_BASE, i, &irqStatus[i], 1);
                if(ret){
                    dev_err(&(chip->client)->dev, "%s failed, port %d, %d\n", "interrupt pending check", i, ret);
                }
                
                if(irqStatus[i]){ // if there is a interrupt flag on this port
#ifdef DEBUG
                    dev_err(&(chip->client)->dev, "%s, port %d\n", "interrupt input port read", i);
#endif
                    
#ifdef IRQ_VALUE_PREFETCH
                    ret = cy8c95xx_readReg(chip, INPUT_REG_BASE, i, &chip->inReg_shadow[i], 1); // update input shadow register
                    if(ret){
                        dev_err(&(chip->client)->dev, "%s failed, port %d, %d\n", "interrupt input port read", i, ret);
                    }
#endif
                    
                }
            }
        }
    }
#else
	ret = cy8c95xx_readReg(chip, INTERRUPT_REG_BASE, 0, irqStatus, 8);
	if(ret){
		return;
	}

#ifdef IRQ_VALUE_PREFETCH
	ret = cy8c95xx_readReg(chip, INPUT_REG_BASE, 0, chip->inReg_shadow, 8);
	if(ret){
		return;
	}
#endif
#endif
	
	for(int i=0; i<8; i++){
		pending[i] = (irqStatus[i] & chip->irq_trig_fall[i]) & ~(chip->inReg_shadow[i]);
		pending[i] |= (irqStatus[i] & chip->irq_trig_raise[i]) & (chip->inReg_shadow[i]);
	}
}

static irqreturn_t cy8c95xx_irq_handler(int irq, void *devid)
{
	struct cy8c95xx_chip *chip = devid;
	uint8_t pending[8] = {0};
	uint8_t level;

	cy8c95xx_irq_pending(chip, pending);

	for(int i=0; i<8; i++){
		//dev_err(&(chip->client)->dev, "%s, port %d pending: 0x%x    mask: 0x%x\n", "interrupt handler", i, pending[i], chip->irqMaskReg_shadow[i]);
		do {
			level = __ffs(pending[i]);
			handle_nested_irq(irq_find_mapping(chip->gpio_chip.irqdomain,
							level + i*8));

			pending[i] &= ~(1 << level);
		} while(pending[i]);
		//dev_err(&(chip->client)->dev, "%s, port %d pending: 0x%x    mask: 0x%x\n", "interrupt handler", i, pending[i], chip->irqMaskReg_shadow[i]);
	}
	
	//dev_err(&(chip->client)->dev, "%s\n", "interrupt handler done");

	return IRQ_HANDLED;
}

static int cy8c95xx_irq_setup(struct cy8c95xx_chip *chip,
			     const struct i2c_device_id *id)
{
	struct i2c_client *client = chip->client;
	struct cy8c95xx_platform_data *pdata = dev_get_platdata(&client->dev);
	int irq_base = 0;
	int ret;

	if (((pdata && pdata->irq_base) || client->irq)) {
		if (pdata)
			irq_base = pdata->irq_base;

		mutex_init(&chip->irq_lock);

		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, cy8c95xx_irq_handler, IRQF_ONESHOT |
				IRQF_TRIGGER_RISING | IRQF_SHARED,
				dev_name(&client->dev), chip);
		if (ret) {
			dev_err(&client->dev, "failed to request irq %d\n",
				client->irq);
			return ret;
		}
		ret =  gpiochip_irqchip_add_nested(&chip->gpio_chip,
						   &cy8c95xx_irq_chip,
						   irq_base,
						   handle_simple_irq,
						   IRQ_TYPE_NONE);
		if (ret) {
			dev_err(&client->dev,
				"could not connect irqchip to gpiochip\n");
			return ret;
		}
		gpiochip_set_nested_irqchip(&chip->gpio_chip,
					    &cy8c95xx_irq_chip,
					    client->irq);
	}

	return 0;
}

#else /* CONFIG_GPIO_CY8C95XX_IRQ */
static int cy8c95xx_irq_setup(struct cy8c95xx_chip *chip,
			     const struct i2c_device_id *id)
{
	struct i2c_client *client = chip->client;
	struct cy8c95xx_platform_data *pdata = dev_get_platdata(&client->dev);

	if ((pdata && pdata->irq_base) || client->irq)
		dev_warn(&client->dev, "interrupt support not compiled in\n");

	return 0;
}
#endif

static ssize_t cy8c95xx_show_ee_por_default(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t writeData = 0x04; //read eeprom default
	uint8_t readData[146] = {};
	
	int ret = 0;
	
	struct cy8c95xx_chip *chip = dev_get_drvdata(dev);
	
	if(chip){
		mutex_lock(&chip->lock);
		
		ret = cy8c95xx_writeReadReg(chip, CONFIG_REG_BASE, COMMAND_OFFSET, &writeData, 1, readData, 146);
	
		if(ret){
			dev_warn(&(chip->client)->dev, "read ee por default failed, %d\n", ret);
			ret = sprintf(buf, "error reading eeprom POR default");
		}else{
			int off=0;
			for(int i=0; i<146; i++){
				off += sprintf(buf+off, "0x%x ", readData[i]);
			}
			off += sprintf(buf+off, "\n");
			
			ret = off;
		}
		
		mutex_unlock(&chip->lock);
	}
	
	
	return ret;
}

static ssize_t cy8c95xx_store_config_to_ee_por_default(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)        //echo Command time,This function will be called
{
	int ret = 0;
	
	struct cy8c95xx_chip *chip = dev_get_drvdata(dev);
	
	if(chip){
		mutex_lock(&chip->lock);
		
		if(strncmp(buf, "save_current", 12) == 0){
			
			ret = cy8c95xx_writeReg(chip, CONFIG_REG_BASE, COMMAND_OFFSET, 0x01);
			
			if(ret){
				dev_warn(&(chip->client)->dev, "save_current ee por default failed, %d\n", ret);
				ret = -EIO;
			}else{
				ret = len;
			}
			
		}else if(strncmp(buf, "restore_factory", 15) == 0){
			ret = cy8c95xx_writeReg(chip, CONFIG_REG_BASE, COMMAND_OFFSET, 0x02);
			
			if(ret){
				dev_warn(&(chip->client)->dev, "restore_factory ee por default failed, %d\n", ret);
				ret = -EIO;
			}else{
				ret = len;
			}
			
		}else if(strncmp(buf, "load_default", 15) == 0){
			ret = cy8c95xx_writeReg(chip, CONFIG_REG_BASE, COMMAND_OFFSET, 0x07);
			
			if(ret){
				dev_warn(&(chip->client)->dev, "restore_default ee por default failed, %d\n", ret);
				ret = -EIO;
			}else{
				ret = len;
			}
			
		}else{
			ret = -EIO;
		}
		
		mutex_unlock(&chip->lock);
	}
	
	
	return ret;
}

static DEVICE_ATTR(ee_por_default, S_IWUSR|S_IRUSR, cy8c95xx_show_ee_por_default, cy8c95xx_store_config_to_ee_por_default);

static int cy8c95xx_setup_gpio(struct cy8c95xx_chip *chip,
					const struct i2c_device_id *id,
					unsigned gpio_start)
{
	struct gpio_chip *gc = &chip->gpio_chip;
	int port = 1;

	gc->get_direction = cy8c95xx_gpio_get_direction;
	gc->direction_input = cy8c95xx_gpio_direction_input;
	gc->direction_output = cy8c95xx_gpio_direction_output;
	gc->set = cy8c95xx_gpio_set_value;
	gc->set_multiple = cy8c95xx_gpio_set_multiple;
	gc->set_config = cy8c95xx_gpio_set_config;
	
	gc->get = cy8c95xx_gpio_get_value;
	gc->can_sleep = true;

	gc->base = gpio_start;
	gc->ngpio = 64;
	gc->label = chip->client->name;
	gc->parent = &chip->client->dev;
	gc->owner = THIS_MODULE;

	return port;
}


static int cy8c95xx_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	
	struct cy8c95xx_platform_data *pdata;
	struct device_node *node;
	struct cy8c95xx_chip *chip;
	uint16_t addr, addr_eeprom;
	int ret, nr_port;

	pdata = dev_get_platdata(&client->dev);
	node = client->dev.of_node;

	if (!pdata && node)
		pdata = of_gpio_cy8c95xx(&client->dev);

	if (!pdata) {
		dev_dbg(&client->dev, "no platform data\n");
		return -EINVAL;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;
	chip->client = client;

	nr_port = cy8c95xx_setup_gpio(chip, id, pdata->gpio_base);
	chip->gpio_chip.parent = &client->dev;

	addr = (client->addr & 0x7f);
	addr_eeprom = (client->addr & 0x7f) | 0x40;

	chip->client = client;
	
	mutex_init(&chip->lock);

	mutex_lock(&chip->lock);
	
	uint8_t writeData = 0x06;
	uint8_t readData[146] = {};
	ret = cy8c95xx_writeReadReg(chip, CONFIG_REG_BASE, COMMAND_OFFSET, &writeData, 1, readData, 146);
	if(ret) goto out;
	
	memcpy(chip->outReg_shadow, readData, 8);
	memcpy(chip->irqMaskReg_shadow, &readData[0x08], 8);
	memcpy(chip->dirReg_shadow, &readData[0x20], 8);
	
	ret = cy8c95xx_readReg(chip, INPUT_REG_BASE, PORT0_OFFSET, chip->inReg_shadow, 8);
	if(ret) goto out;
	
	for(int i=0; i<8; i++){
		chip->pullUpReg_shadow[i] = readData[0x28 + (i*7)];
		chip->pullDownReg_shadow[i] = readData[0x29 + (i*7)];
		chip->DrainHighReg_shadow[i] = readData[0x2a + (i*7)];
		chip->DrainLowReg_shadow[i] = readData[0x2b + (i*7)];
		chip->StrongReg_shadow[i] = readData[0x2c + (i*7)];
		chip->SlowStrongReg_shadow[i] = readData[0x2d + (i*7)];
		chip->HighZReg_shadow[i] = readData[0x2e + (i*7)];
		
		dev_warn(&client->dev, "inReg_shadow[%d] =  0x%x", i, chip->inReg_shadow[i]);
		dev_warn(&client->dev, "dirReg_shadow[%d] =  0x%x", i, chip->dirReg_shadow[i]);
		dev_warn(&client->dev, "pullUpReg_shadow[%d] =  0x%x", i, chip->pullUpReg_shadow[i]);
		dev_warn(&client->dev, "StrongReg_shadow[%d] =  0x%x", i, chip->StrongReg_shadow[i]);
		dev_warn(&client->dev, "HighZReg_shadow[%d] =  0x%x", i, chip->HighZReg_shadow[i]);
	}
	
	ret = gpiochip_add_data(&chip->gpio_chip, chip);
	if(ret) goto out;

	
	ret = cy8c95xx_irq_setup(chip, id);
	if (ret) {
		gpiochip_remove(&chip->gpio_chip);
		goto out;
	}
	

	if (pdata && pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}
	
	dev_set_drvdata(&client->dev, chip);
	device_create_file(&client->dev, &dev_attr_ee_por_default);

	i2c_set_clientdata(client, chip);
	ret = 0;

out:
	mutex_unlock(&chip->lock);
	
	if(ret) dev_warn(&client->dev, "setup failed, %d\n", ret);
	
	return ret;
}

static int cy8c95xx_remove(struct i2c_client *client)
{
	struct cy8c95xx_platform_data *pdata = dev_get_platdata(&client->dev);
	struct cy8c95xx_chip *chip = i2c_get_clientdata(client);
	
	if (pdata && pdata->teardown) {
		int ret;
		
		ret = pdata->teardown(client, chip->gpio_chip.base,
							  chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed, %d\n",
					"teardown", ret);
			return ret;
		}
	}
	
	gpiochip_remove(&chip->gpio_chip);
	
	return 0;
	
}

static struct i2c_driver cy8c95xx_driver = {
	.driver = {
		.name		= "cy8c95xx",
		.of_match_table	= of_match_ptr(cy8c95xx_of_table),
	},
	.probe		= cy8c95xx_probe,
	.remove		= cy8c95xx_remove,
	.id_table	= cy8c95xx_id,
};


static int __init cy8c95xx_init(void) {
	printk(KERN_INFO "Hello, World!\n");
	return i2c_add_driver(&cy8c95xx_driver);
}


static void __exit cy8c95xx_exit(void) {
	printk(KERN_INFO "Goodbye, World!\n");
	i2c_del_driver(&cy8c95xx_driver);
}

subsys_initcall(cy8c95xx_init);
module_exit(cy8c95xx_exit);
