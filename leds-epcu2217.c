/* -*- Mode:C; c-file-style:"linux"; -*-
 * drivers/leds/leds-epcu2217.c
 * Copyright (C) 2024 Gerd Pauli, gp at high-consulting dot de 
 * Led driver which supports following Nuvoton chips:                                                        
 *      NCT6116D

 * used in ADVANTEC EPCU 2217 with Board AIMB-U217 
 * using GPIO on Connector 
 * 16-Bit General Purpose I/O Pin Header (GPIO1)
 * which are connected to GPIO30 to GPIO47
 * 
 * CHIP HEADER       used 
 * ----------------------
 * GP30 GPIO0
 * GP31 GPIO1
 * GP32 GPIO2
 * GP33 GPIO3
 * GP34 GPIO4
 * GP35 GPIO5
 * GP36 GPIO6
 * GP37 GPIO7
 * GP40 GPIO8         x
 * GP41 GPIO9         x
 * GP42 GPIO10        x
 * GP43 GPIO11
 * GP44 GPIO12
 * GP45 GPIO13
 * GP46 GPIO14
 * GP47 GPIO15
 * 
 * Example LED connection:
 *   VCC -> LED -> GPIO-Pin  ( 12mA source sink capability )
 *   0 = led on
 *   1 = led off
 * 
 * thanks to
 *  Tasanakorn Phaipool tasanakorn at gmail dot  (gpio-nct6106d.c)
 *  Sheng-Yuan Huang syhuang3 at nuvoton dot com (gpio-nct6106d.c)
 *  Alan Mizrahi, alan at mizrahi dot com dot ve (leds-apu.c)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* print inb/outb commands */
//#define IODEBUG

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define LEDON              0x01
#define LEDOFF             0x00

static int _nct6116d_addr;

/* LED access parameters */
struct epcu2217_param {
	int addr;                /* for ioread/iowrite */
	int led;                 /* bit position to switch gpio */
};

/* LED private data */
struct epcu2217_led_priv {
	struct led_classdev cdev;
	struct epcu2217_param param;
};
#define cdev_to_priv(c) container_of(c, struct epcu2217_led_priv, cdev)

/* LED profile */
struct epcu2217_led_profile {
	const char *name;
	enum led_brightness brightness;
	int led;
};

struct epcu2217_led_pdata {
	struct platform_device *pdev;
	struct epcu2217_led_priv *pled;
	spinlock_t lock;
};

static struct epcu2217_led_pdata *epcu2217_led;

#define GPIO3 0xec
#define GPIO4 0xf0
static const struct epcu2217_led_profile epcu2217_led_profile[] = {
 	{ "epcu:1", LEDOFF, 0 },
	{ "epcu:2", LEDOFF, 1 },
	{ "epcu:3", LEDOFF, 2 }
};

/* debug use */
static bool debug = false;
module_param(debug, bool, S_IRUGO);
MODULE_PARM_DESC(debug, "debug mode enable");

#define debug_print(fmt, ...)						\
	do { if (debug) printk(fmt, __VA_ARGS__); else pr_debug(fmt, __VA_ARGS__); } while (0)

/*
 * Super-I/O registers
 */

#define SIO_LDSEL               0x07    /* Logical device select */
#define SIO_CHIPID              0x20    /* Chaip ID (2 bytes) */
#define SIO_GPIO_ENABLE         0x30    /* GPIO enable */
#define SIO_GPIO0_MODE          0xE0    /* GPIO0 Mode OpenDrain/Push-Pull */
#define SIO_GPIO1_MODE          0xE1    /* GPIO1 Mode OpenDrain/Push-Pull */
#define SIO_GPIO4_MODE          0xE4    /* GPIO1 Mode OpenDrain/Push-Pull */
#define SIO_GPIO4_DIR           0xF0    /* GPIO4 Direction IN/OUT */
#define SIO_GPIO4_DATA          0xF1    /* GPIO4 Data */
#define SIO_LD_GPIO_MODE        0x0F    /* GPIO mode control device */
#define SIO_UNLOCK_KEY          0x87    /* Key to enable Super-I/O */
#define SIO_LOCK_KEY            0xAA    /* Key to disable Super-I/O */

/*
 * Super-I/O functions.
 */

#ifdef IODEBUG

static void s_outb(int val, int base) {
        debug_print("%s: 0x%x -> 0x%x\n",__func__,val,base);
	outb(val, base);
}

static int s_inb(int base) {
        int val;
	val = inb(base);
        debug_print("%s: 0x%x -> 0x%x\n",__func__,base,val);
        return val;
}

#define OUTB(a,b) s_outb(a,b)
#define INB(a) s_inb(a)

#else

#define OUTB(a,b) outb(a,b)
#define INB(a) inb(a)

#endif

static inline int superio_inb(int base, int reg) {
        OUTB(reg, base);
        return INB(base + 1);
}

static int superio_inw(int base, int reg) {
        int val;
	
        OUTB(reg++, base);
        val = INB(base + 1) << 8;
        OUTB(reg, base);
        val |= INB(base + 1);
	
	return val;
}

static inline void superio_outb(int base, int reg, int val) {
        OUTB(reg, base);
        OUTB(val, base + 1);
}

static inline void superio_setbit( int base, int reg, int position) {
	int val;
        val = superio_inb(base, reg);
	val |= ( 1 << position );
	superio_outb(base, reg, val);
}

static inline void superio_clearbit( int base, int reg, int position) {
	int val;
        val = superio_inb(base, reg);
	val &= ~(1 << position);
	superio_outb(base, reg, val);
}

static inline int superio_enter(int base) {
        /* Don't step on other drivers' I/O space by accident. */
	if (!request_muxed_region(base, 2, KBUILD_MODNAME)) {
                pr_err(KBUILD_MODNAME "I/O address 0x%04x already in use\n", base);
                return -EBUSY;
        }

        /* According to the datasheet the key must be send twice. */
        OUTB(SIO_UNLOCK_KEY, base);
        OUTB(SIO_UNLOCK_KEY, base);
	debug_print("%s: done\n",__func__);
        return 0;
}

static inline void superio_select(int base, int ld) {
	/* select page */
        OUTB(SIO_LDSEL, base);
        OUTB(ld, base + 1);
	debug_print("%s: 0x%x\n",__func__,ld);
}

static inline void superio_exit(int base) {
	/* SuperIO now locked again */
        OUTB(SIO_LOCK_KEY, base);
	release_region(base, 2);
	debug_print("%s:done\n",__func__);
}

/* ---------------------------------------------- */

static void epcu2217_led_brightness_set(struct led_classdev *led, enum led_brightness value)
{
	struct epcu2217_led_priv *pled = cdev_to_priv(led);
	int err;

	
	debug_print("%s: base: 0x%x bit: %d set %d\n",
		    __func__,
		    pled->param.addr,
		    pled->param.led,
		    value);
	
	spin_lock(&epcu2217_led->lock);
	err = superio_enter(pled->param.addr);
	if (err) {
		pr_warn("superio chip is busy");
		spin_unlock(&epcu2217_led->lock);
                return;
	}
	
	superio_select(pled->param.addr, SIO_LDSEL);
	/* led is on when bit set ( invers ) */
	
	if (value)
		superio_clearbit(pled->param.addr,SIO_GPIO4_DATA,pled->param.led);
        else
		superio_setbit(pled->param.addr,SIO_GPIO4_DATA,pled->param.led);
        superio_exit(pled->param.addr);
	
	spin_unlock(&epcu2217_led->lock);
}

static int epcu2217_led_config(struct device *dev, struct epcu2217_led_pdata *epcu2217lp)
{
	int i;
	int err;

	epcu2217_led->pled = devm_kcalloc(dev,
		ARRAY_SIZE(epcu2217_led_profile), sizeof(struct epcu2217_led_priv),
		GFP_KERNEL);

	if (!epcu2217_led->pled)
		return -ENOMEM;
	
	if (!_nct6116d_addr)
		return -ENODEV;
		
	/* enter superio chip for basic gpio configuration */
	err = superio_enter(_nct6116d_addr);
        if (err)
                return err;
	/* enable GPIO for bank 
	 * Logical Device 7 (GPIO) CR30h bit 4 GPIO4 active
         */ 
	superio_select(_nct6116d_addr, SIO_LDSEL);
	superio_setbit(_nct6116d_addr,SIO_GPIO_ENABLE,4);

         /* set Open Drain mode
          * Logical Device F (GPIO) CRE4h set all
          */
	superio_select(_nct6116d_addr, SIO_LD_GPIO_MODE);
	superio_setbit(_nct6116d_addr,SIO_GPIO4_MODE,0);
	superio_setbit(_nct6116d_addr,SIO_GPIO4_MODE,1);
	superio_setbit(_nct6116d_addr,SIO_GPIO4_MODE,2);
          
        /* set GPIO output
         * Logical Device 7 (GPIO) CRF0h. GPIO4 Data Register clear 0,1,2
         */
	superio_select(_nct6116d_addr, SIO_LDSEL);
	superio_outb(_nct6116d_addr, SIO_GPIO4_DIR, 0xf8);

	/* done */
	superio_exit(_nct6116d_addr);

	for (i = 0; i < ARRAY_SIZE(epcu2217_led_profile); i++) {
		struct epcu2217_led_priv *pled = &epcu2217_led->pled[i];
		struct led_classdev *led_cdev = &pled->cdev;

		led_cdev->name = epcu2217_led_profile[i].name;
		led_cdev->brightness = epcu2217_led_profile[i].brightness;
		led_cdev->max_brightness = 1;
		led_cdev->flags = LED_CORE_SUSPENDRESUME;
		led_cdev->brightness_set = epcu2217_led_brightness_set;

		pled->param.addr = _nct6116d_addr;
		pled->param.led = epcu2217_led_profile[i].led;
			
		err = led_classdev_register(dev, led_cdev);
		if (err)
			goto error;

		epcu2217_led_brightness_set(led_cdev, epcu2217_led_profile[i].brightness);
	}

	return 0;

error:
	while (i-- > 0)
		led_classdev_unregister(&epcu2217_led->pled[i].cdev);

	return err;
}

static int __init epcu2217_led_probe(struct platform_device *pdev)
{
	debug_print("%s:\n",__func__);
	epcu2217_led = devm_kzalloc(&pdev->dev, sizeof(*epcu2217_led), GFP_KERNEL);

	if (!epcu2217_led)
		return -ENOMEM;

	epcu2217_led->pdev = pdev;
	spin_lock_init(&epcu2217_led->lock);
	return epcu2217_led_config(&pdev->dev, epcu2217_led);
}

static struct platform_driver epcu2217_led_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
};

#define SIO_ID_MASK             0xFFF0
#define SIO_NCT6116D_ID         (0xd280&SIO_ID_MASK)    /* Chip ID */

static int __init nct6116d_find(int addr) {
	int err;
	int devid;
	
	_nct6116d_addr = 0;
	err = superio_enter(addr);
	if (err)
                return err;
	err = -ENODEV;
	devid = superio_inw(addr, SIO_CHIPID) & SIO_ID_MASK;
	debug_print("%s: 0x%x id @0x%x\n",__func__,devid,addr);
	if ( devid == SIO_NCT6116D_ID ) {
		_nct6116d_addr = addr;
		pr_info("Found Superio nct2116d at %#x chip id 0x%04x\n",addr,devid);
		err = 0;
	} else {
		pr_info("Unsupported device 0x%04x\n", devid);
		goto err;
	}
	/* some other stuff if needed */
err:
	superio_exit(addr);
	return err;
}

static int __init epcu2217_led_init(void)
{
	struct platform_device *pdev;
	int err;

	if (nct6116d_find(0x2e) &&
            nct6116d_find(0x4e))
		return -ENODEV;

	pdev = platform_device_register_simple(KBUILD_MODNAME, -1, NULL, 0);
	if (IS_ERR(pdev)) {
		pr_err("Device allocation failed\n");
		return PTR_ERR(pdev);
	}

	err = platform_driver_probe(&epcu2217_led_driver, epcu2217_led_probe);
	if (err) {
		pr_err("Probe platform driver failed\n");
		platform_device_unregister(pdev);
	}

	return err;
}

static void __exit epcu2217_led_exit(void)
{
	int i;
	int err;
	if (_nct6116d_addr) {
		/* enter superio chip for basic gpio configuration */
		err = superio_enter(_nct6116d_addr);
		if (!err) {
			/* set GPIO output
			 * Logical Device 7 (GPIO) CRF0h. GPIO4 Data Register set 0,1,2
			 */
			superio_select(_nct6116d_addr, SIO_LDSEL);
			superio_outb(_nct6116d_addr, SIO_GPIO4_DIR, 0xff);
			
			/* done */
			superio_exit(_nct6116d_addr);
		}
	}
	for (i = 0; i < ARRAY_SIZE(epcu2217_led_profile); i++)
		led_classdev_unregister(&epcu2217_led->pled[i].cdev);
	
	platform_device_unregister(epcu2217_led->pdev);
	platform_driver_unregister(&epcu2217_led_driver);
}

module_init(epcu2217_led_init);
module_exit(epcu2217_led_exit);

MODULE_AUTHOR("Gerd Pauli");
MODULE_DESCRIPTION("Advantec epcu 2217 gpio LED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:leds_epcu2217");
