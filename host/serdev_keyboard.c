#include <linux/module.h>
#include <linux/init.h>
#include <linux/serdev.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>

#include <linux/input.h>
#include <linux/slab.h>

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Victor Modamio");
MODULE_DESCRIPTION("Serial keyboard");


#define TYPEWRT_KEY	0x7f
#define TYPEWRT_PRESS	0x80

static unsigned char typewrt_keycode[128] = {
    KEY_ESC	, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, 
    KEY_8, KEY_9, KEY_0, KEY_MINUS, KEY_EQUAL, KEY_BACKSPACE, KEY_TAB, KEY_Q, 
    KEY_W, KEY_E, KEY_R, KEY_T, KEY_Y, KEY_U, KEY_I, KEY_O, 
    KEY_P, KEY_LEFTBRACE, KEY_RIGHTBRACE, KEY_ENTER, KEY_LEFTCTRL, KEY_A, KEY_S, KEY_D, 
    KEY_F, KEY_G, KEY_H, KEY_J, KEY_K, KEY_L, KEY_SEMICOLON, KEY_APOSTROPHE, 
    KEY_GRAVE, KEY_LEFTSHIFT, KEY_BACKSLASH, KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B, 
    KEY_N, KEY_M, KEY_COMMA, KEY_DOT, KEY_SLASH, KEY_RIGHTSHIFT, KEY_KPASTERISK, KEY_LEFTALT, 
    KEY_SPACE, KEY_CAPSLOCK, KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, 
    KEY_F7, KEY_F8, KEY_F9, KEY_F10, KEY_NUMLOCK, KEY_SCROLLLOCK, KEY_KP7, KEY_KP8, 
    KEY_KP9, KEY_KPMINUS, KEY_KP4, KEY_KP5, KEY_KP6, KEY_KPPLUS, KEY_KP1, KEY_KP2, 
    KEY_KP3, KEY_KP0, KEY_KPDOT, KEY_ZENKAKUHANKAKU, KEY_102ND, KEY_F11, KEY_F12, KEY_RO, 
    KEY_KATAKANA, KEY_HIRAGANA, KEY_HENKAN, KEY_KATAKANAHIRAGANA, KEY_MUHENKAN, KEY_KPJPCOMMA, KEY_KPENTER, KEY_RIGHTCTRL, 
    KEY_KPSLASH, KEY_SYSRQ, KEY_RIGHTALT, KEY_LINEFEED, KEY_HOME, KEY_UP, KEY_PAGEUP, KEY_LEFT, 
    KEY_RIGHT, KEY_END, KEY_DOWN, KEY_PAGEDOWN, KEY_INSERT, KEY_DELETE, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0, 0, 0 
};

struct typewrt {
	struct serdev_device *serdev;
	unsigned char keycode[128];
	struct input_dev *dev;
	struct serio *serio;
	char phys[32];
};


/* Declate the probe and remove functions */
static int typewrt_probe(struct serdev_device *serdev);
static void typewrt_remove(struct serdev_device *serdev);

static struct of_device_id typewrt_ids[] = {
	{
		.compatible = "typewriter_keyboard",
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, typewrt_ids);

static struct serdev_device_driver typewrt_driver = {
	.probe = typewrt_probe,
	.remove = typewrt_remove,
	.driver = {
		.name = "typewrt-keyboard",
		.of_match_table = typewrt_ids,
	},
};

/**
 * @brief Callback is called whenever a character is received
 */
static size_t typewrt_recv(struct serdev_device *serdev, const unsigned char *buffer, size_t size) {
	struct typewrt *typewrt = serdev_device_get_drvdata(serdev);
	//printk("typewrt - Received last %ld  byte \"%d\"\n", size,  buffer[size-1]);
	input_report_key(typewrt->dev, buffer[size-1] & TYPEWRT_KEY, buffer[size-1] & TYPEWRT_PRESS);
	input_sync(typewrt->dev);
        return 	size;
}

static const struct serdev_device_ops typewrt_ops = {
	.receive_buf = typewrt_recv,
};

/**
 * @brief This function is called on loading the driver 
 */
static int typewrt_probe(struct serdev_device *serdev) {
	struct device *dev = &serdev->dev;
	struct typewrt *typewrt_dev;
	typewrt_dev = devm_kzalloc(dev, sizeof(*typewrt_dev), GFP_KERNEL);
	if (!typewrt_dev)
		return -ENOMEM;

	typewrt_dev->serdev = serdev;

	printk("typewrt - Now I am in the probe function!\n");

	struct input_dev *input_dev;
	input_dev = input_allocate_device();
	if (!input_dev)
		goto fail1;
	typewrt_dev->dev = input_dev;
	//typewrt->serio = serio;
	memcpy(typewrt_dev->keycode, typewrt_keycode, sizeof(typewrt_dev->keycode));

	input_dev->name = "Typewrt Keyboard";
	//input_dev->phys = typewrt->phys;
	//input_dev->id.bustype = BUS_RS232;
	//input_dev->id.vendor = SERIO_NEWTON;
	//input_dev->id.product = 0x0001;
	//input_dev->id.version = 0x0100;
	//input_dev->dev.parent = &serio->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);
	input_dev->keycode = typewrt_dev->keycode;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = ARRAY_SIZE(typewrt_keycode);
	int i;
	for (i = 0; i < 128; i++)
		set_bit(typewrt_dev->keycode[i], input_dev->keybit);
	clear_bit(0, input_dev->keybit);

	int err = input_register_device(typewrt_dev->dev);
	if (err) 
		goto fail2;

	serdev_device_set_drvdata(serdev, typewrt_dev);

	serdev_device_set_client_ops(serdev, &typewrt_ops);
	int status = serdev_device_open(serdev);
	if(status) {
		printk("typewrt - Error opening serial port!\n");
		return -status;
	}

	serdev_device_set_baudrate(serdev, 500000);
	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);

	return 0;
 fail2:	serdev_device_close(serdev);
 fail1:	input_free_device(input_dev);
	//kfree(typewrt);
	return err;
}

/**
 * @brief This function is called on unloading the driver 
 */
static void typewrt_remove(struct serdev_device *serdev) {
	printk("typewrt - Now I am in the remove function\n");
	//struct typewrt *typewrt = serdev_device_get_drvdata(serdev);
	serdev_device_close(serdev);
}

/**
 * @brief This function is called, when the module is loaded into the kernel
 */
static int __init my_init(void) {
	printk("typewrt - Loading the driver...\n");
	if(serdev_device_driver_register(&typewrt_driver)) {
		printk("typewrt - Error! Could not load driver\n");
		return -1;
	}
	return 0;
}

/**
 * @brief This function is called, when the module is removed from the kernel
 */
static void __exit my_exit(void) {
	printk("typewrt - Unload driver");
	serdev_device_driver_unregister(&typewrt_driver);
}

module_init(my_init);
module_exit(my_exit);

