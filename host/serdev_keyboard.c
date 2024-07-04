#include <linux/module.h>
#include <linux/init.h>
#include <linux/serdev.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/input.h>

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Victor Modamio");
MODULE_DESCRIPTION("Serial driver for keyboard");

#define NKBD_KEY	0x7f
#define NKBD_PRESS	0x80

static unsigned char nkbd_keycode[128] = {
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

struct nkbd {
	unsigned char keycode[128];
	struct input_dev *dev;
	char phys[25]; // "Typewriter_serial_keyboar"
};

static struct nkbd *nkbd;

/* Declate the probe and remove functions */
static int serdev_keyboard_probe(struct serdev_device *serdev);
static void serdev_keyboard_remove(struct serdev_device *serdev);

static struct of_device_id serdev_keyboard_ids[] = {
	{
		.compatible = "brightlight,typedev",
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, serdev_keyboard_ids);

static struct serdev_device_driver serdev_keyboard_driver = {
	.probe = serdev_keyboard_probe,
	.remove = serdev_keyboard_remove,
	.driver = {
		.name = "serdev-keyboard",
		.of_match_table = serdev_keyboard_ids,
	},
};

/**
 * @brief Callback is called whenever a character is received
 */
static size_t serdev_keyboard_recv(struct serdev_device *serdev, const unsigned char *buffer, size_t size) {
	for (int m=0; m<size; m++) {
	    input_report_key(nkbd->dev, buffer[m] & NKBD_KEY, buffer[m] & NKBD_PRESS);
	    input_sync(nkbd->dev);
	}
	printk("serdev_keyboard - Received %ld bytes with \"%s\"\n", size, buffer);
	return -ENODEV;
}

static const struct serdev_device_ops serdev_keyboard_ops = {
	.receive_buf = serdev_keyboard_recv,
};

/**
 * @brief This function is called on loading the driver 
 */
static int serdev_keyboard_probe(struct serdev_device *serdev) {
	int status;
	serdev_device_set_client_ops(serdev, &serdev_keyboard_ops);
	status = serdev_device_open(serdev);
	if(status) {
		printk("serdev_keyboard - Error opening serial port!\n");
		return -status;
	}

	serdev_device_set_baudrate(serdev, 115200);
	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	//////////////////////////////////////
	
	//struct nkbd *nkbd;
	struct input_dev *input_dev;
	int err = -ENOMEM;
	int i;

	nkbd = kzalloc(sizeof(struct nkbd), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!nkbd || !input_dev)
		goto fail1;

	//nkbd->serio = serio;
	nkbd->dev = input_dev;
	snprintf(nkbd->phys, sizeof(nkbd->phys), "Typewriter_serial_keyboar");
	memcpy(nkbd->keycode, nkbd_keycode, sizeof(nkbd->keycode));

	input_dev->name = "Newton Keyboard";
	//input_dev->phys = nkbd->phys;
	//input_dev->id.bustype = BUS_RS232;
	//input_dev->id.vendor = SERIO_NEWTON;
	//input_dev->id.product = 0x0001;
	//input_dev->id.version = 0x0100;
	//input_dev->dev.parent = &serio->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);
	input_dev->keycode = nkbd->keycode;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = ARRAY_SIZE(nkbd_keycode);
	for (i = 0; i < 128; i++)
		set_bit(nkbd->keycode[i], input_dev->keybit);
	clear_bit(0, input_dev->keybit);

	//serio_set_drvdata(serio, nkbd);

	//err = serio_open(serio, drv);
	//if (err)
	//	goto fail2;

	err = input_register_device(nkbd->dev);
	if (err)
		goto fail1;

	return 0;

 fail1:	input_free_device(input_dev);
	kfree(nkbd);
	return err;
	/////////////////////////////////////
}

/**
 * @brief This function is called on unloading the driver 
 */
static void serdev_keyboard_remove(struct serdev_device *serdev) {
	printk("serdev_keyboard - Now I am in the remove function\n");
	serdev_device_close(serdev);
}

/**
 * @brief This function is called, when the module is loaded into the kernel
 */
static int __init my_init(void) {
	printk("serdev_keyboard - Loading the driver...\n");
	if(serdev_device_driver_register(&serdev_keyboard_driver)) {
		printk("serdev_keyboard - Error! Could not load driver\n");
		return -1;
	}
	return 0;
}

/**
 * @brief This function is called, when the module is removed from the kernel
 */
static void __exit my_exit(void) {
	printk("serdev_keyboard - Unload driver");
	serdev_device_driver_unregister(&serdev_keyboard_driver);
	input_unregister_device(nkbd->dev);
	kfree(nkbd);
}

module_init(my_init);
module_exit(my_exit);


