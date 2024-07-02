// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (c) 2000 Justin Cormack
 */

/*
 * Newton keyboard driver for Linux
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/serio.h>

#define DRIVER_DESC	"Newton keyboard driver"

MODULE_AUTHOR("Justin Cormack <j.cormack@doc.ic.ac.uk>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#define NKBD_KEY	0x7f
#define NKBD_PRESS	0x80

/* Even though the nkbd_keycode is not used any longer (the keyboard itself is sending already the right keycode for linux)
 * the array needs to be kept, and needs to contain all the keycodes needed...because somewhere in the code later, the 
 * module sends the possible keys that can be used from that array... including KEY_ESC in the array made possible to
 * send the ESC from the keyboard....
 */
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
	struct serio *serio;
	char phys[32];
};

static irqreturn_t nkbd_interrupt(struct serio *serio,
		unsigned char data, unsigned int flags)
{
	struct nkbd *nkbd = serio_get_drvdata(serio);

	/* invalid scan codes are probably the init sequence, so we ignore them */
	//if (nkbd->keycode[data & NKBD_KEY]) {
		input_report_key(nkbd->dev, data & NKBD_KEY, data & NKBD_PRESS);
		input_sync(nkbd->dev);
	//}

	//else if (data == 0xe7) /* end of init sequence */
	//	printk(KERN_INFO "input: %s on %s\n", nkbd->dev->name, serio->phys);
	return IRQ_HANDLED;

}

static int nkbd_connect(struct serio *serio, struct serio_driver *drv)
{
	struct nkbd *nkbd;
	struct input_dev *input_dev;
	int err = -ENOMEM;
	int i;

	nkbd = kzalloc(sizeof(struct nkbd), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!nkbd || !input_dev)
		goto fail1;

	nkbd->serio = serio;
	nkbd->dev = input_dev;
	snprintf(nkbd->phys, sizeof(nkbd->phys), "%s/input0", serio->phys);
	memcpy(nkbd->keycode, nkbd_keycode, sizeof(nkbd->keycode));

	input_dev->name = "Newton Keyboard";
	input_dev->phys = nkbd->phys;
	input_dev->id.bustype = BUS_RS232;
	input_dev->id.vendor = SERIO_NEWTON;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &serio->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);
	input_dev->keycode = nkbd->keycode;
	input_dev->keycodesize = sizeof(unsigned char);
	input_dev->keycodemax = ARRAY_SIZE(nkbd_keycode);
	for (i = 0; i < 128; i++)
		set_bit(nkbd->keycode[i], input_dev->keybit);
	clear_bit(0, input_dev->keybit);

	serio_set_drvdata(serio, nkbd);

	err = serio_open(serio, drv);
	if (err)
		goto fail2;

	err = input_register_device(nkbd->dev);
	if (err)
		goto fail3;

	return 0;

 fail3:	serio_close(serio);
 fail2:	serio_set_drvdata(serio, NULL);
 fail1:	input_free_device(input_dev);
	kfree(nkbd);
	return err;
}

static void nkbd_disconnect(struct serio *serio)
{
	struct nkbd *nkbd = serio_get_drvdata(serio);

	serio_close(serio);
	serio_set_drvdata(serio, NULL);
	input_unregister_device(nkbd->dev);
	kfree(nkbd);
}

static const struct serio_device_id nkbd_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_NEWTON,
		.id	= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ 0 }
};

MODULE_DEVICE_TABLE(serio, nkbd_serio_ids);

static struct serio_driver nkbd_drv = {
	.driver		= {
		.name	= "newtonkbd",
	},
	.description	= DRIVER_DESC,
	.id_table	= nkbd_serio_ids,
	.interrupt	= nkbd_interrupt,
	.connect	= nkbd_connect,
	.disconnect	= nkbd_disconnect,
};

module_serio_driver(nkbd_drv);
