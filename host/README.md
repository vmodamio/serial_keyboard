The serial keyboard is attached to the host (linux) with
inputattach as a modified newton keyboard.

On Arch linux there is a system service for inputattach:
systemctl start inputattach.service

```
$ sudo inputattach --baud 115200 --always --noinit --newtonkbd  /dev/ttyACM0
```

Inputattach loads the kernel module newtonkbd.ko.zst,

This is the newtonkdb code modified so it parses the readed byte directly
as a linux input code, instead of using a map.

1) Compile it as kernel module
2) Compress it with zstd newtonkbd.ko to get the newtonkbd.ko.zst
3) Move it to /usr/lib/modules/($uname -r)/kernel/drivers/input/keyboard/
4) Then run: sudo depmod  (if the module is new, to recreate the 
   module dependency list.


Note: connecting the keyboard MCU through the USB is easy, as it will appear
in the host as ttyUSB or something similar. But when the MCU is wired to other
UART pins in the board, it is not straight forward how the host would recognize
the serial port.

```
# cat /proc/tty/driver/serial
```
shows what is connected to the ttyS0 to S3 that corresponds to COM1 to COM4.


