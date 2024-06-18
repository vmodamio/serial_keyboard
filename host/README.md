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

