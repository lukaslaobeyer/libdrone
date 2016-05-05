# Installing the Bebop navdata server

This program will retransmit the full navigation data logged to the black box at 200Hz in real-time.

Cross-compile ``navdataserver.c`` with ``./compile.sh``. You need the ``gcc-arm-linux-gnueabi`` toolchain.

## 1. Telnet into the Bebop drone

The Bebop has a telnet server. You automatically login as root.

```
telnet 192.168.42.1
```

For Bebop firmware versions 3.X, activate writing to filesystem:
```
mount -o remount,rw /
```

## 2. Activate debug mode

``/etc/debug.conf``: Change ``BLACKBOX=0`` to ``BLACKBOX=1``

``/data/dragon.conf``: Change ``"blackbox_enable" : false`` to ``"blackbox_enable" : true``

## 3. Upload the navdata server

```
cd /data/ftp/internal_000/
mkdir navdataserver
```

Now copy ``a.out`` and ``startnavdataserver.sh`` into the ``/data/ftp/internal_000/navdataserver`` directory using FTP.

Mark both as executable:
```
chmod +x /data/ftp/internal_000/navdataserver/a.out
chmod +x /data/ftp/internal_000/navdataserver/startnavdataserver.sh
```

## 4. Autostart on Bebop boot

edit ``/etc/init.d/rcS`` for Bebop firmwares older than 3.X
edit ``/etc/init.d/rcS_mode_default``

Insert "/data/ftp/internal_000/navdataserver/startnavdataserver.sh &" right before the "sleep 1" at the end of the file.

## 5. Reboot

Done.
