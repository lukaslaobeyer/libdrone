#!/bin/sh

mount -o remount,rw /
chmod +x /data/ftp/internal_000/navdataserver/a.out
chmod +x /data/ftp/internal_000/navdataserver/startnavdataserver.sh
sed -i 's/BLACKBOX=0/BLACKBOX=1/' /etc/debug.conf
sed -i 's/"blackbox_enable" : false/"blackbox_enable" : true/' /data/dragon.conf
sed -i 's/DragonStarter\.sh -out2null &/DragonStarter\.sh -out2null \&\n\/data\/ftp\/internal_000\/navdataserver\/startnavdataserver\.sh \&/' /etc/init.d/rcS_mode_default
