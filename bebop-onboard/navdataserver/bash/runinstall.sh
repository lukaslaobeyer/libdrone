#!/bin/sh
{ sleep 1; echo "chmod +x /data/ftp/internal_000/navdataserver/install.sh"; sleep 1; echo "/data/ftp/internal_000/navdataserver/install.sh"; sleep 1; } | telnet 192.168.42.1
