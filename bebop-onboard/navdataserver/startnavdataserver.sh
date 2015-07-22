#!/bin/sh

# Wait for dragon-prog to finish doing its stuff
sleep 5

rm /data/ftp/internal_000/Debug/archive/debug_*.tar.lzo
/data/ftp/internal_000/navdataserver/a.out &
echo "Started navdata server"
