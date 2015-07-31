#!/bin/sh

ftp -n <<EOF
open 192.168.42.1
user user
mkdir internal_000/navdataserver
put src/install.sh internal_000/navdataserver/install.sh
put a.out internal_000/navdataserver/a.out
put src/startnavdataserver.sh internal_000/navdataserver/startnavdataserver.sh
EOF
