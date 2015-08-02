@echo off
echo user user> ftpcmd.dat
echo bin>> ftpcmd.dat
echo mkdir internal_000/navdataserver>> ftpcmd.dat
echo put src/install.sh internal_000/navdataserver/install.sh>> ftpcmd.dat
echo put a.out internal_000/navdataserver/a.out>> ftpcmd.dat
echo put src/startnavdataserver.sh internal_000/navdataserver/startnavdataserver.sh>> ftpcmd.dat
echo quit>> ftpcmd.dat
ftp -n -s:ftpcmd.dat 192.168.42.1
del ftpcmd.dat
