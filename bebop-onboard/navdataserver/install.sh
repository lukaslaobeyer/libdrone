#!/bin/sh

echo "This will install the AutoFlight extensions for full navigation data on your Bebop drone."
echo "The script will connect to your drone, upload files and modify critical system configuration potentially bricking your drone."
echo "Last tested with Bebop firmware 3.2."
echo "WARNING:   THIS SCRIPT WILL NOT WORK WITH FIRMWARES OLDER THAN VERSION 3.X. Please follow the manual installation procedure for those."
echo "IMPORTANT: Please quickly press your Bebop's power button 4 times before continuing."
read -p "Are you sure you want to continue? (Y/N)" -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
    echo "Uploading files..."
    ./bash/upload.sh
    echo "Running onboard installation..."
    ./bash/runinstall.sh
    echo "Done. Please restart your drone now."
else
    echo "Abort."
fi
