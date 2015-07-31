#!/bin/sh

echo "This will install the AutoFlight extensions for full navigation data on your Bebop drone."
echo "The script will connect to your drone, upload files and modify critical system configuration potentially bricking your drone."
echo "Written on 31/7/2015. Last tested with Bebop firmware 2.0.29."
read -p "Are you sure you want to continue? " -n 1 -r
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
