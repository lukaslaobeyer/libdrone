# Bebop Navdata server

This little program will run on your Bebop drone and retransmit the full navigation data at higher frequencies.

## Automatic installation

It is recommended to run the automated installer. Make sure you have Bebop firmware version 3.2 or later.

### Linux

Just ``cd`` into this directory and run ``./install.sh``.
Then reboot your drone and it should be done.

### Windows

Activate telnet if it is not already installed. Open an administrative command prompt:

```
dism /online /Enable-Feature /FeatureName:TelnetClient
```

Then open a new administrative command prompt and ``cd`` into this directory. Finally run ``.\install.bat``. Do not switch windows while this script runs.
Now reboot your drone and it should be done.

## Manual installation

Please read ``MANUAL_INSTALL.md``.
