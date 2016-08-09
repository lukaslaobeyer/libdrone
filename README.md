## libdrone

This small C++ library aims to provide a common interface to multirotor drones like the Parrot AR.Drone 2.0 or the Parrot Bebop.
Most of the AR.Drone 2.0 communication code is taken directly from [AutoFlight](http://electronics.kitchen/autoflight) as an aim to decouple drone communications functionality from AutoFlight and provide an easy way add support for other devices.

### Supported Drones
Currently support is planned for the Parrot AR.Drone 2.0 and Bebop.
* [AR.Drone 2.0](http://ardrone2.parrot.com)
* [Bebop drone](http://www.parrot.com/usa/products/bebop-drone/)

### Onboard extensions

The Bebop drone does not send navigation data at any usable frequency (5 Hz is way too low for autonomous navigation). Therefore, a very simple navdata server was written that runs on the Bebop drone and retransmits the full navigation data as used internally by the drone at higher frequencies. See ``bebop-onboard/navdataserver`` for more information.

### Building the library on Ubuntu 16.04

First, make sure all dependencies are installed:

```
apt-get install -y unzip build-essential cmake git python3-dev python3-numpy software-properties-common
apt-get install -y libboost-all-dev
apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev
apt-get install -y libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavresample-dev
apt-get install -y libeigen3-dev
apt-get install -y qtbase5-dev qt5-default libqt5webkit5-dev qtmultimedia5-dev # Optional, needed for OpenCV Qt 5 support
```

You will also need OpenCV 3.1. Get it [here](http://opencv.org/downloads.html), and unzip it somewhere. In that directory, do:

```
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_QT=ON -D WITH_OPENGL=ON .. # For WITH_QT=ON you will need to have installed the Qt 5 packages
make -j4
make install
```

Now, go into the directory you have the ``libdrone`` sources in, and build it:

```
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release ..
make -j4
make install
```

### Using the library

Work in progress!
