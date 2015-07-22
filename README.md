## libdrone

This small C++ library aims to provide a common interface to multirotor drones like the Parrot AR.Drone 2.0 or the Parrot Bebop.
Most of the AR.Drone 2.0 communication code is taken directly from [AutoFlight](http://electronics.kitchen/autoflight) as an aim to decouple drone communications functionality from AutoFlight and provide an easy way add support for other devices.

### Supported Drones
Currently support is planned for the Parrot AR.Drone 2.0 and Bebop.
* [AR.Drone 2.0](http://ardrone2.parrot.com)
* [Bebop drone](http://www.parrot.com/usa/products/bebop-drone/)

### Onboard extensions

The Bebop drone does not send navigation data at any usable frequency (5 Hz is way too low for autonomous navigation). Therefore, a very simple navdata server was written that runs on the Bebop drone and retransmits the full navigation data as used internally by the drone at higher frequencies. See ``bebop-onboard/navdataserver`` for more information.

### Using the library

Work in progress!
