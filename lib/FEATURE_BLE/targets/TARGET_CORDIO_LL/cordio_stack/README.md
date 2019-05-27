# BLE API Cordio Link Layer Implementation

The BLE API Cordio link layer implementation allows Cordio licensee to easily
deliver a complete and up to date implementation of mbed BLE to their customers
using mbed OS.

The library a consists of the controller HCI, Bluetooth 5 compliant link layer
protocol core, scheduler, baseband porting layer and a portable software
foundation.

To deliver a BLE port, vendors simply have to provide an HCI driver tailored
for the BLE module present on the board they want to support.

## Source Organization

The root contains the binary distribution `libcordio_stack.a` of the library and
the folders contain the public headers to interface with it.

* `controller`: HCI Controller headers
* `platform`: Platform headers


## Library information

Compiled with: GNU Arm Embedded Toolchain 6-2017-q2-update
