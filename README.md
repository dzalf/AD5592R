#AD5592R 

##8-Channel, 12-Bit, Configurable ADC/DAC with On-Chip Reference, SPI Interface

My take on the AD5592R control library originally developed by @Metaln00b.

Compared to Metaln00b's design, this library is based on a Class instead of a struct.

In terms of the functionalities, they are besically the same, however, the constructor takes a different
approach and the internals of the library do not use pointers.

Examples based on this implementation are provided.

Tested on Arduino and Teensy platforms.