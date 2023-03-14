# AD5592R

## 8-Channel, 12-Bit, Configurable ADC/DAC with On-Chip Reference, SPI Interface

My take on the [AD5592R](https://www.analog.com/en/products/ad5592r.html) control library originally (and brilliantly :wink:) developed by [@Metaln00b](https://github.com/Metaln00b/Arduino-libad5592r)

Compared to Metaln00b's design, my library is based on a Class instead of a struct. Additionally, headers are unified for better readability.

In terms of the functionalities, they are besically the same, however, the constructor takes a different approach and the internals of the library do not use pointers.

Examples based on (or ported to) this implementation are provided.

Tested on Arduino and Teensy platforms.

#TODO:

- Invite @Metaln00b as a collaborator just to get _their_ blessing/approval
- Translate German comments to English (if @Metaln00b agrees)
- Make Tindie page for my little Dev Boards (this was a surprise I have been working on :wink:)
