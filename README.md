# PicoHM01B0

Arduino library that uses the PIO and DMA on an RP2040 to get a good frame rate from a HM01B0 camera

## TODO

the frame buffer needs to be aligned to 4 bytes, to allow 32 bit DMA.
__attribute__((aligned(4)))

