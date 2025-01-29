# PicoHM01B0

Arduino library that uses the PIO and DMA on an RP2040 to get a good frame rate from a HM01B0 camera

## TODO

the frame buffer needs to be aligned to 4 bytes, to allow 32 bit DMA.
\_\_attribute\_\_((aligned(4)))



table with resolutions:

| binning_2x2 | qvga_mode | width | height |
| ----- | ----- | ----- | ----- |
| false | false | 324 | 324 |
| false | true | 324 | 244 |
| true | false | 164 | 162 |
| true | true | 164 | 122 |



exposure auto

insert table with fields, and a column explaining which are mandatory and default values

```
	uint i2c_dat_gpio;  // i2c bus pins
	uint i2c_clk_gpio;
	uint vsync_gpio;
	// D0 pin: in 4 bit mode, D1, D2, D3 must be consecutive after this one
	uint d0_gpio;
	uint pclk_gpio;

	// MCLK pin, -1 means MCLK is provided by the circuit, not the mcu
	int mclk_gpio;
	// MCLK frequency in Hz: must be set only if MCLK is provided externally
	uint mclk_freq;

	// target frame rate in frames per second: the code will try to match
	// this as best as possible within the clock limitations. It can change
	// the horizontal and vertical blanking times and also the MCLK if it is
	// controlled by the mcu
	float frame_rate;

	// image resolution controls:
	// drop 80 lines to get 324x244 (or half in binning_2x2 is true)
	bool qvga_mode;
	// half resolution on both axis for 164x162 (or 164x122 in qvga_mode)
	bool binning_2x2;

	// bus width: true for a 4 bit bus, false for 1 bit
	bool bus_4bit; //TODO

	// image orientation
	bool flip_horizontal, flip_vertical;
```
