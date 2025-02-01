# PicoHM01B0

The PicoHM01B0 is an arduino library that uses the PIO and DMA on an RP2040 to get a good frame rate from a HM01B0 camera, while using very little cpu time.

Using a Arducam Pico4ML as reference hardware, even though it uses only one data bit transfers, it is able to achieve 110Hz at 162x162 or 30Hz at 324x324. With a 4 bit bus the frame rate limits are the ones from the camera chip.


## API

```cpp
int begin(const PicoHM01B0_config &config);
```
Call begin to initialize the camera chip. Returns 1 on success, 0 on error. This must be the first call made before any other methods can be called. The PicoHM01B0_config structure has several fields:

|field|definition|
|-----|----------|
|i2c_dat_gpio|I2C data pin, aka SDA|
|i2c_clk_gpio|I2C clock pin, aka SCL|
|vsync_gpio|vertical sync, aka frame valid pin, aka FVLD|
|d0_gpio|D0 pin: in 4 bit mode, D1, D2, D3 must be consecutive after this one|
|pclk_gpio|pixel clock pin, aka PCLKO|
|mclk_gpio|MCLK pin or -1 if the MCLK is provided by the circuit and not the mcu. In that case mclk_freq must be set to the frequency in Hz that the circuit is using|
|mclk_freq|MCLK frequency in Hz. Only has to be set if mclk_pin is -1 (see above)|
|bus_4bit|bus width: true for a 4 bit bus, false for 1 bit|
|flip_horizontal|set to true to flip image horizontally|
|flip_vertical|set to true to flip image vertically|

NOTE: all pin definitions use the actual gpio number on the RP2040 and ignores arduino translations of gpio to pin.

<br>

```cpp
void start_streaming(float frame_rate, bool binning_2x2, bool qvga_mode);
```
Called after begin, to specify resolution and frame rate (in frames per second) and then start streaming. The code will optimize vertical and horizontal blanking times to get the closest frame rate as possible to the requested one.

The various resolutions that the chip supports are:

| binning_2x2 | qvga_mode | width | height |
| ----- | ----- | ----- | ----- |
| false | false | 324 | 324 |
| false | true | 324 | 244 |
| true | false | 164 | 162 |
| true | true | 164 | 122 |

Note that using 2x2 binning in qvga mode will reduce the size of the frame, but apparently not the time it takes to process the frame, so it doesn't actually increase the frame rate.

<br>

```cpp
void start_capture(uint8_t *dest);
```
Start capturing a frame. The function returns immediately, but the PIO code waits for the vertical sync to start capturing and then uses DMA to do the actual transfer, so the mcu is free to run other code while the frame is being transferred.

If the code is in a loop calling wait_for_frame, do some processing and call start_capture again, if the processing takes less time than the vertical blanking time, the full frame rate is achieved. Otherwise, if by the time start_capture is called the frame has already started, it will wait for the next frame, thus dropping a frame, effectively reducing the frame rate.

Alternatively, if there are 2 buffers, the code can alternate between them and as soon as wait_for_frame returns for buffer 0 start capture can be immediately called for buffer 1 while buffer 0 is being processed.

| :warning: WARNING           |
|:----------------------------|
| The buffer must be aligned to 4 bytes, as the code uses 32 bit DMA to improve bus efficiency. At the highest frame rates it uses less than one transfer every 128 system clocks, which is very lightweight on the bus. An example declaration for an aligned frame buffer is: <br> `uint8_t frame_buffer[HEIGHT][WIDTH] __attribute__((aligned(4)));`  |

<br>

```cpp
bool is_frame_ready(void);
```
Non-blocking call to check if the frame that is being currently transferred is already finished. When it returns true, the frame is ready and can be accessed immediately. No other operation is needed before calling `start_capture` again.

<br>

```cpp
void wait_for_frame(void);
```
Blocks execution, waiting for the completion of a previous call to `start_capture`.

<br>

```cpp
void stop_streaming(void);
```
Stop streaming and change the camera chip state to s/w standby. After calling this method, start_streaming can be called again, including with a different frame rate and/or resolution.

<br>

```cpp
void set_fixed_exposure(float exposure_ms, int d_gain, int a_gain);
```
Set exposure parameters, can only be called after start_streaming, as the exposure settings depends on the frame rate and resolution.
The range of the parameters is as follows:
 - digital gain: 1..255
 - analog gain: 0..7

<br>

```cpp
void set_auto_exposure(void);
```
Select auto exposure. The camera chip will automatically adjust the exposure to achieve some target luminosity average in the image.

A note on exposure and avoiding 50/60Hz flicker: there are basically two ways to avoid electrical grid frequency flicker:
- use an exposure time that is a multiple of half a cycle. At 50Hz, that means a multiple of 10ms. With this setting you can have any frame rate, but the exposure control will have a very coarse granularity. The auto exposure settings on the chip have support for this, but they are not exposed in the API, as they seem to coarse to be useful.
- use a frame rate that is an integer divisor of twice the frequency. At 50Hz this will be 100, 50, 33.333, 25, 20, 16.666,... This can be also thought of in terms of the frame time that must be a multiple of 10ms. By using one of these frame rates, the exposure can be fine tuned to any period between zero and the frame period, and auto-exposure will work well.


### Helper methods

There are a few helper methods that can be summarized in a table:

|function|description|
|--------|-----------|
|float get_actual_frame_rate_fps()|returns the actual frame rate in frames per second that the code was able to achieve (should be very close to the requested frame rate)|
|float get_transfer_period_ms()<br>float get_blanking_period_ms()|the frame time is split between the time it takes to transfer the frame data to the mcu and the vertical blanking time. These methods return each of these times in ms. The sum of both times should match the frame time, which is also `1000.0 / get_actual_frame_rate_fps()`|
|int get_cols()<br>int get_rows()|returns the horizontal and vertical resolution, that depend on the binning and qvga settings that were passed to `start_streaming` (and match the numbers on the table above)|
|int get_line_length()<br>int get_line_count()|these methods return the actual lengths set by the initialization code to achieve the frame rate requested. These are the pixel count plus blanking, bth horizontal and vertical. They should usually not be needed, except for debugging purposes|

## Simple example

```cpp
#include <PicoHM01B0.h>

PicoHM01B0 Camera;

uint8_t frame[324][324] __attribute__((aligned(4)));

void setup()
{
  // set up the initial configuration: pin numbers, orientation
  PicoHM01B0_config config;
  config.i2c_dat_gpio = 4;
  config.i2c_clk_gpio = 5;
  config.vsync_gpio = 16;
  config.d0_gpio = 6;
  config.pclk_gpio = 14;
  config.mclk_gpio = 3;
  config.bus_4bit = false;
  config.flip_vertical = true;
  config.flip_horizontal = true;

  Camera.begin(config);

  // start streaming at 25Hz, full resolution
  Camera.start_streaming(25, false, false);

  // immediately start the capture of the first frame
  Camera.start_capture(frame[0]);
}

void loop()
{
  Camera.wait_for_frame();

  // do processing here that requires the frame to not be overwritten
  // ...

  Camera.start_capture(frame[0]);

  // do more processing here that doesn't need the frame buffer any more
  // ...
}
```
