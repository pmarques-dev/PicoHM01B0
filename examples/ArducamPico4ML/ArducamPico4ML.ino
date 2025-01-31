#include <PicoHM01B0.h>

// this example code works on an Arducam Pico4ML board

PicoHM01B0 Camera;

uint8_t buffer[324][324] __attribute__((aligned(4)));

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
  Camera.start_capture(buffer[0]);
}

void loop()
{
  static uint last_stamp;
  uint m;

  // wait for the frame transfer to finish
  Camera.wait_for_frame();

  // do processing here that requires the frame to not be overwritten
  int total = 0;
  for (int y = 0; y < 324; y += 4)
    for (int x = 0; x < 324; x += 4)
      total += buffer[y][x];

  // as soon as the frame buffer is not required, start a new capture
  Camera.start_capture(buffer[0]);

  // do more processing or communications here that doen't need the
  // frame buffer any more
  Serial.print("total luminosity: ");
  Serial.print(total);

  m = micros();
  Serial.print(", time ");
  Serial.print(m - last_stamp);
  Serial.println(" us");
  last_stamp = m;
}
