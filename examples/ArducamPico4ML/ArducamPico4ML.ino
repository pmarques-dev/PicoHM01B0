#include <PicoHM01B0.h>


PicoHM01B0 Camera;


void setup()
{
    Camera.begin(4, 5, 16, 6, 14, 3);
}

static uint8_t frame[324][324];

void loop()
{
    Camera.start_capture(frame[0]);
    Camera.wait_for_frame();
    Serial.writeln("catured frame");
}
