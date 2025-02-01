#ifndef PicoHM01B0_H
#define PicoHM01B0_H

#include <Arduino.h>

#ifdef ARDUINO_ARCH_RP2040

#include <string.h>
#include "hardware/pio.h"


struct sensor_reg;	// forward declaration

class PicoHM01B0_config {
public:
	// pin definitions
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

	// bus width: true for a 4 bit bus, false for 1 bit
	bool bus_4bit;

	// image orientation
	bool flip_horizontal, flip_vertical;

	PicoHM01B0_config() {
		// since this class is actually a POD, we can do this to give a
		// zero default to all fields in one go
		memset(this, 0, sizeof(*this));
	}
};


class PicoHM01B0 {
public:
	// call begin to initialize the camera chip. Returns 1 on success, 0 on
	// error
	int begin(const PicoHM01B0_config &config);

	// called after begin, to specify resolution and frame rate and start
	// streaming. If none of the exposure methods is called, the default is
	// auto exposure
	void start_streaming(float frame_rate, bool binning_2x2, bool qvga_mode);

	// start capturing a frame. The function returns immediately, but the
	// PIO code waits for the vertical sync to start capturing and then uses
	// DMA to do the actual transfer, so the mcu is free to run other code
	// while the frame is being transferred. If the code is in a loop
	// calling wait_for_frame, do some processing and call start_capture
	// again, if the processing takes less time than the vertical blanking
	// time, the full frame rate is achieved. Alternatively, if there are 2
	// buffers, the code can alternate between them and as soon as
	// wait_for_frame returns for buffer 0 start capture can be immediately
	// called for buffer 1 while buffer 0 is being processed.
	void start_capture(uint8_t *dest);

	// non-blocking call to check if the frame that is being currently
	// transferred is already finished
	bool is_frame_ready(void);

	// blocks waiting for the completion of a previous call to start_capture
	void wait_for_frame(void);

	// stop streaming, move the camera chip state to s/w standby
	void stop_streaming(void);

	// set exposure parameters, can only be called after start_streaming.
	// The range of the parameters is as follows:
	//  - digital gain: 1..255
	//  - analog gain: 0..7
	void set_fixed_exposure(float exposure_ms, int d_gain, int a_gain);

	// select auto exposure, can only be called after start_streaming
	void set_auto_exposure(void);


	float get_actual_frame_rate_fps(void) const {
		return actual_frame_rate;
	}
	float get_transfer_period_ms(void) const {
		return 1000.0 * (line_length * get_rows()) / (config.mclk_freq / 8);
	}
	float get_blanking_period_ms(void) const {
		return 1000.0 * (line_length * (line_count - get_rows())) / (config.mclk_freq / 8);
	}

	// these methods reflect the resolution set by the qvga / binning flags
	int get_cols(void) const {
		return binning_2x2 ? 164 : 324;
	}
	int get_rows(void) const {
		int ret = qvga_mode ? 244 : 324;
		return binning_2x2 ? ret / 2 : ret;
	}

	// these methods return the actual lengths set by the initialization
	// code to achieve the frame rate requested (and the actual rate). They
	// should usually not be needed, except for debug purposes
	int get_line_length(void) const {
		return line_length;
	}
	int get_line_count(void) const {
		return line_count;
	}

	// the constructor only initializes the "state" field
	PicoHM01B0() {
		state = STATE_RESET;
	}

private:
	enum state_t {
		STATE_RESET = 0,
		STATE_BEGIN = 1,
		STATE_STREAMING = 2,
		STATE_CAPTURING = 3,
	};

	// variable that keeps track of the current state
	state_t state;

	// current active configuration
	PicoHM01B0_config config;

	// target frame rate in frames per second: the code will try to match
	// this as best as possible within the clock limitations. It can change
	// the horizontal and vertical blanking times and also the MCLK if it is
	// controlled by the mcu
	float frame_rate;
	float actual_frame_rate;

	// internal clock divisor selected
	int clock_div;

	// image resolution controls:
	// drop 80 lines to get 324x244 (or half in binning_2x2 is true)
	bool qvga_mode;
	// half resolution on both axis for 164x162 (or 164x122 in qvga_mode)
	bool binning_2x2;

	// keep track of current exposure settings to avoid wasting time setting
	// registers that don't need to be set
	bool exp_auto;
	int exp_lines, exp_analog_gain, exp_digital_gain;

	// RP2040 resources
	PIO data_pio;
	uint data_pio_sm;
	uint data_pio_offset;
	uint dma_channel;

	PIO clock_pio;
	uint clock_pio_sm;
	uint clock_pio_offset;

	// line length and count actually used, set by calc_optimal_length. Can
	// be useful for debugging or exposure settings
	uint line_length, line_count;

	// this method computes the settings for line length and line count to
	// achieve the closest frame rate to the one requested
	void calc_optimal_length(void);

	// set config.mclk_freq and clock divisor vars
	void set_clock_vars(void);

	// the camera code only uses i2c for initialization, se we just do a
	// bit-bang i2c to allow full flexibility in the pin selection
	void i2c_bus_start(void);
	void i2c_bus_stop(void);
	void i2c_bus_send_ack(void);
	int i2c_bus_write_byte(int data);
	int i2c_write_reg(int regID, int regDat);

	void arducam_regs_write(const sensor_reg *camera_regs, int count);
};

#else // ARCH
#error PicoEncoder library requires a PIO peripheral and only works on the RP2040 architecture
#endif

#endif
