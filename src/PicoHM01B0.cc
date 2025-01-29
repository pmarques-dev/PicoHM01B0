#include <stdint.h>
#include <stdlib.h>

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/pio.h>
#include <hardware/i2c.h>
#include <pico/time.h>

#include <hardware/i2c.h>

#include "PicoHM01B0.h"


// ----------------------------
//     clock PIO

#define clock_wrap_target 0
#define clock_wrap 1

static const uint16_t clock_program_instructions[] = {
		//     .wrap_target
	0xe001, //  0: set    pins, 1
	0xe000, //  1: set    pins, 0
		//     .wrap
};

static const struct pio_program clock_program = {
	.instructions = clock_program_instructions,
	.length = 2,
	.origin = -1,
};

static inline pio_sm_config clock_program_get_default_config(uint offset)
{
	pio_sm_config c = pio_get_default_sm_config();
	sm_config_set_wrap(&c, offset + clock_wrap_target, offset + clock_wrap);
	return c;
}

static void clock_program_init(PIO pio, uint sm, uint offset, uint pin_base)
{
	pio_sm_set_consecutive_pindirs(pio, sm, pin_base, 1, true);
	pio_gpio_init(pio, pin_base);
	pio_sm_config c = clock_program_get_default_config(offset);
	sm_config_set_set_pins(&c, pin_base, 1);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
	sm_config_set_clkdiv(&c, 2);
	pio_sm_init(pio, sm, offset, &c);
	pio_sm_set_enabled(pio, sm, true);
}


// ------------------------------
//     image PIO

#define image_wrap_target 0
#define image_wrap 2

static uint16_t image_program_instructions[] = {
		//     .wrap_target
	0x208E, //  0: wait   1 gpio, 14
	0x4001, //  1: in     pins, 1
	0x200E, //  2: wait   0 gpio, 14
		//     .wrap
};

static const struct pio_program image_program = {
	.instructions = image_program_instructions,
	.length = 3,
	.origin = -1,
};

static inline pio_sm_config image_program_get_default_config(uint offset)
{
	pio_sm_config c = pio_get_default_sm_config();
	sm_config_set_wrap(&c, offset + image_wrap_target, offset + image_wrap);
	return c;
}

static uint image_program_init(PIO *pio, uint *sm, uint *offset, uint pin_d0, int pin_pclk)
{
	// to set the PCLK pin independently of the d0 pin, we need to
	// dynamically re-write the PIO code
	image_program_instructions[0] = 0x2080 | pin_pclk;
	image_program_instructions[2] = 0x2000 | pin_pclk;

	// we need to use a new program for each instance, because the code is
	// configured specifically for the PCLK pin passed
	if (!pio_claim_free_sm_and_add_program(&image_program, pio, sm, offset))
		return 0;

	pio_gpio_init(*pio, pin_d0);
	pio_sm_set_consecutive_pindirs(*pio, *sm, pin_d0, 1, false);
	pio_gpio_init(*pio, pin_pclk);
	pio_sm_set_consecutive_pindirs(*pio, *sm, pin_pclk, 1, false);

	pio_sm_config c = image_program_get_default_config(*offset);
	sm_config_set_in_pins(&c, pin_d0);
	sm_config_set_in_shift(&c, true, true, 32);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
	pio_sm_init(*pio, *sm, *offset, &c);

	return 1;
}


// -----------------------------------------------------------------------------------
//     bit banging i2c code

static const int sensor_address = 0x24;

static void small_pause(void)
{
	sleep_us(1);
}

static void slow_gpio_put(int pin, int value)
{
	gpio_put(pin, value);
	small_pause();
}

void PicoHM01B0::i2c_bus_start(void)
{
	slow_gpio_put(config.i2c_dat_gpio, 1);
	slow_gpio_put(config.i2c_clk_gpio, 1);
	slow_gpio_put(config.i2c_dat_gpio, 0);
	slow_gpio_put(config.i2c_clk_gpio, 0);
}

void PicoHM01B0::i2c_bus_stop(void)
{
	slow_gpio_put(config.i2c_dat_gpio, 0);
	slow_gpio_put(config.i2c_clk_gpio, 1);
	slow_gpio_put(config.i2c_dat_gpio, 1);
}

void PicoHM01B0::i2c_bus_send_ack(void)
{
	slow_gpio_put(config.i2c_dat_gpio, 0);
	slow_gpio_put(config.i2c_clk_gpio, 0);
	slow_gpio_put(config.i2c_clk_gpio, 1);
	slow_gpio_put(config.i2c_clk_gpio, 0);
	slow_gpio_put(config.i2c_dat_gpio, 0);
}

int PicoHM01B0::i2c_bus_write_byte(int data)
{
	int i, tem;

	for (i = 0; i < 8; i++) {
		slow_gpio_put(config.i2c_dat_gpio, ((data << i) & 0x80) != 0);
		slow_gpio_put(config.i2c_clk_gpio, 1);
		slow_gpio_put(config.i2c_clk_gpio, 0);
	}

	gpio_set_dir(config.i2c_dat_gpio, GPIO_IN);
	small_pause();
	slow_gpio_put(config.i2c_clk_gpio, 1);
	tem = !gpio_get(config.i2c_dat_gpio);
	slow_gpio_put(config.i2c_clk_gpio, 0);
	gpio_set_dir(config.i2c_dat_gpio, GPIO_OUT);
	return tem;
}

int PicoHM01B0::i2c_write_reg(int regID, int regDat)
{
	i2c_bus_start();
	if (i2c_bus_write_byte(sensor_address << 1) == 0)
		return 0;
	sleep_us(10);
	if (i2c_bus_write_byte(regID >> 8) == 0)
		return 0;
	sleep_us(10);
	if (i2c_bus_write_byte(regID) == 0)
		return 0;
	sleep_us(10);
	if (i2c_bus_write_byte(regDat) == 0)
		return 0;
	i2c_bus_stop();
	return 1;
}

// -----------------------------------------------------------------------------------
//     camera code

void PicoHM01B0::calc_optimal_length(void)
{
	int min_line_length, min_line_count;
	int clocks_per_frame, max_line_count;
	int best_diff, best_lc, best_ll;

	// frame period = (cam_line_count - v_advance (=1)) * (cam_line_length - h_advance (=2)) * (1/33MHz)

	if (binning_2x2) {
		min_line_length = 215;
		//min_line_count = qvga_mode ? 128 : 172;  // 172 not in datasheet, measured...
		min_line_count = qvga_mode ? 172 : 172;  // 172 not in datasheet, measured...
	} else {
		min_line_length = 376;	// min measured 369
		min_line_count = qvga_mode ? 260 : 344; // min measured 340
		//min_line_length = 369;	// min measured 369
		//min_line_count = qvga_mode ? 260 : 340; // min measured 340
	}

	// if we have a MCLK pin, it means we are generating the clock and it's
	// 1/4 of the main clock
	if (config.mclk_gpio >= 0)
		config.mclk_freq = clock_get_hz(clk_sys) / 4;

	clocks_per_frame = (config.mclk_freq / 8) / frame_rate;

	max_line_count = (clocks_per_frame + min_line_length - 1) / min_line_length;

	best_diff = 0x10000000;
	best_lc = min_line_count;
	best_ll = min_line_length;

	// loop in reverse so that if we have two identical solutions, we pick
	// the one with higher line count, that gives better exposure
	// granularity
	for (int lc = max_line_count; lc >= min_line_count; lc--) {
		int ll_round_down = clocks_per_frame / lc;
		for (int ll = ll_round_down; ll <= ll_round_down + 1; ll++) {
			if (ll < min_line_length)
				continue;
			int cpf = lc * ll;
			int diff = abs(cpf - clocks_per_frame);
			if (diff < best_diff) {
				best_diff = diff;
				best_lc = lc;
				best_ll = ll;
			}
		}
	}

	line_length = best_ll;
	line_count = best_lc;

	actual_frame_rate = (float) (config.mclk_freq / 8) / (best_ll * best_lc);
}


struct sensor_reg {
	uint16_t reg;
	uint8_t val;
};

static const sensor_reg camera_reset_regs[] = {
	{ 0x0103, 0x00 }, // reset
	{ 0x0100, 0x00 }, // mode: 000 - standby

	{ 0x0101, 0x00 }, // image orientation: 3: flip vertical and horizontal

	// {  0x202 ... 0x20F: exposure control

	{ 0x0350, 0x7F }, // ???

	// { 0x1012, 0x00 },	// 3: sync shift disable

	{ 0x1000, 0x43 }, // black level enable
	{ 0x1001, 0x40 }, // ???
	{ 0x1002, 0x32 }, // ???
	{ 0x1003, 0x08 }, // black level (default 0x20)
	{ 0x1006, 0x01 }, // BLI enable
	{ 0x1007, 0x08 }, // black level 2 (must be the same as black level)
	{ 0x1008, 0x00 }, // dpc control: dpc off, boundary bypass disable
	{ 0x1009, 0xA0 }, // cluster hot pixel threshold
	{ 0x100A, 0x60 }, // cluster cold pixel threshold
	{ 0x100B, 0x90 }, // single hot pixel threshold	(default 0xFF)
	{ 0x100C, 0x40 }, // single cold pixel threshold	(default 0xFF)

	// motion detection ROI
	{ 0x2000, 0x05 }, // motion detection disable, AE stat enable, avg 16 frame

	// automatic exposure
	{ 0x2100, 0x01 }, // AE enable
	{ 0x2101, 0x5F }, // AE target mean (default 0x3C)
	{ 0x2102, 0x0A }, // AE minimum mean (default 0x0A)
	{ 0x2103, 0x03 }, // converge in threshold (default 0x03)
	{ 0x2104, 0x05 }, // converge out threshold (default 0x05)
	{ 0x2107, 0x02 }, // Minimum INTG (default 0x02)
	{ 0x2108, 0x03 }, // Maximum Analog gain in full frame mode (default 0x03)
	{ 0x2109, 0x03 }, // Maximum Analog gain in BIN2 frame mode (default 0x04)
	{ 0x210A, 0x00 }, // Minimum Analog gain (default 0x00)
	{ 0x210B, 0x80 }, // Maximum digital gain (default 0xC0)
	{ 0x210C, 0x40 }, // Minimum digital gain (default 0x40)
	{ 0x210D, 0x20 }, // Damping factor (default 0x20)
	{ 0x210E, 0x00 }, // Flicker step control [0]:enable, [1] 0:50Hz, 1:60Hz (disabled)
	{ 0x210F, 0x00 }, // Flicker Step 60Hz parameter High Byte (default 0x00)
	{ 0x2110, 0x85 }, // Flicker Step 60Hz parameter Low Byte (default 0x3C)
	{ 0x2111, 0x00 }, // Flicker Step 50Hz parameter High Byte (default 0x00)
	{ 0x2112, 0x70 }, // Flicker Step 50Hz parameter Low Byte (default 0x32)
	// { 0x2113, 0x66 }, // Flicker Step hysteresis threshold (default 0x66)

	{ 0x2150, 0x02 }, // Motion detection control [0] enable

	{ 0x3011, 0x70 }, // 8-bit mode

	{ 0x3022, 0x01 }, // [0-4] ADVANCE_VSYNC	(default 2)

	{ 0x3044, 0x0A }, // ???
	{ 0x3045, 0x00 }, // ???
	{ 0x3047, 0x0A }, // ???
	{ 0x3050, 0xC0 }, // ???
	{ 0x3051, 0x42 }, // ???
	{ 0x3052, 0x50 }, // ???
	{ 0x3053, 0x00 }, // ???
	{ 0x3054, 0x03 }, // ???
	{ 0x3055, 0xF7 }, // ???
	{ 0x3056, 0xF8 }, // ???
	{ 0x3057, 0x29 }, // ???
	{ 0x3058, 0x1F }, // ???

	{ 0x3059, 0x22 }, // serial 1-bit mode
	{ 0x3060, 0x20 }, // vt_sys_div 8, vt_reg_div 4, lsb first, gated clock
	{ 0x3062, 0xCC }, // drive strength control, PCLK, D0
	{ 0x3064, 0x00 }, // trigger sync disable
	{ 0x3065, 0x04 }, // output pin status control(?)
	{ 0x3067, 0x00 }, // MCLK enable, rising edge (documentation seems to have this reversed)
	{ 0x3068, 0x20 }, // PCLK polarity rising edge

	{ 0x0104, 0x01 }, // group parameter hold(?)
};

static const sensor_reg camera_start_streaming_regs[] = {
	{ 0x0340, 0x00 }, // frame lines H
	{ 0x0341, 0x00 }, // frame lines L
	{ 0x0342, 0x00 }, // line length H
	{ 0x0343, 0x00 }, // line length L

	{ 0x0383, 0x00 }, // readout X full
	{ 0x0387, 0x00 }, // readout Y full
	{ 0x0390, 0x00 }, // no binning
	{ 0x1012, 0x00 }, // vsync shift enable

	{ 0x3010, 0x00 }, // [0] 1 enable QVGA

	{ 0x2105, 0x00 }, // Maximum INTG High Byte (default 0x01) note: if the integration time is larger than (rows - 2), the sensor starts skipping frames
	{ 0x2106, 0x00 }, // Maximum INTG Low Byte (default 0x54)

	{ 0x0104, 0x01 }, // group parameter hold(?)

	{ 0x0100, 0x01 }, // mode: streaming
};


void PicoHM01B0::arducam_regs_write(const sensor_reg *camera_regs, int count)
{
	// send register values to the camera chip, adjusting the values that
	// depend on configured values
	for (int i = 0; i < count; i++) {
		const int reg = camera_regs[i].reg;
		uint8_t value = camera_regs[i].val;

		switch (reg) {
		case 0x0101: value = config.flip_horizontal + config.flip_vertical * 2; break; // flip horizontal, flip vertical

		case 0x1012: value = binning_2x2 ? 0x03 : 0x01; break;	// vsync shift enable

		// Maximum INTG High/Low Bytes (default 0x01) note: if the
		// integration time is larger than (rows - 2), the sensor starts
		// skipping frames
		case 0x2105: value = (line_count - 2) >> 8; break;
		case 0x2106: value = (line_count - 2) & 0xFF; break;

		case 0x3010: value = qvga_mode && !binning_2x2; break;	// [0] 1 enable QVGA

		case 0x0340: value = line_count >> 8; break;	// frame lines H
		case 0x0341: value = line_count & 0xFF; break;	// frame lines L
		case 0x0342: value = line_length >> 8; break;	// line length H
		case 0x0343: value = line_length & 0xFF; break;	// line length L

		case 0x0383:	// readout X full
		case 0x0387: value = binning_2x2 ? 0x03 : 0x01; break;	// readout Y full
		case 0x0390: value = binning_2x2 ? 0x03 : 0x00; break;	// binning
		}

		// actually send the data on the i2c bus
		if (i2c_write_reg(reg, value) == 0)
			i2c_bus_stop();

		// after the reset command, wait 200 ms to allow the camera to
		// go through its internal reset. This time must be larger than
		// the frame period the camera is acquiring at the time the
		// reset is sent
		if (reg == 0x0103)
			sleep_ms(200);
	}
}


int PicoHM01B0::begin(const PicoHM01B0_config &config)
{
	// the object stores the pio numbers and not instances, so we just need
	// temporary instances in this function
	PIO clock_pio, data_pio;

	// save a copy of the configuration
	this->config = config;

	// setup a master clock, if the hardware doesn't provide one
	if (config.mclk_gpio >= 0) {
		if (!pio_claim_free_sm_and_add_program(&clock_program, &clock_pio, &clock_pio_sm, &clock_pio_offset))
			return 0;
		clock_program_init(clock_pio, clock_pio_sm, clock_pio_offset, config.mclk_gpio);
		clock_pio_idx = PIO_NUM(clock_pio);
	}

	// allocate the PIO code to transfer image data
	if (!image_program_init(&data_pio, &data_pio_sm, &data_pio_offset, config.d0_gpio, config.pclk_gpio)) {
		if (config.mclk_gpio >= 0)
			pio_remove_program_and_unclaim_sm(&clock_program, clock_pio, clock_pio_sm, clock_pio_offset);
		return 0;
	}
	data_pio_idx = PIO_NUM(data_pio);

	// allocate DMA channel dynamically
	dma_channel = dma_claim_unused_channel(true);

	// set the i2c pins as outputs by default, both pins at level high (idle)
	gpio_init(config.i2c_clk_gpio);
	gpio_set_dir(config.i2c_clk_gpio, GPIO_OUT);
	gpio_put(config.i2c_clk_gpio, 1);
	gpio_init(config.i2c_dat_gpio);
	gpio_set_dir(config.i2c_dat_gpio, GPIO_OUT);
	gpio_put(config.i2c_dat_gpio, 1);

	// give a few milliseconds with the MCLK already running to initialize
	// the camera's internal circuits
	sleep_ms(50);

	arducam_regs_write(camera_reset_regs, sizeof(camera_reset_regs) / sizeof(sensor_reg));

	return 1;
}

void PicoHM01B0::set_fixed_exposure(float exposure_ms, int d_gain, int a_gain)
{
	int exp_lines = line_count * (exposure_ms * actual_frame_rate / 1000.0);

	if (exp_lines > (int)(line_count - 2))
		exp_lines = line_count - 2;
	if (exp_lines < 2)
		exp_lines = 2;

	if (a_gain < 0)
		a_gain = 0;
	if (a_gain > 7)
		a_gain = 7;

	if (d_gain < 1)
		d_gain = 1;
	if (d_gain > 255)
		d_gain = 255;

	i2c_write_reg(0x2100, 0x00); // AE disable

	i2c_write_reg(0x0202, exp_lines >> 8); // integration time high
	i2c_write_reg(0x0203, exp_lines & 0xff); // integration time low
	i2c_write_reg(0x0205, a_gain << 4);
	i2c_write_reg(0x020E, d_gain >> 6);
	i2c_write_reg(0x020F, d_gain << 2);

	i2c_write_reg(0x0104, 0x01); // group parameter hold(?)
}

void PicoHM01B0::set_auto_exposure(void)
{
	i2c_write_reg(0x2100, 0x01); // AE enable
}

void PicoHM01B0::start_streaming(float frame_rate, bool binning_2x2, bool qvga_mode)
{
	this->frame_rate = frame_rate;
	this->binning_2x2 = binning_2x2;
	this->qvga_mode = qvga_mode;

	// the actual line length and count must depend on the frame rate and we
	// need to compute them dynamically
	calc_optimal_length();

	// initialise the camera chip over i2c
	arducam_regs_write(camera_start_streaming_regs, sizeof(camera_start_streaming_regs) / sizeof(sensor_reg));

	// wait for v-sync
	int timeout = 10000000UL;
	while (gpio_get(config.vsync_gpio) == false && timeout)
		timeout--;
}

void PicoHM01B0::stop_streaming(void)
{
	//TODO: keep track of current state (streaming, capturing, ...) don't stop streaming if not appropriate
	i2c_write_reg(0x0100, 0x00); // mode: standby
}

void PicoHM01B0::start_capture(uint8_t *dest)
{
	// the full frame size is always a multiple of 4 bytes, so we transfer
	// 32 bits at a time to make better use of the bus
	const uint32_t image_buf_size = (get_rows() * get_cols()) / 4;

	// setup the DMA transfer
	const PIO data_pio = PIO_INSTANCE(data_pio_idx);
	dma_channel_config c = dma_channel_get_default_config(dma_channel);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_dreq(&c, pio_get_dreq(data_pio, data_pio_sm, false));
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	dma_channel_configure(dma_channel, &c, dest, &(data_pio->rxf[data_pio_sm]), image_buf_size, false);

	// wait for vsync to be low to start acquiring a new frame
	do { } while (gpio_get(config.vsync_gpio) == true);

	dma_channel_start(dma_channel);
	pio_sm_set_enabled(data_pio, data_pio_sm, true);
}

void PicoHM01B0::wait_for_frame(void)
{
	// wait for DMA to finish
	dma_channel_wait_for_finish_blocking(dma_channel);
	// disable the image transfer PIO
	pio_sm_set_enabled(PIO_INSTANCE(data_pio_idx), data_pio_sm, false);
}
