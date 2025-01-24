#include <stdint.h>

//#include <hardware/i2c.h>
//#include <hardware/pwm.h>

#include <hardware/dma.h>
#include <hardware/pio.h>
#include <hardware/i2c.h>
#include <pico/time.h>

#include <hardware/i2c.h>

#include "PicoHM01B0.h"

//TODO: allow configurations to be done on the begin call, instead of being hard-coded:
//     - MCLK generated or external
//     - frame rate
//     - resolution: 162x162, 324x324 or 324 x 244
//     - bus width: 1 or 4 bits (or 8?)
//     - flip horizontal / vertical
//     - (?) auto exposure / manual setting
//     - (?) flicker control: off, 50Hz, 60Hz


//#define BINNING_MODE
//#define QVGA_MODE

#ifdef BINNING_MODE
#define CAM_COLS 164
#define CAM_LINE_LENGTH 215 // minimum 215
#ifdef QVGA_MODE
#error not suported
#define CAM_ROWS 122
#else
#define CAM_ROWS 162
#define CAM_LINE_COUNT 193 // minimum 180 (based on QVGA binning example)
#endif
#else
#define CAM_COLS 324
#define CAM_LINE_LENGTH 376	// 376
#ifdef QVGA_MODE
#define CAM_ROWS 244
#define CAM_LINE_COUNT 260
#else
#define CAM_ROWS 324
#define CAM_LINE_COUNT 344

//#define CAM_LINE_COUNT 442	// 25Hz

#endif
#endif

// frame period = (CAM_LINE_COUNT - v_advance (=1)) * (CAM_LINE_LENGTH - h_advance (=2)) * (1/33MHz)


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
//     camera code

static const int sensor_address = 0x24;

struct sensor_reg {
	uint16_t reg;
	uint8_t val;
};

struct sensor_reg camera_regs[] = {
	{0x0103, 0x00}, // reset
	{0x0100, 0x00}, // mode: 000 - standby

	{0x0101, 0x03}, // image orientation: 3: flip vertical and horizontal

	// { 0x202 ... 0x20F: exposure control

	//	{ 0x0340, 0x01 },	// frame lines H
	//	{ 0x0341, 0x7A },	// frame lines L
	//	{ 0x0342, 0x01 },	// line length H
	//	{ 0x0343, 0x77 },	// line length L

	{0x0340, CAM_LINE_COUNT >> 8},    // frame lines H
	{0x0341, CAM_LINE_COUNT & 0xFF},  // frame lines L
	{0x0342, CAM_LINE_LENGTH >> 8},   // line length H
	{0x0343, CAM_LINE_LENGTH & 0xFF}, // line length L

	{0x0350, 0x7F}, // ???

#ifdef BINNING_MODE
	{0x0383, 0x03}, // readout x BIN2 timing
	{0x0387, 0x03}, // readout y BIN2 timing
	{0x0390, 0x03}, // 1/4 binning
	{0x1012, 0x03}, // 3: vsync+hsync shift enable
#else
	{0x0383, 0x01}, // readout X full
	{0x0387, 0x01}, // readout Y full
	{0x0390, 0x00}, // no binning
	{0x1012, 0x01}, // vsync shift enable
#endif
	//{ 0x1012, 0x00 },	// 3: sync shift disable

	{0x1000, 0x43}, // black level enable
	{0x1001, 0x40}, // ???
	{0x1002, 0x32}, // ???
	{0x1003, 0x08}, // black level (default 0x20)
	{0x1006, 0x01}, // BLI enable
	{0x1007, 0x08}, // black level 2 (must be the same as black level)
	{0x1008, 0x00}, // dpc control: dpc off, boundary bypass disable
	{0x1009, 0xA0}, // cluster hot pixel threshold
	{0x100A, 0x60}, // cluster cold pixel threshold
	{0x100B, 0x90}, // single hot pixel threshold	(default 0xFF)
	{0x100C, 0x40}, // single cold pixel threshold	(default 0xFF)

	// motion detection ROI
	{0x2000, 0x05}, // motion detection disable, AE stat enable, avg 16 frame

	// automatic exposure
	{0x2100, 0x01},		     // AE enable
	{0x2101, 0x5F},		     // AE target mean (default 0x3C)
	{0x2102, 0x0A},		     // AE minimum mean (default 0x0A)
	{0x2103, 0x03},		     // converge in threshold (default 0x03)
	{0x2104, 0x05},		     // converge out threshold (default 0x05)
	{0x2105, (CAM_ROWS - 2) >> 8},   // Maximum INTG High Byte (default 0x01) note: if the integration time is larger than (rows - 2), the sensor starts skipping frames
	{0x2106, (CAM_ROWS - 2) & 0xFF}, // Maximum INTG Low Byte (default 0x54)
	{0x2107, 0x02},		     // Minimum INTG (default 0x02)
	{0x2108, 0x03},		     // Maximum Analog gain in full frame mode (default 0x03)
	{0x2109, 0x03},		     // Maximum Analog gain in BIN2 frame mode (default 0x04)
	{0x210A, 0x00},		     // Minimum Analog gain (default 0x00)
	{0x210B, 0x80},		     // Maximum digital gain (default 0xC0)
	{0x210C, 0x40},		     // Minimum digital gain (default 0x40)
	{0x210D, 0x20},		     // Damping factor (default 0x20)
	{0x210E, 0x00},		     // Flicker step control [0]:enable, [1] 0:50Hz, 1:60Hz (disabled)
	{0x210F, 0x00},		     // Flicker Step 60Hz parameter High Byte (default 0x00)
	{0x2110, 0x85},		     // Flicker Step 60Hz parameter Low Byte (default 0x3C)
	{0x2111, 0x00},		     // Flicker Step 50Hz parameter High Byte (default 0x00)
	{0x2112, 0x70},		     // Flicker Step 50Hz parameter Low Byte (default 0x32)
	//{ 0x2113, 0x66 },	// Flicker Step hysteresis threshold (default 0x66)

	{0x2150, 0x02}, // Motion detection control [0] enable

#ifdef QVGA_MODE
	{0x3010, 0x01}, // [0] 1 enable QVGA
#else
	{0x3010, 0x00}, // [0] 1 enable QVGA
#endif
	{0x3011, 0x70}, // 8-bit mode

	{0x3022, 0x01}, // [0-4] ADVANCE_VSYNC	(default 2)
	//{ 0x3023, 0x02 },	// [0-5] ADVANCE_HSYNC	(default 2)

	{0x3044, 0x0A}, // ???
	{0x3045, 0x00}, // ???
	{0x3047, 0x0A}, // ???
	{0x3050, 0xC0}, // ???
	{0x3051, 0x42}, // ???
	{0x3052, 0x50}, // ???
	{0x3053, 0x00}, // ???
	{0x3054, 0x03}, // ???
	{0x3055, 0xF7}, // ???
	{0x3056, 0xF8}, // ???
	{0x3057, 0x29}, // ???
	{0x3058, 0x1F}, // ???

	{0x3059, 0x22}, // serial 1-bit mode
	{0x3060, 0x20}, // vt_sys_div 8, vt_reg_div 4, lsb first, gated clock
	{0x3062, 0xCC}, // drive strength control, PCLK, D0
	{0x3064, 0x00}, // trigger sync disable
	{0x3065, 0x04}, // output pin status control(?)
	{0x3067, 0x00}, // MCLK enable, rising edge (documentation seems to have this reversed)
	{0x3068, 0x20}, // PCLK polarity rising edge

	{0x0104, 0x01}, // group parameter hold(?)

	{0x0100, 0x01}, // mode: streaming
};


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
	slow_gpio_put(pin_siod, 1);
	slow_gpio_put(pin_sioc, 1);
	slow_gpio_put(pin_siod, 0);
	slow_gpio_put(pin_sioc, 0);
}

void PicoHM01B0::i2c_bus_stop(void)
{
	slow_gpio_put(pin_siod, 0);
	slow_gpio_put(pin_sioc, 1);
	slow_gpio_put(pin_siod, 1);
}

void PicoHM01B0::i2c_bus_send_ack(void)
{
	slow_gpio_put(pin_siod, 0);
	slow_gpio_put(pin_sioc, 0);
	slow_gpio_put(pin_sioc, 1);
	slow_gpio_put(pin_sioc, 0);
	slow_gpio_put(pin_siod, 0);
}

int PicoHM01B0::i2c_bus_write_byte(int data)
{
	int i, tem;

	for (i = 0; i < 8; i++) {
		slow_gpio_put(pin_siod, ((data << i) & 0x80) != 0);
		slow_gpio_put(pin_sioc, 1);
		slow_gpio_put(pin_sioc, 0);
	}

	gpio_set_dir(pin_siod, GPIO_IN);
	small_pause();
	slow_gpio_put(pin_sioc, 1);
	tem = !gpio_get(pin_siod);
	slow_gpio_put(pin_sioc, 0);
	gpio_set_dir(pin_siod, GPIO_OUT);
	return tem;
}

int PicoHM01B0::wrSensorReg16_8(int regID, int regDat)
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

void PicoHM01B0::arducam_regs_write(void)
{
	const int entry_count = sizeof(camera_regs) / sizeof(camera_regs[0]);
	for (int i = 0; i < entry_count; i++) {
		if (wrSensorReg16_8(camera_regs[i].reg, camera_regs[i].val) == 0)
			i2c_bus_stop();
		// after the reset command, wait 100 ms to allow the camera to
		// go through its internal reset
		if (i == 0)
			sleep_ms(100);
	}
}


int PicoHM01B0::begin(int i2c_dat, int i2c_clk, int vsync, int d0, int pclk, int mclk)
{
	// the object stores the pio numbers and not instances, so we just need
	// temporary instances in this function
	PIO clock_pio, data_pio;

	pin_sioc = i2c_clk;
	pin_siod = i2c_dat;
	pin_mclk = mclk;
	pin_vsync = vsync;
	pin_pclk = pclk;
	pin_d0 = d0;

	// setup a master clock, if the hardware doesn't provide one
	if (pin_mclk >= 0) {
		if (!pio_claim_free_sm_and_add_program(&clock_program, &clock_pio, &clock_pio_sm, &clock_pio_offset))
			return 0;
		clock_program_init(clock_pio, clock_pio_sm, clock_pio_offset, pin_mclk);
		clock_pio_idx = PIO_NUM(clock_pio);
	}

	// allocate DMA channel dynamically
	dma_channel = dma_claim_unused_channel(true);

	// set the i2c pins as outputs by default, both pins at level high (idle)
	gpio_init(pin_sioc);
	gpio_set_dir(pin_sioc, GPIO_OUT);
	gpio_put(pin_sioc, 1);
	gpio_init(pin_siod);
	gpio_set_dir(pin_siod, GPIO_OUT);
	gpio_put(pin_siod, 1);

	// give a few milliseconds with the MCLK already running to initialize
	// the camera's internal circuits
	sleep_ms(50);

	// initialise the camera chip over i2c
	arducam_regs_write();

	// allocate the PIO code to transfer image data
	if (!image_program_init(&data_pio, &data_pio_sm, &data_pio_offset, pin_d0, pin_pclk)) {
		if (pin_mclk >= 0)
			pio_remove_program_and_unclaim_sm(&clock_program, clock_pio, clock_pio_sm, clock_pio_offset);
		return 0;
	}
	data_pio_idx = PIO_NUM(data_pio);

	// wait for v-sync
	int timeout = 10000000UL;
	while (gpio_get(pin_vsync) == false && timeout)
		timeout--;

	return timeout != 0;
}

void PicoHM01B0::start_capture(uint8_t *dest)
{
	// the full frame size is always a multiple of 4 bytes, so we transfer
	// 32 bits at a time to make better use of the bus
	const uint32_t image_buf_size = (CAM_ROWS * CAM_COLS) / 4;

	// setup the DMA transfer
	const PIO data_pio = PIO_INSTANCE(data_pio_idx);
	dma_channel_config c = dma_channel_get_default_config(dma_channel);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_dreq(&c, pio_get_dreq(data_pio, data_pio_sm, false));
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);

	dma_channel_configure(dma_channel, &c, dest, &(data_pio->rxf[data_pio_sm]), image_buf_size, false);

	// wait for vsync falling edge to start frame
	while (gpio_get(pin_vsync) == true)
		;

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
