#ifndef PicoHM01B0_H
#define PicoHM01B0_H

class PicoHM01B0 {
public:
	// call begin to initialize the camera chip. If the hardware provides
	// the MCLK (sometimes marked as XCLK) to the chip, pass -1 as mclk pin
	// (default). If a valid mclk pin is passed, the code will provide a
	// master clock on this pin. Returns 1 on success, 0 on error
	int begin(int i2c_dat, int i2c_clk, int vsync, int d0, int pclk, int mclk = -1);

	// start capturing a frame. This waits for the vertical sync to start
	// capturing. The code uses DMA to do the transfer, so the function
	// returns immediately. If the code is in a loop calling wait_for_frame,
	// do some processing and call start_capture again, if the processing
	// takes less time than the vertical blanking time, you get the full
	// frame rate. Alternatively, if there are 2 buffers, the code can
	// alternate between them and as soon as wait_for_frame returns for
	// buffer 0 start capture can be immediately called for buffer 1 while
	// buffer 0 is being processed.
	void start_capture(uint8_t *dest);

	// blocks waiting for the completion of a previous call to start_capture
	void wait_for_frame(void);

private:
	// pin definitions
	uint pin_sioc;
	uint pin_siod;
	uint pin_mclk;
	uint pin_vsync;
	uint pin_pclk;
	uint pin_d0;

	// RP2040 resources
	uint data_pio_idx;
	uint data_pio_sm;
	uint data_pio_offset;
	uint dma_channel;

	uint clock_pio_idx;
	uint clock_pio_sm;
	uint clock_pio_offset;

	// the camera code only uses i2c for initialization, se we just do a
	// bit-bang i2c to allow full flexibility in the pin selection
	void i2c_bus_start(void);
	void i2c_bus_stop(void);
	void i2c_bus_send_ack(void);
	int i2c_bus_write_byte(int data);
	int wrSensorReg16_8(int regID, int regDat);
	void arducam_regs_write(void);
};

#endif
