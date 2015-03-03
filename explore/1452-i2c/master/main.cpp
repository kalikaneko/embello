// I2C master, reads calibration values from BMP180

#include "stdio.h"
#include "serial.h"

#include "lpc_types.h"
#include "romapi_8xx.h"

uint32_t i2cBuffer [24];
I2C_HANDLE_T* ih;

extern "C" void SysTick_Handler () {                                             
    // the only effect is to generate an interrupt, no work is done here         
}

void delay (int millis) {
    while (--millis >= 0)
        __WFI(); // wait for the next SysTick interrupt
}

void i2cSetup () {
    LPC_SWM->PINENABLE0 = 3<<2;             // disable SWCLK and SWDIO
    LPC_SWM->PINASSIGN7 = 0x02FFFFFF;       // SDA on P2, pin 4
    LPC_SWM->PINASSIGN8 = 0xFFFFFF03;       // SCL on P3, pin 3
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<5);    // enable I2C clock

    ih = LPC_I2CD_API->i2c_setup(LPC_I2C_BASE, i2cBuffer);
    LPC_I2CD_API->i2c_set_bitrate(ih, 12000000, 400000);
}

uint8_t BMP180_ADDR = 0x77;

void i2cSelectReg(uint8_t select_reg) {
    static uint8_t buf_send [2];
    static uint8_t buf_recv [1];

    I2C_PARAM_T param;
    I2C_RESULT_T result;

    buf_send[0] = (BMP180_ADDR << 1);
    buf_send[1] = select_reg;
    buf_recv[0] = (BMP180_ADDR << 1);

    param.num_bytes_send  = 2;
    param.num_bytes_rec   = 0;

    param.buffer_ptr_send = buf_send;
    param.buffer_ptr_rec = buf_recv;
    param.stop_flag       = 1;

    LPC_I2CD_API->i2c_set_timeout(ih, 100000);
    LPC_I2CD_API->i2c_master_transmit_poll(ih, &param, &result);
}

void i2cReadReg (uint8_t * buf_recv, int len_buf_calib) {
/* buf_recv is an array with the n + 1 elements
/* len_calib is the length of the array
 * (first element in the receive buffer is the address)
 */
    static uint8_t buf_send [1];
    static int err_code;

    I2C_PARAM_T param;
    I2C_RESULT_T result;

    buf_send[0] = (BMP180_ADDR << 1)|1;
    buf_recv[0] = (BMP180_ADDR << 1)|1;

    /* Setup parameters for transfer */
    param.num_bytes_send  = 1;
    param.num_bytes_rec   = len_buf_calib;

    param.buffer_ptr_send = buf_send;
    param.buffer_ptr_rec = buf_recv;
    param.stop_flag       = 1;

    LPC_I2CD_API->i2c_set_timeout(ih, 100000);
    err_code = LPC_I2CD_API->i2c_master_tx_rx_poll(ih, &param, &result);

    printf("\n");
    printf("[err  ] %d \n", err_code);
    printf("[read ] recv: %02x %02x\n", result.n_bytes_sent, result.n_bytes_recd);
}

void readCalibrationValues(uint8_t * buf_calib, int len_buf_calib) {
    // XXX todo return struct !
    
    i2cSelectReg(0xAA);
    delay(50);
    i2cReadReg(buf_calib, len_buf_calib);

    printf("[CALIBRATION] AC1 value: %02x %02x\n", buf_calib[1], buf_calib[2]);
    printf("[CALIBRATION] AC2 value: %02x %02x\n", buf_calib[3], buf_calib[4]);
    printf("[CALIBRATION] AC3 value: %02x %02x\n", buf_calib[5], buf_calib[6]);
    printf("[CALIBRATION] AC4 value: %02x %02x\n", buf_calib[7], buf_calib[8]);
    printf("[CALIBRATION] AC5 value: %02x %02x\n", buf_calib[9], buf_calib[10]);
    printf("[CALIBRATION] AC6 value: %02x %02x\n", buf_calib[11], buf_calib[12]);
    printf("[CALIBRATION] B1  value: %02x %02x\n", buf_calib[13], buf_calib[14]);
    printf("[CALIBRATION] B2  value: %02x %02x\n", buf_calib[15], buf_calib[16]);
    printf("[CALIBRATION] MB  value: %02x %02x\n", buf_calib[17], buf_calib[18]);
    printf("[CALIBRATION] MC  value: %02x %02x\n", buf_calib[19], buf_calib[20]);
    printf("[CALIBRATION] MD  value: %02x %02x\n", buf_calib[21], buf_calib[22]);
}

int main () {
    static uint8_t buf_calib[23];

    // send UART output to GPIO 4, running at 115200 baud
    LPC_SWM->PINASSIGN0 = 0xFFFFFF04UL;
    serial.init(LPC_USART0, 115200);

    printf("\n[master, reading BMP180]\n");
    SysTick_Config(12000000/1000); // 1000 Hz
    i2cSetup();

    while (true) {
	readCalibrationValues(buf_calib, sizeof(buf_calib));
	delay(5000);
    }
}
