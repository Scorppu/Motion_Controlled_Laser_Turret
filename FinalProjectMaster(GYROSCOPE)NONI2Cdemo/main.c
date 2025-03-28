#include <stdbool.h>
#include <stdint.h>
#include <stdbool.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "math.h"

int RoundedYaw, RoundedPitch;
char YawToSend[3], PitchToSend[3];
double dt = 0.35;
double dt2 = 0.2;
double yaw = 90, pitch = 30;
double ay, ax;
volatile bool g_bMPU6050Done;
tMPU6050 sMPU6050;
tI2CMInstance g_sI2CMSimpleInst;

//
// The function that is provided by this example as a callback when MPU6050
// transactions have completed.
//
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    // See if an error occurred.
    if (ui8Status != I2CM_STATUS_SUCCESS)
    {
    }

    // Indicate that the MPU6050 transaction has completed.
    g_bMPU6050Done = true;
}

//
// The interrupt handler for the I2C module.
//
void I2CMSimpleIntHandler(void)
{
    //
    // Call the I2C master driver interrupt handler.
    //
    I2CMIntHandler(&g_sI2CMSimpleInst);
}

void Initialization(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.
    // Use the system clock for the I2C0 module.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());
    // Register the interrupt handler for I2C interrupts
    I2CIntRegister(I2C0_BASE, I2CMSimpleIntHandler);

    // Configure the MPU6050
    g_bMPU6050Done = false;
    MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    // UART5 init
    // enable UART5 and GPIOE to communicate with BLUETOOTH
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // configure PE4 for RX, PE5 for TX
    GPIOPinConfigure(GPIO_PE4_U5RX);
    GPIOPinConfigure(GPIO_PE5_U5TX);
    // set PORTE pin4 and pin5 as type UART
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // set UART5 base address, clock and baud rate
    UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 38400, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));


    // UART0 init
    // enable UART0 and GPIOE to check outputs

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // set GPIO A0 and A1 as UART pins.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    // set GPIO A0 and A1 as UART pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(),115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

int main()
{
    // Set the system clock to use the PLL with a 16 MHz crystal oscillator.
    // The clock is divided by 1 (SYSCTL_SYSDIV_1) and uses an internal oscillator (SYSCTL_OSC_INT).
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);

    // Initialize the system (e.g., peripherals, hardware components)
    Initialization();

    // Declare arrays to store accelerometer and gyroscope data
    float fAccel[3], fGyro[3];

    // Reset the MPU6050 sensor by writing to the power management register (PWR_MGMT_1)
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_1, 0x00, 0b00000010 & MPU6050_PWR_MGMT_1_DEVICE_RESET, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    // Configure the MPU6050 to not be low power mode by writing to the power management register (PWR_MGMT_2)
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_2, 0x00, 0x00, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
    }

    // Main infinite loop to repeatedly read data from the MPU6050
    while (1)
    {
        //
        // Request another reading from the MPU6050 sensor
        //
        g_bMPU6050Done = false;
        MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }

        // Extract the accelerometer data (in floating-point format)
        MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0], &fAccel[1], &fAccel[2]);

        // Extract the gyroscope data (in floating-point format)
        MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);

        // Accel raw data into usable data (DISCARDED)
        // ay = atan2(-1*fAccel[0], sqrt( pow(fAccel[1], 2) + pow(fAccel[2], 2))) * 180 / M_PI;
        // ax = atan2(fAccel[1], sqrt( pow(fAccel[0], 2) + pow(fAccel[2], 2))) * 180 / M_PI;

        // Complementary filtering (DISCARDED)
        yaw = yaw - fGyro[2] * dt;
        if (yaw > 170) {
            yaw = 170;
        } else if (yaw < 10) {
            yaw = 10;
        }

        pitch = pitch - fGyro[1] * dt2;
        if (pitch > 80) {
            pitch = 80;
        } else if (pitch < 30) {
            pitch = 30;
        }

        RoundedYaw = (int) yaw;
        RoundedPitch = (int) pitch;

        int i;
        for (i = 0; i < 3; i++) {
            YawToSend[i] = RoundedYaw % 10 + '0';
            PitchToSend[i] = RoundedPitch % 10 + '0';
            RoundedYaw /= 10;
            RoundedPitch /= 10;
        }

        UARTCharPut(UART0_BASE, '#');
        UARTCharPut(UART0_BASE, YawToSend[2]);
        UARTCharPut(UART0_BASE, YawToSend[1]);
        UARTCharPut(UART0_BASE, YawToSend[0]);
        UARTCharPut(UART0_BASE, ',');
        UARTCharPut(UART0_BASE, PitchToSend[2]);
        UARTCharPut(UART0_BASE, PitchToSend[1]);
        UARTCharPut(UART0_BASE, PitchToSend[0]);
        UARTCharPut(UART0_BASE, '\n');

        UARTCharPut(UART5_BASE, '#');
        UARTCharPut(UART5_BASE, YawToSend[2]);
        UARTCharPut(UART5_BASE, YawToSend[1]);
        UARTCharPut(UART5_BASE, YawToSend[0]);
        UARTCharPut(UART5_BASE, ',');
        UARTCharPut(UART5_BASE, PitchToSend[2]);
        UARTCharPut(UART5_BASE, PitchToSend[1]);
        UARTCharPut(UART5_BASE, PitchToSend[0]);
        UARTCharPut(UART5_BASE, '\n');

        // Adjust the delay as necessary
        SysCtlDelay(SysCtlClockGet() / 1000);
    }
}
