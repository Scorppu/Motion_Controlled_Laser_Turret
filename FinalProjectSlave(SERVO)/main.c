// hardware connection:
// servo red wire -> V Bus
// servo brown wire -> GND
// servo (pitch) orange wire -> PD0
// servo (yaw) orange wire -> PD1
// pitch: up-down, yaw: left-right

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "utils/uartstdio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"


float servo_pwm_freq = 50;
int pitchBT, yawBT;
int lastPitch, lastYaw;
int diffYaw, diffPitch;
int filtered_yaw_angle = 0, filtered_pitch_angle = 0;
int alpha = 0.1;
float pitch_duty_cycle, yaw_duty_cycle;
bool receiving = false;

// determine the duty cycle according to the desired angle
float angleToPWMDutyCycle(float angle)
{
    // angle (duty cycle): 0 (0.5ms/20ms), 90 (1.5ms/20ms), 180 (2.5ms/20ms)
    // angle to pulse width: pulse_width = angle / 90 + 0.5
    // pulse width to duty cycle: duty_cycle = pulse_width / period
    // valid angle range: 0-180
    return (angle / 90 + 0.5) / (1000 / servo_pwm_freq);
}

void UART5IntHandler(void);

int main()
{
    // set the system clock and the PWM clock
    // system clock frequency : PWM clock frequency = 64 : 1
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    // enable module PWM1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlDelay(SysCtlClockGet() / 30); // avoid program overheat & logic issues
    // configure generator 0 of PWM1
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    // calculate the number of PWM instruction cycles in each PWM period
    uint32_t pwm_period = (SysCtlClockGet() / 64 / servo_pwm_freq);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, pwm_period);
    // enable the 0th and 1st outputs of PWM1
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
    // PD0 and PD1 to send the signals to the servos
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);

    // UART
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

    // UART INTERRUPT
    IntMasterEnable();
    IntEnable(INT_UART5);
    UARTIntEnable(UART5_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntRegister(UART5_BASE, UART5IntHandler);

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

    // float pitch_angle, yaw_angle;

    while (true)
   {
        // Provided Code
        /*
        yaw_angle = 0;
        yaw_duty_cycle = angleToPWMDutyCycle(yaw_angle);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 3);
        yaw_angle = 90;
        yaw_duty_cycle = angleToPWMDutyCycle(yaw_angle);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 3);
        yaw_angle = 180;
        yaw_duty_cycle = angleToPWMDutyCycle(yaw_angle);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 3);
        yaw_angle = 90;
        yaw_duty_cycle = angleToPWMDutyCycle(yaw_angle);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 3);
        pitch_angle = 60;
        pitch_duty_cycle = angleToPWMDutyCycle(pitch_angle);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * pitch_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 3);
        pitch_angle = 90;
        pitch_duty_cycle = angleToPWMDutyCycle(pitch_angle);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * pitch_duty_cycle);
        SysCtlDelay(SysCtlClockGet() / 3);
        */


       // My code
        yaw_duty_cycle = angleToPWMDutyCycle(yawBT);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * yaw_duty_cycle);

        pitch_duty_cycle = angleToPWMDutyCycle(pitchBT);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0) * pitch_duty_cycle);


        SysCtlDelay(SysCtlClockGet() / 1000);
    }
}

// My code as well
void UART5IntHandler(void) {
    uint32_t ui32Status = UARTIntStatus(UART5_BASE, true);

    char input[8] = {0};
    int i = 0;
    while (1) {
        char ch = UARTCharGet(UART5_BASE);
        // using # as starting character is necessary to make sure each received packet is in order with no corrupt data
        if(ch == '#'){
            receiving = true;
            UARTCharPut(UART0_BASE, ch);
        }
        else if(receiving){
            if (ch == '\n') {
                UARTCharPut(UART0_BASE, ch);
                receiving = false;
                break;
            }
            input[i++] = ch;
            UARTCharPut(UART0_BASE, ch);
        }
    }
    input[i] = '\0';
    yawBT = 180 - ((input[0] - '0') * 100 + (input[1] - '0') * 10 + (input[2] - '0'));
    pitchBT = (input[4] - '0') * 100 + (input[5] - '0') * 10 + (input[6] - '0');

    // BELOW ARE FAILED ATTEMPTS AT REMOVING SERVO JITTER
    //diffYaw = abs(yawBT - lastYaw);
    //diffPitch = abs(pitchBT - lastPitch);

    //lastYaw = yawBT;
    //lastPitch = pitchBT;
    //filtered_yaw_angle = alpha * angleBT + (1 - alpha) * filtered_yaw_angle;
    //filtered_pitch_angle = alpha * pitchBT + (1 - alpha) * filtered_pitch_angle;



    UARTIntClear(UART5_BASE, ui32Status);
}
