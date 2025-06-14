#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

// MPU6050 Registers and Address
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

// Filter Parameters
#define ALPHA 0.96f           // Complementary filter coefficient
#define DT 0.05f             // Sample time (50ms)
#define RAD_TO_DEG 57.2958f  // Conversion factor

// PID Constants
#define KP 25.0f             // Proportional gain
#define KI 0.1f              // Integral gain
#define KD 0.1f              // Derivative gain
#define SETPOINT -7.0f        // Target angle
#define DEADZONE 0.5f        // Error deadzone
#define MAX_INTEGRAL 100.0f  // Anti-windup limit

// Motor 1 Pins (PORTA)
#define M1_STEP_PIN GPIO_PIN_2
#define M1_DIR_PIN GPIO_PIN_3
#define M1_ENABLE_PIN GPIO_PIN_4

// Motor 2 Pins (PORTA)
#define M2_STEP_PIN GPIO_PIN_5
#define M2_DIR_PIN GPIO_PIN_6
#define M2_ENABLE_PIN GPIO_PIN_7

// Motor Control Parameters
#define MOTOR_PORT GPIO_PORTA_BASE
#define MIN_STEP_DELAY 3000   // Maximum speed
#define MAX_STEP_DELAY 17000  // Minimum speed
#define BASE_STEP_DELAY 1000 // Default speed

// DRV8825 Control Pins (PORTF)
#define DRV_ENABLE GPIO_PIN_1
#define DRV_M0 GPIO_PIN_2
#define DRV_M1 GPIO_PIN_3
#define DRV_M2 GPIO_PIN_4
#define DRV_SLEEP GPIO_PIN_4

// Speed Control
#define ACCEL_RATE 1         // Acceleration rate
#define MAX_SPEED_CHANGE 5   // Max speed change per update
#define ERROR_SCALE 30       // Error to speed conversion

// Initial values for global variables
volatile float filtered_angle = 0.0f;
volatile float gyro_rate = 0.0f;
volatile float integral = 0.0f;
volatile float last_error = 0.0f;
volatile uint32_t motor_step_delay = BASE_STEP_DELAY;
volatile bool motor_direction = true;
volatile int16_t pitch = 0;
// I2C Initialization
void InitI2C(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
}

// MPU6050 Write Register
void MPU6050Write(uint8_t reg, uint8_t data)
{
    while (I2CMasterBusy(I2C0_BASE));
    I2CMasterSlaveAddrSet(I2C0_BASE, MPU6050_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C0_BASE));

    I2CMasterDataPut(I2C0_BASE, data);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C0_BASE));
}

// MPU6050 Read Registers
void MPU6050Read(uint8_t reg, uint8_t *buffer, uint8_t size)
{
    uint8_t i;

    while (I2CMasterBusy(I2C0_BASE));
    I2CMasterSlaveAddrSet(I2C0_BASE, MPU6050_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, MPU6050_ADDR, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

    for (i = 0; i < size - 1; i++)
    {
        while (I2CMasterBusy(I2C0_BASE));
        buffer[i] = I2CMasterDataGet(I2C0_BASE);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    }

    while (I2CMasterBusy(I2C0_BASE));
    buffer[size - 1] = I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while (I2CMasterBusy(I2C0_BASE));
}

// Initialize MPU6050
void InitMPU6050(void)
{
    MPU6050Write(PWR_MGMT_1, 0x00);     // Wake up the MPU6050
    MPU6050Write(0x1B, 0x00);           // Gyro range: ±250 degrees/sec
    MPU6050Write(0x1C, 0x00);           // Accel range: ±2g
    MPU6050Write(0x1A, 0x03);           // Digital low-pass filter
}

void calculatePID(void) {
    float error = filtered_angle - SETPOINT;

    // Only integrate if within reasonable bounds (anti-windup)
    if (fabs(error) < 10.0f) {
        integral += error * DT;
    }

    // Clamp integral term
    if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;

    // Calculate derivative (using gyro rate directly instead of error derivative)
    float derivative = -gyro_rate;  // Negative because we want to counteract the motion

    // Calculate PID output
    float output = KP * error + KI * integral + KD * derivative;

    // Convert PID output to motor control
    if (fabs(error) > DEADZONE) {
        // Determine direction
        motor_direction = (output > 0);

        // Convert output to step delay (higher output = faster speed = lower delay)
        float speed_factor = fabs(output) / 100.0f;  // Normalize to 0-1 range
        if (speed_factor > 1.0f) speed_factor = 1.0f;

        motor_step_delay = (uint32_t)(MIN_STEP_DELAY + (1.0f - speed_factor) *
                          (MAX_STEP_DELAY - MIN_STEP_DELAY));
    }
}


// Angle Measurement ISR
void AngleISR(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    // Read accelerometer and gyroscope data
    uint8_t buffer[14];  // Increased buffer size to read both accel and gyro
    int16_t ax, ay, az, gx, gy, gz;
    float ax_g, ay_g, az_g;
    float acc_angle;

    // Read both accelerometer and gyroscope data
    MPU6050Read(ACCEL_XOUT_H, buffer, 14);

    // Parse accelerometer data
    ax = (buffer[0] << 8) | buffer[1];
    ay = (buffer[2] << 8) | buffer[3];
    az = (buffer[4] << 8) | buffer[5];

    // Parse gyroscope data
    gx = (buffer[8] << 8) | buffer[9];
    gy = (buffer[10] << 8) | buffer[11];
    gz = (buffer[12] << 8) | buffer[13];

    // Convert accelerometer data to g-force
    ax_g = ax / 16384.0f;
    ay_g = ay / 16384.0f;
    az_g = az / 16384.0f;

    // Calculate pitch angle from accelerometer
    acc_angle = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;

    // Convert gyro data to degrees per second (±250 deg/s range)
    gyro_rate = gy / 131.0f;  // Using Y-axis for pitch rotation

    // Apply complementary filter
    filtered_angle = ALPHA * (filtered_angle + gyro_rate * DT) + (1 - ALPHA) * acc_angle;

    // Convert to integer pitch for existing control logic
    pitch = (int16_t)filtered_angle;

    calculatePID();

//    // Update error calculation (keeping your existing mapping)
//    error = (((pitch-SETPOINT) + 30)/60)*1800-900;
//    if (error > 900){
//        error = 900;
//    }
//    if (error < -900){
//        error = -900;
//    }
}

// Initialize Angle Measurement Timer (TIMER3)
void InitAngleTimer(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3)) {}

    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet() / 20);  // 50ms interval (20 Hz)
    TimerIntRegister(TIMER3_BASE, TIMER_A, AngleISR);
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A);
}




void setup(void)
{
    // Enable GPIO Port A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);


    // Configure PA2 (STEP) and PA3 (DIR) as outputs
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_5 );
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);  // Enable
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);  // M0
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);  // M2
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);  // rest/sleep

    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0);  // enable
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);  // M0
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6);  // M2
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);  // rest/sleep
}

int main(void) {
    // Your existing initialization code
    InitI2C();
    InitMPU6050();
    InitAngleTimer();
    IntMasterEnable();
    setup();

    while(1) {
        if (fabs(filtered_angle - SETPOINT) > DEADZONE) {
            // Set direction
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, motor_direction ? 0 : GPIO_PIN_3);
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, motor_direction ? GPIO_PIN_6 : 0);

            // Step motors
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_5,
                        GPIO_PIN_2 | GPIO_PIN_5);
            SysCtlDelay(motor_step_delay / 3);  // SysCtlDelay uses 3 cycles per loop
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_5, 0);
            SysCtlDelay(motor_step_delay / 3);
        }
    }
}
