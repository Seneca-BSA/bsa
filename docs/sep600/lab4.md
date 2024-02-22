# Lab 4 : Serial UART and I2C Communication

<font size="5">
Seneca College</br>
SEP600 Embedded Systems
</font>

## Introduction

Documentation of the Cortex-M4 instruction set, board user's guide, and the microcontroller reference manual can be found here:

### Cortex M4

- [Arm Cortex-M4 Processor Technical Reference Manual Revision](https://developer.arm.com/documentation/100166/0001)
- [ARMv7-M Architecture Reference Manual](https://developer.arm.com/documentation/ddi0403/latest/)

### FRDM-K64F

- [FRDM-K64F Freedom Module User’s Guide](FRDMK64FUG.pdf) (From [nxp.com](https://www.nxp.com/webapp/Download?colCode=FRDMK64FUG))
- [Kinetis K64 Reference Manual](K64P144M120SF5RM.pdf) (From [nxp.com](https://www.nxp.com/webapp/Download?colCode=K64P144M120SF5RM))
- [FRDM-K64F mbed](https://os.mbed.com/platforms/FRDM-K64F/)

### FRDM-K66F

- [FRDM-K66F Freedom Module User’s Guide](FRDMK66FUG.pdf) (From [nxp.com](https://www.nxp.com/webapp/Download?colCode=FRDMK66FUG))
- [Kinetis K66 Reference Manual](K66P144M180SF5RMV2.pdf) (From [nxp.com](https://www.nxp.com/webapp/Download?colCode=K66P144M180SF5RMV2))
- [FRDM-K66F mbed](https://os.mbed.com/platforms/FRDM-K66F/)

## Materials
- Safety glasses (PPE)
- Breadboard
- Jumper Wires

## Preparation

> ### Lab Preparation Question
> 1. Read over the lab and understand the procedures.

## Procedures

### Part 1: Onboard I2C Accelerometer and Magnetometer

In Part 1, we'll take a look at how to setup getting reading from the onboard accelerometer and magnetometer.

<div style="padding: 15px; border: 1px solid orange; background-color: orange; color: black;">
Check to see if the accelerometer is assembled on your board. NXP had a production change in 2023 and no longer assemble the FXOS8700CQ onto the Freedom board. If your board is missing the accelerometer chip (as shown in Figure 4.1 below), this part of the Lab will no work.
</div>

The location U8 on the Freedom Board should be assembled with the FXOS8700CQ accelerometer chip.

![Figure 4.1](lab4-u8.png)

***Figure 4.1** Freedom Board with missing FXOS8700CQ accelerometer chip*

1. In order to use the FXOS8700CQ, you'll need to add the FXOS8700CQ library to your project. Start Mbed Studio than go to File > Add Library to Active Program. When prompted, provide the following link [https://os.mbed.com/teams/NXP/code/FXOS8700Q/](https://os.mbed.com/teams/NXP/code/FXOS8700Q/).

1. The I2C pins used to connect to the accelerometer for the Freedom board are as follow:

    | | K64F | K66F |
    |---|---|---|
    |SDA|PTE25|PTD9|
    |SCL|PTE24|PTD8|

    Start your program with the following code to include the proper library and setup I2C.
    <pre>

        #include "mbed.h"
        #include "FXOS8700Q.h"

        I2C i2c(I2C_SDA, I2C_SCL); // replace with I2C pins
    </pre>

1. Next, we'll create the accelerometer and magnetometer objects using the I2C object we created and the accelerometer's address. You can find the address in the header file.
    <pre>

        FXOS8700QAccelerometer acc(i2c, FXOS8700CQ_SLAVE_ADDR1);
        FXOS8700QMagnetometer mag(i2c, FXOS8700CQ_SLAVE_ADDR1);
    </pre>

    > **Lab Question:** Look into the header file to find the slave address in HEX?

1. Declare the variables for the sensor data within the `main` function then enable the sensor.
    <pre>

        motion_data_units_t acc_data, mag_data;
        float faX, faY, faZ, fmX, fmY, fmZ, tmp_float;

        acc.enable();
        mag.enable();
    </pre>

1. Add a `while` loop to get accelerometer readings and print it out.
    <pre>

        while (true) {
            acc.getAxis(acc_data);
            mag.getAxis(mag_data);
            printf("%3.3f %3.3f\r\n", acc_data.x, mag_data.x);
            ThisThread::sleep_for(500ms);
        }
    </pre>

1. Per [Minimal printf and snprintf](https://github.com/ARMmbed/mbed-os/blob/master/platform/source/minimal-printf/README.md), as of mbed OS 6, printf no longer print floating point by default to save memory. In order to enable printing of floating point value, enable it by creating a file called `mbed_app.json` in the root project folder and add the following code to it.
    <pre>

        {
            "target_overrides": {
                "*": {
                    "target.printf_lib": "std"
                }
            }
        }
    </pre>

1. Run your program and you should now see accelerometer and magnetometer readings. Refer to the FXOS8700Q libraries for other library functions and reading you can get.
    > **Lab Question:** Try getting reading from different axis to figure out which direction is X, Y, and Z? When there are acceleration in an axis, you'll get acceleration reading on that axis (including gravity).

### Part 2: Visualize I2C Signal

1. Power off the Freedom board and connect the SDA pin to CH1 and SCL pin to CH2 of the oscilloscope.

1. With the I2C code running, adjust the oscilloscope to see the full I2C data frame. You might need to do this manually. If you cannot see the I2C signal, decrease the delay in each loop so data are send more often.
    > **Lab Question:** Using the figure below as a reference, identify the start condition, the address, ACK, data, and stop condition of your I2C signal.

    ![Figure 4.2](lab4-i2c-frame.jpg)

    ***Figure 4.2** I2C data frame [1]*

### Part 3: UART Communication

In this part of the lab, you'll be working with the group beside you to communicate between processor board.

1. Add the following code to your program create a unbuffered serial object for UART.

    | | K64F | K66F |
    |---|---|---|
    |UART TX|PTC17|PTC4|
    |UART RX|PTC16|PTC3|

    <pre>
    
        static UnbufferedSerial serial_port(UART_TX, UART_RX); // replace with UART pins
    </pre>

1. Add the following interrupt code to send incoming UART data to USB.
    <pre>

        void on_rx_interrupt() {
            char c;

            if (serial_port.read(&c, 1)) {
                // Echo the input back to the terminal.
                printf("%c", c);
            }
        }
    </pre>

1. Then add the following in the `main` function before the `while` loop.
    <pre>

    // Set desired properties (9600-8-N-1).
    serial_port.baud(9600);
    serial_port.format(
        /* bits */ 8,
        /* parity */ SerialBase::None,
        /* stop bit */ 1
    );

    // Register a callback to process a Rx (receive) interrupt.
    serial_port.attach(&on_rx_interrupt, SerialBase::RxIrq);
    </pre>

1. Add the following in your `while` loop to send some data.
    <pre>

    static char c = 'a';
    static int x = 1;
    serial_port.write(&c, 1);
    if (c >= 'z' || c <= 'a')
        x *= -1;
    c += x;
    </pre>

1. Connect the UART TX pin from one board to the UART RX pin on another board. Once you run the program, the TX board will start sending a char per loop to the RX board and the received data will be display on the serial console.

## Reference

- [1] [https://learn.sparkfun.com/tutorials/i2c/all](https://learn.sparkfun.com/tutorials/i2c/all)
- [UnbufferedSerial](https://os.mbed.com/docs/mbed-os/v6.16/apis/unbufferedserial.html)