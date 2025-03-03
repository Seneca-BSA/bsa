# Lab 4 : Serial UART and I2C Communication

<font size="5">
Seneca Polytechnic</br>
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

### Part 3: Analog Input

Next, we'll connect the ADC to an analog voltage input so it can be read into the microcontroller. The voltage signal will come from the bench power supply. To prevent damaging the ADC, we'll first use a voltage division to halve the signal. **Never connect the power supply directly to the ADC without a resistor.**

![Figure 3.2](lab3-vs-adc.png)

***Figure 3.2** Voltage division circuit for ADC input*

1. Assemble the voltage division circuit above on your breadboard and connect the voltage divider output to an analog input pin. Vs will be provided by the bench power supply. Ensure the power supply output is OFF. Optionally, you can use a potentiometer to achieve the same.
1. As a precaution, we only want to supply a maximum of 3.3V to the ADC pin, this means the power supply output should never to above 6.6V. Set the power supply output to 1V.
1. Add the following code in the main function of your code but before the while loop to set up an analog input pin. Remember, you'll need to replace PTXX with the PWM pin that you are using.
    <pre>
    
        int main()
        {
            ...
            AnalogIn ain(PTXX); // replace with a ADC pin
            ...
        }
    </pre>

1. Next, replace the while loop with the following code so we are controlling the DAC output and the PWM duty cycle with the analog input.
    <pre>
    
        while (true)
        {
            i = ain; // read ADC
            aout = i; // set DAC out %
            pwm = i; // set PWM duty cycle %

            // delay for 10ms, 1000ms for each ramp up
            ThisThread::sleep_for(100ms);
        }
    </pre>

1. Next, let's print the reading out as well. Modify your code to include the following respectively.
    <pre>
    
        while (true)
        {
            ...
            // print the percentage, 16 bit normalized values, and something we understand
            printf("percentage: %3.3f%% ", ain.read() * 100.0f);
            printf("normalized: 0x%04X ", ain.read_u16());
            printf("normalized: %3.3fV \n", ain * 3.3); // % * 3.3V
            ...
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

1. Turn on the power supply output and run the program. You should see a constant 0.5V on the DSO, at about 15% duty cycle PWM wave, and a serial output state of about 15% and 0.5V.

    > **Lab Question:** What do you think can be done to reduce reading fluctuation? How do you think that can be achieved?

### Part 1: Onboard I2C Accelerometer and Magnetometer

In Part 1, we'll take a look at how to get reading from the onboard accelerometer and magnetometer.

<div style="padding: 15px; border: 1px solid orange; background-color: orange; color: black;">
Check to see if the accelerometer is assembled on your board. NXP had a production change in 2023 and no longer assembles the FXOS8700CQ onto the Freedom board. If your board is missing the accelerometer chip (as shown in Figure 4.1 below), this part of the Lab will not work.
</div>

The location U8 on the Freedom Board should be assembled with the FXOS8700CQ accelerometer chip.

![Figure 4.1](lab4-u8.png)

***Figure 4.1** Freedom Board with missing FXOS8700CQ accelerometer chip*

1. To use the FXOS8700CQ, you'll need to add the FXOS8700CQ library to your project. Start Mbed Studio then go to File > Add Library to Active Program. When prompted, provide the following link [https://os.mbed.com/teams/NXP/code/FXOS8700Q/](https://os.mbed.com/teams/NXP/code/FXOS8700Q/).

1. The I2C pins used to connect to the accelerometer for the Freedom board are as follows:

    | | K64F | K66F |
    |---|---|---|
    |SDA|PTE25|PTD9|
    |SCL|PTE24|PTD8|

    Start your program with the following code to include the proper library and set up I2C.
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

1. Per [Minimal printf and snprintf](https://github.com/ARMmbed/mbed-os/blob/master/platform/source/minimal-printf/README.md), as of mbed OS 6, printf no longer prints floating point by default to save memory. To enable printing of floating point value, enable it by creating a file called `mbed_app.json` in the root project folder and adding the following code to it.
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
    
    > **Lab Question:** Try getting readings from different axes to figure out which direction is X, Y, and Z? When there is acceleration in an axis, you'll get acceleration reading on that axis (including gravity).

### Part 2: Visualize I2C Signal

1. Power off the Freedom board and connect the SDA pin to CH1 and the SCL pin to CH2 of the oscilloscope. If you are using the K66F board, use I2C1 at PTC11 and PTC10 for this part of the lab.

1. With the I2C code running, adjust the oscilloscope to see the full I2C data frame. Use the "Serial" option under Measure on the right of the face plate to align the I2C signal. If Serial measurement is not available, you might need to do this manually. If you cannot see the I2C signal, decrease the delay in each loop so data are sent more often and use "Single" reading instead of continuous readings.
    
    > **Lab Question:** Using the figure below as a reference, identify the start condition, the address, ACK, data, and stop condition of your I2C signal. You should be able to identify the address you are sending from Part 1.

    ![Figure 4.2](lab4-i2c-frame.jpg)

    ***Figure 4.2** I2C data frame [1]*

### Part 3: UART Communication

In this part of the lab, you'll be working with the group beside you to communicate between processor board.

1. Add the following code to your program create an unbuffered serial object for UART.

    | | K64F | K66F |
    |---|---|---|
    |UART TX|PTC17|PTC4|
    |UART RX|PTC16|PTC3|

    <pre>
    
        static BufferedSerial serial_port(UART_TX, UART_RX, 9600); // replace with UART pins
    </pre>

1. Add the following in your `while` loop to send some data.
    <pre>

        static char c = 'a';
        static int x = 1;
        serial_port.write(&c, 1);
        c += x;
        if (c >= 'z' || c <= 'a')
            x *= -1;
        ThisThread::sleep_for(1s);
    </pre>

1. Add the following code in the while loop to read incoming UART data from buffer and print it to terminal.
    <pre>

        if (serial_port.read(&c, 1)) {
            // Echo the input back to the terminal.
            ThisThread::sleep_for(100ms); // allow reading to finish
            printf("%c\n", c);
        }
        
    </pre>

1. Connect the UART TX pin from one board to the UART RX pin on another board as well as a common ground. Once you run the program, the TX board will start sending a char per loop to the RX board and the received data will be displayed on the serial console.

    > **Lab Question:** Change your code to send multiple characters through UART.

## Reference

- [AnalogIn](https://os.mbed.com/docs/mbed-os/v6.16/apis/i-o-apis.html)
- [1] [https://learn.sparkfun.com/tutorials/i2c/all](https://learn.sparkfun.com/tutorials/i2c/all)
- [UnbufferedSerial](https://os.mbed.com/docs/mbed-os/v6.16/apis/unbufferedserial.html)