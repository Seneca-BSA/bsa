# Lab 6

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

In this lab, we'll explore the use of software filtering techniques to remove noise from a digital signal and then plot the data on a computer. We'll also explore the idea of multi-threading to handle the two tasks.

1. Start the function generator to output a 1Vpp 1kHz Noise (or Triangular) wave with a 2V DC offset.

1. Connect the output of the function generator to an ADC (Analog input) pin on the K64F or K66F board.

1. Use the following code to read the signal from the ADC channel. Replace `PTXX` with the pin that you are using.
    <pre>

        #include "mbed.h"

        int main() {

            AnalogIn ain(PTXX); // Replace with your ADC pin

            float reading = 0; // for saving readings

            while (true) {
                reading = ain; // read ADC
                printf("Reading: %d\n", (int) (reading * 100)); // print as int
                // delay for 1ms for each reading
                ThisThread::sleep_for(1ms);
            }
        }

    </pre>

1. Your serial output should now be flooded with data output between 0 to 100.

1. Next, instead of printing data directly from within the `while` loop of the `main()` function, we'll put the printing into a thread so it won't interfere with the ADC reading and let the processor decide how to optimize the process.

1. Move the `printf` statement into a new function called `print_data()`. To pass variables into a thread, we'll make use of a pointer. Add the following code above the `main()` function.
    <pre>

        void print_data(float *reading) {
            while (true) {
                printf("%d\n", (int) (*reading * 100));
                // delay for 1ms for between print
                ThisThread::sleep_for(100ms);
            }
        }

    </pre>

1. Next, we'll define a new thread called `print_data_thread` and use `print_data` as the callback function and pass the address of `reading` as the argument. Add the following above the `print_data()` function.
    <pre>

        Thread print_data_thread;

    </pre>

1. Add the following in the `main()` function to start the thread.
    <pre>

        print_data_thread.start(callback(print_data, &reading));

    </pre>

1. Remember to remove the `printf` statement from your `main()` function.

1. Run your new code. It should be doing the same task as before but now it is running in multi-thread.

1. Let's read the data from the computer using Python and plot it out. Run the following Python script on your computer. Install `pyserial` using `pip install pyserial` and `matplotlib` using `pip install matplotlib` as required.
    <pre>

        import serial
        import matplotlib.pyplot as plt
        import numpy as np
        plt.ion()
        fig=plt.figure()

        x = list()
        y = list()
        i = 0

        ser = serial.Serial('XXXX', 9600) # Replace XXXX with your serial port
        ser.close()
        ser.open()

        while True:

            data = ser.readline()
            
            x.append(i)
            y.append(data.decode())

            plt.scatter(i, float(data.decode()))
            i += 1
            plt.show()
            plt.pause(0.000001)

    </pre>

1. Run the Python code and it should open the specified serial and start plotting the data. As you can tell, it is not the most optimized plotting code. Feel free to improve the plotting.

    ![Figure 6.1](lab6-plot.png)

    ***Figure 6.1***

1. From what you can tell, the data is a bit noisy. Your task is to add the simple IIR filter discussed in class to smooth out the data on the controller. This means adding a new variable called `alpha` and `old_reading` to save the reading for the next iteration.

## Reference

- [Thread](https://os.mbed.com/docs/mbed-os/v6.16/apis/thread.html)
- [Mutex](https://os.mbed.com/docs/mbed-os/v6.16/apis/mutex.html)