# Lab 1 : mbed OS and IDE

<font size="5">
Seneca Polytechnic</br>
SEP600 Embedded Systems
</font>

## Introduction

Documentation of the Cortex-M4 instruction set, board user's guide, and the microcontroller reference manual can be found here:

Documentation of the Freedom K64 and K66 board and it's microcontroller can be found here:

- [FRDM-K64F Freedom Module User’s Guide](https://www.nxp.com/webapp/Download?colCode=FRDMK64FUG) ([PDF](FRDMK64FUG.pdf))
- [Kinetis K64 Reference Manual](https://www.nxp.com/webapp/Download?colCode=K64P144M120SF5RM) ([PDF](K64P144M120SF5RM.pdf))
- [FRDM-K66F Freedom Module User’s Guide](https://www.nxp.com/webapp/Download?colCode=FRDMK66FUG) ([PDF](FRDMK66FUG.pdf))
- [Kinetis K66 Reference Manual](https://www.nxp.com/webapp/Download?colCode=K66P144M180SF5RMV2) ([PDF](K66P144M180SF5RMV2.pdf))

Documentation of the Cortex-M4 instruction set can be found here:

- [Arm Cortex-M4 Processor Technical Reference Manual Revision](https://developer.arm.com/documentation/100166/0001) ([PDF](Cortex-M4-Proc-Tech-Ref-Manual.pdf))
    - [Table of processor instructions](https://developer.arm.com/documentation/100166/0001/Programmers-Model/Instruction-set-summary/Table-of-processor-instructions)
- [ARMv7-M Architecture Reference Manual](https://developer.arm.com/documentation/ddi0403/latest/) ([PDF](DDI0403E_e_armv7m_arm.pdf))

## mbed OS and Library

Mbed OS is an open-source, real-time operating system designed for embedded systems and Internet of Things (IoT) devices. Developed by Arm, it provides a robust platform for building applications that require low power, high performance, and real-time capabilities. Mbed OS offers a rich set of libraries and middleware that simplify the development process, including device drivers, networking stacks, security features, and cloud connectivity. It supports a wide range of microcontrollers and hardware platforms, making it easier for developers to create scalable and reliable solutions for connected devices. With its focus on ease of use and efficiency, Mbed OS helps streamline development for embedded systems in industries like automotive, healthcare, and industrial automation.

The Mbed OS `mbed.h` include hepler functions, objects, and keyword to simplify the development process. Although using the mbed OS is not required for the Freedom K64F and K66F microcontroller board, making use of it will greatly speedup the development cycle.

## Preparation

1. Read over the lab manual for this lab.
1. Acquire a Freedom (K64F or K66F) microcontroller board and install the necessary IDE as described in the lab manual.

## Procedures

Recently, ARM made the [announcement](https://os.mbed.com/blog/entry/Important-Update-on-Mbed/) on retiring the Mbed platform and OS. However, the Mbed codebase will still be useable for simplification of embedded system development. The retiring of Mbed OS also mean limited support on the Mbed Cloud and Mbed Studio IDE. However, ARM is still actively maintaining it's [Keil Studio Cloud](https://studio.keil.arm.com/) and related IDE products to allow the use of the Mbed OS in embedded system development.

1. [Keil Studio Cloud](https://studio.keil.arm.com/) will be the perferred IDE that we are going to use for this course.
    - [Mbed Studio](https://os.mbed.com/studio/) can still be used but are no longer perferred
1. **Using the Chrome Browser**, login to [Keil Studio Cloud](https://studio.keil.arm.com/). Create an Mbed account as necessary with any e-mail address of your choice. Chrome is the most browser for use with Keil Studio Cloud.

    ![Figure 1.1 Keil Studio Cloud Login](lab1-login.png)

    ***Figure 1.1** Keil Studio Cloud Login*

1. After login, you'll see a familar interface that is based on Visual studio Code. Click "+ New project" and select "mbed-os-example-blinky" under "Mbed OS 6" from the Example project menu. Rename the project as "lab1" then **Add project**.

    ![Figure 1.2 Add Project](lab1-add-project.png)

    ***Figure 1.2** Add Project*

1. Open up `main.cpp` from the project tree and you should see a very simple LED blinking code in the IDE. At this point, the code will not compile because we have not defined the "Build target". Select "FRDM-K64F" or "FRDM-K66F" depending on your hardware. All error should go away now.

    ![Figure 1.3 Keil Studio Cloude IDE with Blinky project](lab1-add-project.png)

    ***Figure 1.3** Keil Studio Cloude IDE with Blinky project*

1. Keil Studio Cloud uses the web USB interface to connect with your micrcontroller board. The interface will only work if the Chrome browser have full premission to use your computer's USB port. Plug your microcontroller board into your computer. Once plugged in, under "Connected device", you should see your microcontroller board show up. If your board is not showing up, don't worry, you can still uplpoad your code onto your microcontroller using an alternative method.

1. Mbed OS is an embedded Real-time Operating System (RTOS) that we'll be discussing later in the term. A RTOS, comparing to non-RTOS (or bare metal), allow the operating system to manage pirority and resource to ensure timely execution of each task or thread.

    In the example code:

        int main()
        {
            // Initialise the digital pin LED1 as an output
            DigitalOut led(LED1);

            while (true) {
                led = !led;
                ThisThread::sleep_for(BLINKING_RATE);
            }
        }

    `DigitalOut` define a object called `led` using the `LED1` keyword. This is the pin connected to the red LED.

    The `led` object can then be used directly as a variable to toggle the ON/OFF of the LED.

    `ThisThread::sleep_for();` put the current `main` thread to sleep for 500ms as defined by `BLINKING_RATE`. In later lab, you'll be able to define additional threads.
    
    Below are references to learn more about the code:

    - [Mbed API list](https://os.mbed.com/docs/mbed-os/v6.16/apis/index.html)
    - [FRDM-K64F Mbed Reference](https://os.mbed.com/platforms/FRDM-K64F/)
    - [FRDM-K64F Mbed pinnames](https://os.mbed.com/teams/Freescale/wiki/frdm-k64f-pinnames)
    - [FRDM-K66F Mbed Reference](https://os.mbed.com/platforms/FRDM-K66F/)
    - [FRDM-K66F Mbed pinnames](https://os.mbed.com/teams/NXP/wiki/FRDM-K66F-Pinnames)

1. At this point, there are two ways to upload the code onto your mircocontroller board.

    - **Method 1:** If your browser have full permission to the web USB interface, you'll be able to click the "Run project" (Play) button to build and upload the code. The "Flashing" status will go to 100%.
    - **Method 2:** If you cannot upload directly from your browser, you can click the "Build project" (Hammer) button to compile the code. After build is complete, you'll be offered to save the `.bin` file. Save and copy that file onto the "K64F" or "K66F" directory (from your computer's files explorer) to flash your microcontroller board.

1. Your board should start blinking at 1 Hz.

1. Next, we'll output something through USB serial so we can read it using our computer's serial port. Depending on your browser permission, the Keil Studio Cloud online interface may work. However, to increase robustness, use a terminal software to read serial port. Here are some options:

    - Windows: [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html) or [TeraTerm](https://teratermproject.github.io/index-en.html)
    - Mac/Linux: `screen` or `putty`

1. Add the following line to your code with a logic so everytime when the LED turns ON and *only when the LED turns ON*, the message is printed to the serial port. You can be creative with your message.

        printf("SEP600 is Cool!\n");

    Remember, print *only when the LED turns ON*.

1. Open the serial port that your microcontroller is connected to (ie. COM4 or /dev/ttyACM0, etc. depending on your platform) at baud rate 9600 using PuTTY, screen or a termianl program of your choice. You should see the following being printed:

        SEP600 is Cool!
        SEP600 is Cool!
        SEP600 is Cool!
        ...

    To find which port your microcontroller board is connected to:

    - Windows: open device manager and look under COM port then unplug and re-plug your board to find the COM port
    - Mac/Linux: use `dmesg | grep /dev/tty*` or look in `/sys/class/tty` then unplug and re-plug your board to find the port

    If don't see the output, check your code and ensure you are opening the proper COM port. If problem presist, be resourceful and try to troubleshoot the issue.

    Typical setting for PuTTY:

    ![Figure 1.4 Open COM port with PuTTY](lab1-putty.jpg)

    ***Figure 1.4** Open COM port with PuTTY*

    Typical command for `screen`:
    
        screen /dev/ttyACM0 9600

1. Lastly, add an input logic using `scanf`, `getchar` or any input function so the blinking will only start after a serial input (such as pressing "Enter") from the user.

    Initial Output:

        Press Enter to Start!

    After Enter key pressed:

        Press Enter to Start!
        SEP600 is Cool!
        SEP600 is Cool!
        SEP600 is Cool!
        ...

Once you've completed all the steps above, ask the lab professor or instructor over and demostrate that you've completed the lab. You might be asked to explain some of the concepts you've learned in this lab.

## Reference

- [Keil Studio Cloud](https://studio.keil.arm.com/)
- [Mbed API list](https://os.mbed.com/docs/mbed-os/v6.16/apis/index.html)
- [FRDM-K64F Mbed Reference](https://os.mbed.com/platforms/FRDM-K64F/)
- [FRDM-K64F Mbed pinnames](https://os.mbed.com/teams/Freescale/wiki/frdm-k64f-pinnames)
- [FRDM-K66F Mbed Reference](https://os.mbed.com/platforms/FRDM-K66F/)
- [FRDM-K66F Mbed pinnames](https://os.mbed.com/teams/NXP/wiki/FRDM-K66F-Pinnames)