# Lab 1 : mbed library

<font size="5">
Seneca College</br>
SEP600 Embedded Systems
</font>

## Introduction

Documentation of the Cortex-M4 instruction set, board user's guide, and the microcontroller reference manual can be found here:

- [Arm Cortex-M4 Processor Technical Reference Manual Revision](https://developer.arm.com/documentation/100166/0001)
- [ARMv7-M Architecture Reference Manual](https://developer.arm.com/documentation/ddi0403/latest/)
- [FRDM-K64F Freedom Module User’s Guide](https://www.nxp.com/webapp/Download?colCode=FRDMK64FUG)
- [Kinetis K64 Reference Manual](https://www.nxp.com/webapp/Download?colCode=K64P144M120SF5RM)
- [FRDM-K66F Freedom Module User’s Guide](https://www.nxp.com/webapp/Download?colCode=FRDMK66FUG)
- [Kinetis K66 Reference Manual](https://www.nxp.com/webapp/Download?colCode=K66P144M180SF5RMV2)

## Preparation

Read over the lab manual for this lab. Acquire the Freedom microcontroller board and install the necessary IDE as described in the lab manual.

> ### Lab Preparation Question
> 1. Read over the lab and understand the procedures.

## Procedures

1. Go to the following link to download the code for a simple blink LED program. This is the code you'll base your lab exercise with. [https://github.com/Seneca-BSA/bsa-sep600](https://github.com/Seneca-BSA/bsa-sep600)
1. Once you've downloaded the code above, unzip it locally.
1. Open MCUXpresso then import the project by File > Import Project from File System or Archive. Find the folder that you've unzipped the code from above then select the folder "lab" for import.
1. Click Finish to import the project.
    <div style="padding: 15px; border: 1px solid orange; background-color: orange; color: black;">
    The code in the project is for a K64F board, ff you are using the Freedom K66F board, the pin configurations is difference. Refer to the Freedom K66F board manual for the correct pin number.
    </div>
1. Open up the main.cpp code and you should see a printf statement along with a few functions that controls the pin direction and the pin state. We are now using the mbed-os library and most of the coding done from this point will be using C++. However, with knowledged of assembly language, you can still program the microcontroller using assembler.
1. Build and debug the code and you should see the LED blink.
1. The API used in the code is the [mbed I/O API](https://os.mbed.com/docs/mbed-os/v6.16/apis/i-o-apis.html). Our baremetal mbed OS might not be at the latest version so some of the features might not work properly.
1. Your task for this lab is to modify the progam to simulate brigthening and dimming of the LED using a concept called PWM but implementing it from a software side. Re-write the code so the LED will turn on for 99 microsecond and off for 1 microsecond. This means your LED is on 99% of the time. Build and debug the program. What do you notice?
1. Change your code so your LED is ON for 50% of the time. What do you notice?
1. Lastly, re-write your program so your LED will loop between 0% and 100% brightness with sufficient delay so we (as human) can see the fading in and out of the LED.

## Reference