# Lab 6 : LCD and Interrupt

<font size="5">
Seneca Polytechnic</br>
SEP600 Embedded Systems
</font>

## Introduction

Documentation for the Cortex-M4 instruction set, the board user's guide, and the microcontroller reference manual can be found here:

Documentation for the Freedom K64 and K66 boards and their microcontrollers can be found here:

- [FRDM-K64F Freedom Module User’s Guide](https://www.nxp.com/webapp/Download?colCode=FRDMK64FUG) ([PDF](FRDMK64FUG.pdf))
- [Kinetis K64 Reference Manual](https://www.nxp.com/webapp/Download?colCode=K64P144M120SF5RM) ([PDF](K64P144M120SF5RM.pdf))
- [FRDM-K64F Mbed Reference](https://os.mbed.com/platforms/FRDM-K64F/)
- [FRDM-K64F Mbed Pin Names](https://os.mbed.com/teams/Freescale/wiki/frdm-k64f-pinnames)
- [FRDM-K66F Freedom Module User’s Guide](https://www.nxp.com/webapp/Download?colCode=FRDMK66FUG) ([PDF](FRDMK66FUG.pdf))
- [Kinetis K66 Reference Manual](https://www.nxp.com/webapp/Download?colCode=K66P144M180SF5RMV2) ([PDF](K66P144M180SF5RMV2.pdf))
- [FRDM-K66F Mbed Reference](https://os.mbed.com/platforms/FRDM-K66F/)
- [FRDM-K66F Mbed Pin Names](https://os.mbed.com/teams/NXP/wiki/FRDM-K66F-Pinnames)

Documentation for the Cortex-M4 instruction set can be found here:

- [Arm Cortex-M4 Processor Technical Reference Manual Revision](https://developer.arm.com/documentation/100166/0001) ([PDF](Cortex-M4-Proc-Tech-Ref-Manual.pdf))
    - [Table of Processor Instructions](https://developer.arm.com/documentation/100166/0001/Programmers-Model/Instruction-set-summary/Table-of-processor-instructions)
- [ARMv7-M Architecture Reference Manual](https://developer.arm.com/documentation/ddi0403/latest/) ([PDF](DDI0403E_e_armv7m_arm.pdf))

### 16×2 (or other size) LCD Module

A 16×2 Liquid Crystal Display module has 16 columns and 2 rows of characters. Most 16×2 (as well as 8×1, 20×2, and 20×4) LCD modules use a Hitachi HD44780 (or compatible) LCD controller. Each character space can display a single alphanumeric character, symbol, or custom character. The display operates by selectively controlling the liquid crystal pixels, making them opaque or transparent to create characters or graphics. The LCD controller works in 2 main modes: 

- **4-bit Mode:** Data is sent to the LCD module in two consecutive nibbles. The higher nibble, consisting of data lines D4 to D7, is sent first, followed by the lower nibble with data lines D0 to D3. This configuration allows us to send 8-bit data using only four data lines, conserving valuable I/O pins on the microcontroller.
- **8-bit Mode:** The LCD can directly receive 8-bit data in a single transmission, using all eight data lines (D0 to D7). As a result, this mode offers faster and more efficient data transfer compared to the 4-bit mode. However, it requires more I/O pins on the microcontroller, potentially limiting its application in projects with limited resources.

When using the LCD module in 4-bit mode, only 4 wires are required for parallel data transfer, plus 2 wires for enable. However, with the help of an I2C parallel port expander (I2C backpack), only 2 wires through I2C are required to work with the LCD module.

![Figure 6.1](lab6-lcd.png)

***Figure 6.1** 16x2 LCD*

Some common commands are:

| Command Name | HEX Value | Description |
|---|---|---|
| Clear Display | 0x01 | This command clears the entire display, resetting the cursor position to the home position (0, 0). |
| Return Home | 0x02 | Sending this command moves the cursor to the home position (0, 0) without clearing the display. |
| Entry Mode Set | 0x04 | This command determines the cursor movement direction and whether the display should shift. |
| Display On/Off Control | 0x08 | This command controls the display, cursor, and cursor blinking options. |
| Cursor or Display Shift | 0x10 | Used to shift the cursor or the entire display left or right without changing the display data. |
| Function Set | 0x20 | This command sets the LCD data length (4-bit or 8-bit), number of display lines, and font size. |
| Set CGRAM Address | 0x40 | This command sets the address of the Character Generator RAM (CGRAM) for custom character creation. |
| Set DDRAM Address | 0x80 | This command sets the address of the Display Data RAM (DDRAM), allowing data to be written to a specific location on the LCD. |

Reference: [HD44780 LCD Controller](https://en.wikipedia.org/wiki/Hitachi_HD44780_LCD_controller)

## Materials
- Safety glasses (PPE)
- Freedom K64F or K66F Board
- Breadboard
- Jumper Wires
- LCD Display (Parallel or I2C)
- Various 1kΩ–10kΩ resistors
- Button

**If you are using an I2C LCD, connect the LCD to the I2C pins and use the I2C library instead of the parallel LCD library.**

## Preparation

> ### Lab Preparation Question
> 1. Read over the lab and understand the procedures.

## Procedures

### Part 1: LCD Module

1. Acquire an LCD and a resistor, then connect them to the Freedom K64F/K66F board as per the connection table and the diagram below. If you are using an I2C LCD, connect the LCD to the I2C pins.

    ![Figure 6.2](lab6-lcd-connection.png)

    ***Figure 6.2** LCD connection with Freedom board*

    The typical pinout and connection for a 16x2 LCD are given below. Please keep in mind that depending on the manufacturer, some labels and configurations may vary.

    | LCD Pin # | LCD Label | K64F/K66F Pin |
    |---|---|---|
    | 1 | GND / VSS | GND / 0V |
    | 2 | VDD / VCC | 5V |
    | 3 | VO | 1kΩ to GND / 0V |
    | 4 | RS | D9 |
    | 5 | R/W | GND / 0V |
    | 6 | E | D8 |
    | 7 | DB0 | N/C |
    | 8 | DB1 | N/C |
    | 9 | DB2 | N/C |
    | 10 | DB3 | N/C |
    | 11 | DB4 | D4 |
    | 12 | DB5 | D5 |
    | 13 | DB6 | D6 |
    | 14 | DB7 | D7 |
    | 15 | LED+ | 1kΩ to 5V |
    | 16 | LED- | N/C |

    - Some models work with 3.3V instead of 5V.
    - VO pin configuration varies depending on the manufacturer. A potentiometer can be used instead of a 1kΩ resistor for adjustable contrast.

    You may change the pins used on the K64F/K66F board depending on your application and pin availability.

2. Open Keil Studio and install the following library to your project depending on whether you are using the Parallel or I2C version of the LCD.

    - Parallel LCD: https://os.mbed.com/users/sstaub/code/mbedLCD/
    - I2C LCD: https://os.mbed.com/users/sstaub/code/mbedLCDi2c/

3. Use the following code to output a message on the display:

        #include "mbed.h"
        #include "LCD.h"

        LCD lcd(D9, D8, D4, D5, D6, D7, LCD16x2); // RS, EN, D4-D7, Type
 
        int main() {

            lcd.cls(); // clear display
            lcd.locate(0, 0); // set cursor location
            lcd.printf("SEP600\n"); // display text
            ThisThread::sleep_for(2s);
            lcd.cls(); // clear display
            lcd.locate(0, 0); // set cursor location
            lcd.printf("Hello World!\n"); // display text

        }

4. After uploading your code, the LCD should show "SEP600" for 2 seconds, then "Hello World!".

    > **Lab Question:** Modify your code to display your name and student number on row 1, and your lab partner's name and student number on row 2 (or be creative, like "SEP600 Embedded Systems is Awesome"). Since the message will be too wide for the LCD, display the text as a horizontal scrolling message at a reasonable rate.
    >
    > **Hint:** There are many ways to do this. Refer to the library documentation on how to move the print cursor.

5. If you are using an I2C LCD module, use the oscilloscope to see the I2C data frame. You should be able to see the message that you are sending to the module, such as 83 in binary for the letter "S".

### Part 2: Interrupt

An interrupt is a way for the microcontroller to listen for events without continuously polling the input. Most of the GPIO pins on the Freedom K64F or K66F board can be attached to an interrupt.

1. Connect a pull-up or pull-down button to any digital pin of your choosing.

2. Add the following code before `main()` to create an interrupt object.

        InterruptIn button(PTXX);

3. Add the following interrupt routine before `main()` and include the appropriate code for displaying a message on the LCD when the interrupt is triggered. Display the message for a few seconds, then return to displaying the previous message.

        void button_isr(){
            // Display an interrupt message on the LCD
            // Use wait_us for delay
            // Do NOT use ThisThread::sleep_for
        }

    > **Hint:** You can try triggering a flag in the interrupt and then display the message in the `main()` loop or in a separate thread.

4. Within `main()`, attach the interrupt routine to the button and adjust for the rising or falling edge, depending on your circuit configuration.

        button.rise(&button_isr);

5. Upload and test your interrupt.

Once you've completed all the steps above (and ONLY when you are ready, as you'll only have one opportunity to demo), ask the lab professor or instructor to come over and demonstrate that you've completed the lab. You may be asked to explain some of the concepts you've learned in this lab.

## Reference

- [InterruptIn](https://os.mbed.com/docs/mbed-os/v6.16/apis/interruptin.html)