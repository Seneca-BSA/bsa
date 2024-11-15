# Lab 8 : GPIO Input and Code Optimization

<font size="5">
Seneca Polytechnic</br>
SEH500 Microprocessors and Computer Architecture
</font>

## Introduction

Documentation of the Cortex-M4 instruction set can be found here:

- [Arm Cortex-M4 Processor Technical Reference Manual Revision](https://developer.arm.com/documentation/100166/0001) ([PDF](Cortex-M4-Proc-Tech-Ref-Manual.pdf))
    - [Table of processor instructions](https://developer.arm.com/documentation/100166/0001/Programmers-Model/Instruction-set-summary/Table-of-processor-instructions)
- [ARMv7-M Architecture Reference Manual](https://developer.arm.com/documentation/ddi0403/latest/) ([PDF](DDI0403E_e_armv7m_arm.pdf))

Documentation of the Freedom K64 and K66 board and it's microcontroller can be found here:

- [FRDM-K64F Freedom Module User’s Guide](https://www.nxp.com/webapp/Download?colCode=FRDMK64FUG) ([PDF](FRDMK64FUG.pdf))
- [Kinetis K64 Reference Manual](https://www.nxp.com/webapp/Download?colCode=K64P144M120SF5RM) ([PDF](K64P144M120SF5RM.pdf))
- [FRDM-K66F Freedom Module User’s Guide](https://www.nxp.com/webapp/Download?colCode=FRDMK66FUG) ([PDF](FRDMK66FUG.pdf))
- [Kinetis K66 Reference Manual](https://www.nxp.com/webapp/Download?colCode=K66P144M180SF5RMV2) ([PDF](K66P144M180SF5RMV2.pdf))

In this last lab of the course, we'll use program in assembly language and C programming language interchangeablely and explore the advantage and disadvantage of using a high-level language.

## Procedures

### Reading Input using Polling

1. First, let's setup out project in a similar manner as Lab 7. Create a new project ensure the following driver is available in addition to the default settings.

    - pit

    If there are missing driver and SDK components from your project, you may add them at any time by clicking **Manage SDK Component** at the top of the project explorer or **Right click on the project > SDK Management > Manage SDK Component**.

1. Use the **ConfigTools > Config Tools Overview** or right click on your project in the **Project Explorer** and Open the **MCUXpresso Config Tools > Open Tools Overview** windows to enable he following:

    Under Pins > Functional groups, ensure the followings are enabled:

    - BOARD_InitPins
    - BOARD_InitBUTTONsPins
    - BOARD_InitLEDsPins

    By enabling the above, you are updating the clock gate and similar settings so their respective port can be used.

1. Leave all the `BOARD_Init...` code as is then replace the remaining codes with the following:

        /* setup the switch pin config as output per fsl_gpio.h */
        gpio_pin_config_t sw_config = {
            kGPIO_DigitalInput,
            0,
        };

        /* setup the led pin config as output per fsl_gpio.h */
        gpio_pin_config_t led_config = {
            kGPIO_DigitalOutput,
            0,
        };

        /* setup the switch and led pins as per fsl_gpio.h */
        /* the gpio and pin keyword can be found in board.h */
        GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &sw_config);
        GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, &led_config);
        LED_RED_OFF(); // ensure the red led is off
        
        PRINTF("\r\n --- Program Start --- \r\n");
        
        while(1) {

            /* we'll use polling instead of interrupt to listen for button press (LOW) */
            if (GPIO_PinRead(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN) == 0) {
                // button pressed
                LED_RED_ON();
                delay(); // delay so we can see the LED turn on
                LED_RED_OFF();
            }

        }

        return 0 ;

    Also add the following `delay()` function to your code:

        void delay(void)
        {
            volatile uint32_t i = 0;
            /* at 120MHz, 120,000,000 instructions = 1s */
            for (i = 0; i < 120000000; ++i)
            {
                __asm("NOP"); /* delay */
            }
        }

    Now that we know how to program the K64 (or K66) processor using assembly, we can use some of the built-in functions and keywords from the SDK to help us with programming the board. There are no compenhensive manual on all the helper functions and keywords aviable. The best place to look is their respecitve library file. This also allow us to write one set of code for multiple processors as long as the helper functions and keywords are available. 

1. Build and Debug, then Run he code using **Resume (F8)**. Press switch 2 and see the red LED turn on and off.

1. What happens when you press the switch when the LED is ON? How is the approach of using polling to read input differ then using interrupt? Which one is better? Submit your answer on Blackboard.

### Code Optimization

In class, we discussed various ways on code optimizing. One key thing to remember is shorter code does not means faster code.

A Taylor Series expansion is a way of finding the approximiate value of a function. The Taylor Series expansion of a Sine function is given as follow:

$$
sin(x) = x - \frac{x^3}{3!} + \frac{x^5}{5!} - \frac{x^7}{7!} + ...
$$

Each term is generalized to:

$$
\textrm{Term}_n = (-1)^n \frac{x^{(2n + 1)}}{(2n + 1)!}
$$

1. Go to **ConfigTools > Peripherals** from the top menu. In the "Components" tab, Under "Peripheral drivers (Device specific)" add the "PIT" configuration components. Set up the PIT at **1us** (one microsecond interval).

1. Add a Global variable `loop_timer` and the PIT ISR at the end of your code. We'll use them to measure execution time.

        volatile static long timer_counter = 0;

    PIT ISR:

        void PIT_CHANNEL_0_IRQHANDLER(void) /*ISR to process PIT channel 0 interrupts*/
        {
            PIT_ClearStatusFlags(PIT, PIT_CHANNEL_0, kPIT_TimerFlag);
            //clear PIT channel 0 interrupt status flag
            timer_counter++;
        }

1. Next, we have a few helper functions to help calculate the Taylor Series expansion.

        /* factorial calculation */
        int factorial(int num) {
            int result = 1;
            int i = 0;
            for (i = 1; i <= num; i++) {
                result *= i;
            }
            return result;
        }

        /* power calculation */
        float pow(float base, int exp) {
            float result = 1.0;
            int i = 0;
            for (i = 0; i < exp; i++) {
                result *= base;
            }
            return result;
        }

        /* each taylor series term */
        float taylor_term(float x, int n) {
            float result = 0.0;
            int degree = n * 2 + 1;
            result = pow(x, degree) / factorial(degree);
            result *= (n % 2 == 0 ? 1.0 : -1.0);
            return result;
        }

        /* taylor series approximation */
        float sin_taylor(float x) {
            float result = 0.0;
            int degree = 11;
            int terms = (degree + 1) / 2;
            int i = 0;
            for (i = 0; i < terms; i++) {
                result += taylor_term(x, i);
            }
            return result;
        }

1. Next, change all of the from the `printf` statement onward to:

        float x = 0.0;
        float sin_x = 0.0;
        const float pi = 3.1416;
        int xs[4000] = {0};
        int sin_xs[4000] = {0};
        int end = 0;
        int i = 0;
        long elasped_time = 0;

        PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);

        while(1) {

            if (GPIO_PinRead(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN) == 0) {
                LED_RED_ON();

                PRINTF("\r\n --- SIN Calculation Start --- \r\n");

                x = 0.0;
                i = 0;
                timer_counter = 0; // reset timer counter and start

                while (x < pi) {
                    sin_x = sin_taylor(x); // calculate sin(x) using Taylor Series
                    /* save everything as integer of (x1000) to make it faster for print*/
                    xs[i] = (int)(x*1000.0);
                    sin_xs[i] = (int)(sin_x*1000.0);
                    i++;
                    x += (pi / 1000.0);
                }

                elasped_time = timer_counter; // timer end

                end = i;

                for (i = 0; i < end; i++) {
                    PRINTF("x = %d, sin_x = %d\r\n", xs[i], sin_xs[i]);
                }

                PRINTF("\r\n --- SIN Calculation End --- \r\n");
                PRINTF("\r\n --- time = %ld --- \r\n", elasped_time);

                LED_RED_OFF();
            }
        }
        return 0;

1. Build and Debug, then Run he code using **Resume (F8)**. Press switch 2 and see the red LED turn on and off. Everytime when you press switch 2, \(sin(x)\) calculation from \(0\) to \(\pi\) are calculated at one-thousand intervals. If you open the **Serial Monitor (Terminal)**, it'll say the execution took about 100ms.

    The output should be similar to this:

        ...
        x = 3122, sin_x = 18
        x = 3125, sin_x = 15
        x = 3129, sin_x = 12
        x = 3132, sin_x = 8
        x = 3135, sin_x = 5
        x = 3138, sin_x = 2

        --- SIN Calculation End --- 

        --- time = 109995 ---

## Post-Lab Questions

Using the skills and knowledge acquired from this lab, answer the following post-lab question(s) on Blackboard. Due one week after the lab.

1. How is the approach of using polling to read input differ then using interrupt? Which one is better? Submit your answer on Blackboard.

1. The Taylor Series expansion code is written in a manner that's modular and somewhat easy to understand but not in an optimized manner in terms of fast execution. Using techniques that you've learned in class and your knowledge of assembly instruction, modify the code between timer start and timer end, including the helper function and any related variables, so it'll take less then 80ms (20% improvement) to execute the code. For every change and optimization you made, provide a brief reason on how it improve execution time.

    Hints and limiations:

    - You may modify the code in C or re-write the code in assembly or use inline assembly
    - You may add or remove any variables or functions
    - You may perform pre-calculation as you see fit 
    - You cannot use a SIN function lookup table

    Paste your code, explanation of every changes, and a screenshot of the terminal output on blackboard. The output value after optimization should be very similar to the value after optimization.

    **Challenge:** Achieve at least 50% improvement in speed.

## Reference

[1] Yiu, J. (2013). The Definitive Guide to ARM® Cortex®-M3 and Cortex®-M4 Processors. (3rd ed.). Elsevier Science & Technology.
