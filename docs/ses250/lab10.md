# Lab 10 : RLC Circuit Resonance

<font size="5">
Seneca College</br>
SES250 Electromagnatics
</font>

## Purposes
- To examine the AC response of an RLC circuit and find the resonance frequency

## Objectives
- Assemble electronic components onto a breadboard
- Setup function generator to output AC sine wave
- Build an RLC circuit for resonant frequency measurement

## Description

An RLC circuit is an electrical circuit consisting of a resistor (R), an inductor (L), and a capacitor (C), connected in series or in parallel. The circuit forms a harmonic oscillator for current and resonates in a manner similar to an LC circuit. Introducing the resistor increases the decay of these oscillations, which is also known as damping. The resistor also reduces the peak resonant frequency.

An important property of this circuit is its ability to resonate at a specific frequency, the resonance frequency, \(f_{0}\). Its angular frequency alternative, \(\omega_{0}\), is given as:

$$ \omega_{0} = 2 \pi f_0 $$

The resonant frequency is defined as the frequency at which the impedance of the circuit is at a minimum. Equivalently, it can be defined as the frequency at which the impedance is purely real (that is, purely resistive). This occurs because the impedances of the inductor and capacitor at resonant are equal but of opposite sign and cancel out. The resonant frequency for an RLC circuit is:

$$ \omega_{0} = {1 \over { \sqrt{LC}}} $$

Source: [Wikipedia: RLC circuit](https://en.wikipedia.org/wiki/RLC_circuit)

### Reference
- [Series RC Circuit Impedance Calculator](https://www.translatorscafe.com/unit-converter/en-US/calculator/series-rc-impedance/)

## Materials
- Safety glasses (PPE)
- [Lab Supplies](supplies.md)
    - Breadboard
    - Jumper Wires
    - (1x) 10kΩ resistor (brown-black-orange)
    - (1x) 100nF Ceramic Capacitor
    - (1x) 100μH toroidal inductor (supplied by the instructor)

## Preparation

> **Lab Preparation Question:**
>
> 1. Read and summarize the lab as necessary.
> 1. Copy observation Table 1 of this lab into your notebook.
> 1. Sketch a breadboard diagram of Figure 10.1 onto your notebook.
> 1. Calculate the theoretical resonant frequency for a series RLC circuit in AC with a 10kΩ resistor, 100nF capacitor, and the inductor value you used from Lab 9. Show all calculations.

## Procedures

In this lab, we'll observe the AC response of an RLC Circuit and the relationship between the voltage \(V(t)\) and the current \(I(t)\) using an oscilloscope. Since we cannot measure \(I(t)\) directly using an oscilloscope, we'll be measuring the voltage across the resistor \(V_R(t)\) since the current and voltage are always in phase at the resistor.

![Figure 10.1](lab10-rlc-circuit.png)

***Figure 10.1***

1. Set up the circuit in Figure 10.1 using a 10kΩ resistor, a 100nF ceramic capacitor and 47μH (or the inductor you used in Lab 9).
    <div style="padding: 15px; border: 1px solid red; background-color: red; color: white;">
    <p style="font-size: 18px"><strong>DO NOT USE A POLARIZED CAPACITOR!</strong><p>
    </div>
1. Ensure that the function generator’s output is off then set the output of the function generator to High-Z.
1. Set the output waveform to be a sine wave then set the output amplitude to 6 Vpp and the frequency to 10.0 Hz. Leave offset voltage and phase at 0.
1. Turn on the oscilloscope then connect CH1 and CH2 to the circuit per the circuit diagram.

    **NOTE:** The ground (black cable) for both CH1 and CH2 is connected to the negative (black) node of the function generator.

1. Turn on the function generator output and observe the relationship between CH1 and CH2. Adjust the voltage and time division to see about two periods on the display.

1. From the two signals that you see, find a peak from CH1 (measuring \(V(t)\)) and the closest peak from CH2 (measuring \(V_R(t) = I(t)\)).

    > **Lab Question 1:** Find the time difference between the two signals. You may use the horizontal position knob to move one of the signals to the centre for easier measurement. Afterward, convert this time difference you found to the phase angle difference by using the formula below. Does it agree with the phase angle difference you calculated in your pre-lab? Write your result in Table 1.
    >
    > $$ \Phi = 2 \pi {\Delta t \over T} $$
    >
    > **NOTE:** If \(I(t)\) is leading, the time difference is a negative time. Also, the time difference might be in ms and the phase angle difference might be in RAD depending on your setting.
    >
    >   **Table 1:**  
    > 
    >   |Freq|Theoretical \(Phi\)|Measured \(V_{Rpp}\)|\(\Delta T\)|\(\Phi\)|\(\|I\|\)|\(\|Z_T\|\)|
    >   |---|---|---|---|---|---|---|
    >   |10 Hz|
    >   |500 Hz|
    >   |1 kHz|
    >   |5 kHz|
    >   |10 kHz|
    >   |50 kHz|
    >   |100 kHz|
    >   |500 kHz|
    >   |...|
    >
    >   - \(\Phi = 2 \pi {\Delta t / T} = 360° (\Delta t) f\)

1. Calculate the magnitude of the current \(\|I\|\) and magnitude of the total impedance \(\|Z_T\|\).
    
    >   **Lab Question 2:** Using the measured peak-to-peak \(V_{R}\), calculate the magnitude of the current \(\|I\|\) and magnitude of the total impedance \(\|Z_T\|\). Record your answers in Table 1.
    >
    >   - \(\|I\| = V_R / R\)
    >   - \(\|Z_T\| = V_S / \|I\|\)

1. Open a spreadsheet software and plot the data.
    
    >   **Lab Question 3:** Using the data from Table 1, open a spreadsheet software and plot the following:
    >
    >   1. Phase Shift vs Frequency
    >   1. Magnitude of the Current \(\|I\|\) vs Frequency
    >   1. Magnitude of the Total Impedance \(\|Z_T\|\) vs. Frequency

1. Repeat the measurement at more frequencies until a minimum phase shift can be found.

    >   **Lab Question 4:** Using the additional data you recorded onto Table 1 and your plots:
    >
    >   1. Find the frequency at which \(\Phi = 0°\).
    >   1. Find the frequency at which \(\|I\|\) has a maximum value.
    >   1. Find the frequency at which \(\|Z_T\|\) has a minimum value.

Once you've completed all the above steps, ask the lab professor or instructor over and demostrate that you've completed the lab and written down all your observations. You might be asked to explain some of the concepts you've learned in this lab.