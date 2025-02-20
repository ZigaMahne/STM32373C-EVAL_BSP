# STM32373C-EVAL Evaluation board

## Overview

The STM32373C-EVAL evaluation board is designed as a complete demonstration and development platform for STMicroelectronics’ ARM cortex-M4 core-based STM32F373VCT6 microcontroller. It features two I2Cs, three SPIs, three USARTs, one CAN, one CEC controller, one 12-bit ADC, three 16-bit sigma delta ADCs, three 12-bit DACs, internal 32-KByte SRAM and 256-KByte Flash, touch sensing, USB FS, and JTAG debugging support. This evaluation board can be used as a reference design for user application development but it is not considered as the final application.

The full range of hardware features on the board can help the user evaluate all peripherals (USB FS, USART, audio DAC, microphone ADC, color LCD, IrDA, LDR (light-dependent resistor), MicroSD card, HDMI CEC, ECG (electrocardiogram), pressure sensor, CAN, IR (infrared) transmitter and receiver, EEPROM, touch slider, temperature sensor, etc.) and develop their own applications. Extension headers make it possible to easily connect a daughterboard or wrapping board for a specific application.

An ST-LINK/V2 is integrated on the board as an embedded in-circuit debugger and programmer for the STM32 MCU.

## Getting started

- [User manual](https://www.st.com/resource/en/user_manual/um1564-stm32373ceval-evaluation-board-stmicroelectronics.pdf)

### ST-LINK driver installation and firmware upgrade (on Microsoft Windows)

1. Download the latest [ST-LINK driver](https://www.st.com/en/development-tools/stsw-link009.html).
2. Extract the archive and run `dpinst_amd64.exe`. Follow the displayed instructions.
3. Download the latest [ST-LINK firmware upgrade](https://www.st.com/en/development-tools/stsw-link007.html).
4. Extract the archive and run the `ST-LinkUpgrade.exe` program.
5. Connect the board to your PC using a USB cable and wait until the USB enumeration is completed.
6. In the **ST-Link Upgrade** program, press the **Device Connect** button.
7. When the ST-LINK driver is correctly installed, the current ST-LINK version is displayed.
8. Press the **Yes >>>>** button to start the firmware upgrade process.

## Technical reference

- [STM32F373VC microcontroller](https://www.st.com/en/microcontrollers-microprocessors/stm32f373vc.html)
- [STM32373C-EVAL board](https://www.st.com/en/evaluation-tools/stm32373c-eval.html)
- [User manual](https://www.st.com/resource/en/user_manual/um1564-stm32373ceval-evaluation-board-stmicroelectronics.pdf)
- [Data brief](https://www.st.com/resource/en/data_brief/stm32373c-eval.pdf)
- [Schematic](https://www.st.com/resource/en/schematic_pack/stm32373c-eval_sch.zip)
