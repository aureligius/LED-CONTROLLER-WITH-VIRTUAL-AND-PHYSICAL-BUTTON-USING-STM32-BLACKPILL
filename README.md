# LED-CONTROLLER-WITH-VIRTUAL-AND-PHYSICAL-BUTTON-USING-STM32-BLACKPILL

## Contributors:
1. Aurellia Safa Madrim
2. Ursula Maurentti Amarely

## 1. Introduction
This project aims to design and implement an embedded system using STM32 Blackpill by integrating various input and output components. This project is also capable of operating in different modes, including LED sequencing, analog control using potentiometer, sound generation, and wireless control via smartphone and computer vision. Demonstrated the application of GPIO, ADC, PWM, UART communication in real-world systems.

## 2. Component List
* STM32F401CCU6
* ST-Link V2
* Breadboard
* Jumper Wires
* 8 LED Light (4 Red, 2 Blue, 2 Green) 
* Potentiometer
* 2 Tactile Button
* Active Buzzer
* Passive Buzzer
* HC-05 Module
* CH340

## 3. Peripheral Mapping

| Peripheral | STM32 Pin(s) | Mode / Function |
| :--- | :--- | :--- |
| **LED Group 1** | PB9, PB7, PB5, PB3 | Output Push-Pull |
| **LED Group 2** | PA10, PA8, PB14, PB12 | Output Push-Pull |
| **Button 1** | PA0 | Input EXTI Falling Edge (PU) |
| **Button 2** | PA1 | Input EXTI Falling Edge (PU) |
| **Active Buzzer** | PB0 | GPIO Output |
| **Passive Buzzer** | PB1 | AF2 (TIM3_CH4) PWM |
| **Potentiometer** | PA4 | ADC1_CH4 |
| **Serial (Link)** | PA2 (TX), PA3 (RX) | USART2 (HC-05/CH340) |
