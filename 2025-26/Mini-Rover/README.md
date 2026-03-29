# UBC Rover 2025-26 Mini Rover Project

STM32-based motor controller for a 6-wheeled rover, 
communicating with a Jetson Nano over UART.

## Hardware Requirements

| Component         | Spec                              | Quantity |
|-------------------|-----------------------------------|----------|
| Microcontroller   | STM32F446RE Nucleo-64             | 1        |
| Motors            | 36mm DC Planetary, 12V 95RPM      | 6        |
| Motor Driver      | L298N                             | 3        |
| Computer          | Jetson Nano                       | 1        |

## Nucleo Pin Configuration

| Timer (PWM) | Left (TIM1) |        | Right (TIM3) |        |
|-------------|-------------|--------|--------------|--------|
| Front (Ch1) | PA8         | D7     | PA6          | D12    |
| Mid (Ch2)   | PA9         | D8     | PA7          | D11    |
| Back (Ch3)  | PA10        | D2     | PB0          | A3     |

| Motor       | Direction | Left Pin | Left CN | Right Pin | Right CN |
|-------------|-----------|----------|---------|-----------|----------|
| Front       | Forward   | PC0      | A5      | PC6       | D15      |
|             | Backward  | PC1      | A4      | PC7       | D9       |
| Mid         | Forward   | PC2      | CN7-35  | PC8       | CN10-2   |
|             | Backward  | PC3      | CN7-37  | PC9       | CN10-1   |
| Back        | Forward   | PC4      | CN10-34 | PC10      | CN7-1    |
|             | Backward  | PC5      | D14     | PC11      | CN7-2    |

## Serial Command Reference

Commands sent over UART at 115200 baud, terminated with `\n`.

| Command      | Example           | Motor ID Options        | Description                  |
|--------------|-------------------|-------------------------|------------------------------|
| $set_speed() | $set_speed(0, 70) | 0-5, L, R, A, S         | Set motor speed (-100 to 100)|
| $get_speed() | $get_speed(0)     | 0-5                     | Read RPM over 100ms window   |

Motor ID key: `0-5` = individual motors, `L` = all left, `R` = all right, 
`A` = all motors