## Install Keil
[link](https://www2.keil.com/mdk5)  

## Install STM32 tools
[STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)  

## Clone code
`git clone --recursive  --depth 1 --branch v1.11.5 https://github.com/STMicroelectronics/STM32CubeF0.git`
`git clone https://github.com/eiffelpeter/stm32_mpu6050_dumbbell.git`  


## Connect stm32 nucleo-F030R8 and mpu6050
![IMAGE ALT TEXT HERE](./img/IMG_2681.jpg)  

## Pin connection for test
| Item | pin | 
|---------|-----|
| I2C SCL | PB6 |
| I2C SDA | PB7 |

## Generate code by STM32CubeMX
  open `stm32_mpu6050_dumbbell.ioc` then generate code.  ( select NO download firmware package )

## Build and program it
  open `stm32_mpu6050_dumbbell.uvprojx` by Keil.  
  build and program stm32.  

## Open console log
  baud rate 115200.  
  ![IMAGE ALT TEXT HERE](./img/console_log.jpg)  
