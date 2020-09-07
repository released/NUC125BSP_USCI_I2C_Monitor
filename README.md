# NUC125BSP_USCI_I2C_Monitor
 NUC125BSP_USCI_I2C_Monitor

update @ 2020/09/07

1. base on sample code : USCI_I2C_Monitor , 

2. Add define ENABLE_NUC125 , ENABLE_NUC126

- UART : TX (NUC125 : PB1 , NUC126 : PD.1) , RX (NUC125 : PB.0 , NUC126 : PD0)

- I2C0 MASTER : SCL (NUC125 : PC12 , NUC126 : PA3) , SDA (NUC125 : PC11 , NUC126 : PA2)

- I2C1 SLAVE :  SCL (NUC125 : PA11 , NUC126 : PA8) , SDA (NUC125 : PA10 , NUC126 : PA9)

- USCI MONITOR : SCL (NUC125 : PC0 , NUC126 : PC4) , SDA (NUC125 : PC3 , NUC126 : PC5)

3. Use terminal to check master TX and monitor data , press any key to increase each master TX data index

4. below is capture screen ,

![image](https://github.com/released/NUC125BSP_USCI_I2C_Monitor/blob/master/log.jpg)

![image](https://github.com/released/NUC125BSP_USCI_I2C_Monitor/blob/master/log2.jpg)