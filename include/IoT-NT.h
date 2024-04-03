/*=========================================================================================================
Este archivo contiene redefiniciones de los pines del ESP32 en una tarjeta PCB fabricada 
por Network Telemetrix para fines de monitoreo y control IoT de multiples parametros mediantes sensores.
Especificaciones tecnicas de IoT-board-NT v3.0:
- 4 Digital Input   --> IN_1...IN_4
- 4 Analog Input    --> AN_...AN_4 
- 8 Digital Output  --> RELAY_1...RELAY_4
- 1 SPI Port        --> Pins: MISO, MOSI, CLK, SS, +5V, +3.3V, GND
- 1 I2C Port        --> Pins: SDA, SCL, +5V, GND
- 1 One-Wire Port   --> Pins: 1-Wire, +5V, GND
- 2 PWM Port        --> Pins: GND, -48V, PWM, Tach 
- 1 PWR Port        --> -48VDC
- Vcc Power Supply  --> +5VDC 
- Serial Port       --> Pins: Tx, Rx
- WiFi and Bluetooth connection
La tarjeta PCB ha sido diseñada con componentes en el mercado de larga durabilidad, excelente calidad
y prestaciones, ademas de basar su diseño en un criterio Plug and Play, es decir, Se puede reemplazar
sensores y el controlador (ESP32), lo que convierte a esta tarjeta en un diseño robusto, resistente e 
ideal para su uso en aplicaciones indoor, outdoor y ambientes industriales.
La tarjeta se puede energizar con 2 fuentes de alimentacion: Un puerto de alimentacion de -48VDC 
muy util en aplicaciones industriales y un puerto de alimentacion de +5V para aplicaciones 
residenciales y domatica. Solo se puede seleccionar una de las fuentes a la vez mediante unos 
pines selector de power supply.
A la entrada de la alimentacion de -48VDC la tarjeta cuenta con un circuito de proteccion de 
polarizacion inversa y de corriente de cortocircuito.
Los puertos PWM son adecuados para control de FAN de 48VDC. Estos FANs tambien tienen un circuito 
de proteccion de corriente mediante fusibles de 5A a 6A.
========================================================================================================*/

/*PWM GPIO Pins*/
#define PWM_FAN_1 GPIO_NUM_13
#define PWM_FAN_2 GPIO_NUM_14
#define TACH_FAN_1 GPIO_NUM_2
#define TACH_FAN_2 GPIO_NUM_0
//#define TACH_FAN_2 GPIO_NUM_12

/*Relays Outputs GPIO Pins*/
#define RELAY_OUT_1 GPIO_NUM_17     //Relay OUT1
#define RELAY_OUT_2 GPIO_NUM_16     //Relay OUT2
#define RELAY_OUT_3 GPIO_NUM_4      //Relay OUT3
#define RELAY_OUT_4 GPIO_NUM_15     //Relay OUT4

/*Digital Input GPIO Pins*/
#define IN_1 GPIO_NUM_32      //Digital IN1
#define IN_2 GPIO_NUM_33      //Digital IN2
#define IN_3 GPIO_NUM_25      //Digital IN3
#define IN_4 GPIO_NUM_26      //Digital IN4

/*Analog GPIO Pins*/
#define AN_1 GPIO_NUM_36    //Analog ADC1
#define AN_2 GPIO_NUM_39    //Analog ADC2
#define AN_3 GPIO_NUM_34    //Analog ADC3
#define AN_4 GPIO_NUM_35    //Analog ADC4

/*I2C Communication Pin*/
#define I2C_SDA_PIN GPIO_NUM_21  //For I2C SDA communication Sensor
#define I2C_SCL_PIN GPIO_NUM_22  //For I2C SCL communication Sensor

/*1-WIRE Communication Pin*/
#define ONE_WIRE_PIN GPIO_NUM_27  //For 1-Wire communication Sensor

/*SPI Communication Pin*/
#define CS_PIN GPIO_NUM_5