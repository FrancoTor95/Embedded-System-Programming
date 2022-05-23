# Brushed DC Motors and Servo Motors

***
</br>

## Overview 
This lecture shows few examples usage of two main ***D**irect **C**urrent* (DC) motors: the *brushed motors*  and the *servo motors*. Both require a control that passes through the use of appropriate *PWM* signals and both are widely used as actuators when controlling embedded systems.

* [Brush DC Motors (*BDCM*)](#Motori-CC-a-Spazzole-(BDCM))
  * [H bridge // L9110](#Ponte-H-//-L9110)
  * [Control of a brush motor](#Controllo-di-un-motore-a-spazzole)
* [Servo Motors](#Servomotori)
  * [Control of a servomotor](#Controllo-di-un-servomotore)

</br></br>

## Brushed DC Motors (*BDCM*)
<img align="left" src="img/brushed-dc.gif" width="30%">
They are used in many industrial applications, in contexts related to the automotive world and also in many applications related to the world of robotics; not to mention the various uses in everyday devices. These are actuators that, in general, are found at a relatively low price and are easy to use even in embedded control systems.

These motors are operated by means of two terminals. A potential difference is applied in order move the *rotor*, which will rotate at a speed proportional to the applied voltage. The direction of rotation depends on the polarity of the voltage. For more details and insights regarding the equations that define these *electrical machines*, please refer to specific texts.

<p style="margin-bottom:3%"></p>

In applications where it is necessary to control a DC motor, the parameter which is mainly concerned is the speed of the motor and, if given, the direction of rotation. For simple applications it is sufficient to supply an adequate voltage to the motor which, as known, is directly proportional to its rotation speed. For the development of more efficient applications, in which perhaps it is also necessary to vary the direction of rotation and a more accurate speed control is required, *PWM* signals are used. Before reaching the motor these signals are adjusted and elaborated by an appropriate  dedicated circuits called *driver*. Obviously, the *PWM* signal will be used by varying its *duty-cycle* so as to vary the average value of the signal arriving at the motor.

In their basic form, these circuits are composed of at least one resistor and a transistor, but to avoid overheating problems and improve the system's capabilities, a particular *driver* is generally used which consists of 4 transistors suitably connected to each othe and with the engine itself. This circuit, due to its graphic schematization, is called *H-bridge* (*H-bridge* or *full-bridge*).

<p align="center">
  <img src="img/h-bridge.png" width="60%">
</P>

Depending on which transistors are active, different "paths" are created for the current, allowing in a simplified way, the polarity inversion of the voltage across the motor, placed in the center.

The control of the motor therefore passes through the generation of the signals that control the transistors of the *driver*, the *H bridge*.

#### H bridge // L9110
We take as an example, the *H bridge* [**L9110**](docs/L9110-datasheet.pdf), an integrated cicuit which realizes the *driver* in question and allows the control of motors that work at different voltage levels, from 2.5V to 12V. In the following it will be used encapsulated inside a module that provides, in a simplified way, the possibility of connection with motors and microcontroller. In particular, this module allows the connection and control of two motors simultaneously: each motor is managed by two control pins.

<p align="center"> 
  <img src="img/l911smodule.png" width="30%">
</p>

Generally, the pins are labelled according to their usage:

| Pin |Description|
| :-: | :-:            |
|B-IA	|Motor B Input A |
|B-IB	|Motor B Input B |
|GND	|Ground          |
|VCC	|Voltage  2.5-12V|
|A-IA	|Motor A Input A |
|A-IB	|Motor A Input B |

The relationship between the configuration of the module inputs and the output result, on the pins of the *H bridge*, is direct. Expressed in terms of logical voltage values ( low (**L**) and high (**H**)) this relation is illustrated in the following table:.

|Input (IA // IB) |	Output (OA // OB) | Resulting motor action|
| :-: |  :-:  | :-: |
|L // L	| L // L	|Off| 
|H // L	| H	// L	|Forward| 
|L // H	| L	// H	|Reverse| 
|H // H	| H	// H	|Off| 

#### :dart: PWM control signal
The speed of the motor is directly proportional to the voltage applied to its pins. Working with a *PWM* signal, the voltage to refer to will be the average value of the signal, which corresponds to the *DC component* of the signal. It is therefore important to choose the *PWM* signal so that its frequency is high enough and has a higher value than the *cut-off frequency* of the system describing the motor.

The transfer function that describes a brushed DC motor can generally be seen as:

<p align="center">
  <img src="https://render.githubusercontent.com/render/math?math=%5Cfrac%7B%5Comega(s)%7D%7BV(s)%7D%20%3D%20%5Cfrac%7BK%7D%7B%5Ctau%20s%2B1%7D" width="13%">
</p>

which is the characteristic transfer function of a first order system (a low pass filter, like any real system) with a cutoff frequency equal to:

<p align="center">
  <img src="https://render.githubusercontent.com/render/math?math=f_c%20%3D%20%5Cfrac%7B1%7D%7B2%5Cpi%5Ctau%7D" width="10%">
</p>

If the frequency of the *PWM* signal is above this, then it will be "ignored" by the system and the only component that will be taken into consideration will be the DC component, which corresponds precisely to the average value of the *PWM* signal.

Only by choosing an adequate frequency for the *PWM* signal will it be possible to control the motor properly, without it undergoing sudden acceleration changes. Ultimately, the parameter to know is the *motor time constant*, which is generally a parameter that can be found on the datasheet of the motor you are using.

Basically, a good choice of the frequency respects the following inequality

<p align="center">
  <img src="https://render.githubusercontent.com/render/math?math=f_%7BPWM%7D%20%5Cge%20%5Cfrac%7B5%7D%7B2%5Cpi%5Ctau%7D" width="13%">
</p>

### Brushed DC Motor Control
An example of control of a *BDCM*, via *H bridge*, can be achieved with *STM32Cube* by controlling the motor speed via a potentiometer: this is read via *ADC* and the input value is mapped on a scale of 0-100, which will correspond to the *duty-cyle* value of the *PWM* signal that will control one of the two inputs of the *H bridge H*.

<p align="center">
  <img src="img/dcmotor_hbridge_scheme.png" width="70%">
</p>

For the configuration of *ADC* and *Timer* for the generation of the *PWM* signal, the graphic environment *CubeMX* is used. The configuration consists of:

* **ADC1**
  * Input: `IN1`
  * Resolution: `8 bit` 
  * Continuous mode: `DISABLE`
  * Other: *default*
* **TIM2**
  * Clock source: `Internal Clock`
  * Channel: `PWM Generation CH1`
  * Prescaler: `84-1`
  * Counter: `100-1`
  * Other: *default*


<p align="center">
  <img src="img/motor_control_MXconfig.png">
</p>

It is important to make sure that the frequency of the *PWM* signal is high enough, in this case a high frequency is kept in order to avoid problems with the motor used in the examples (whose time constant is not known); if the datasheet of the motor is available, refer to the value of the *mechanical time constant* (or other if specified) to determine the value of the period of the *PWM* signal.

In this case, with the parameters used, a signal is obtained that has a period of 0.1ms, more than sufficient for correct motor control.
<p align="center">
  <img src="img/pwm_01ms.png" width="30%">
</p>

</br>

## Servo Motors

A *servo motor* belongs to the family of "rotary actuators" and can be efficiently controlled in order to move with a precise speed or to achieve desired angular configuration. These are devices widely used in the world of robotics (certainly in robotic arms) and in general in contexts involving industrial automation, where precise position controls are important.

Internally it is composed of a series of electro-mechanical devices that determine a closed loop system, this allows the servomotor to be, to some extent, already internally controlled. In addition, a gear wheel system allows it to provide more torque than a simple brush motor.

<p align="center">
  <img src="img/servomotor.jpg" width="60%">
</p>

From an external point of view it is managed through a *PWM* signal, of which, this time, the period is also important; as long as this signal is supplied to the motor, it will maintain a precise angular position.

Generally, for many servomotors a *PWM* signal with a period of 20ms (50Hz) is used and the variation of the *duty cycle* determines the variation of the motor rod angle, from 0 to 180 degrees (usually) . The specific *duty cycle* values vary from engine to engine, generally we have:
* *duty cycle 1ms* : 0 degrees
* *duty cycle 2ms* : 180 degrees

Any other duty cycle value between these two will result in an angle between 0 and 180 degrees linearly.

<p align="center">
  <img src="img/servo_system.png">
</p>

### Servo Motors Control
Below is reported an example of control of a servomotor of the [Tower Pro, model SG90](http://www.towerpro.com.tw/product/sg90-7): a small servomotor that can rotate the axis about 180 degrees, but not able to develop a significant torque (1.80 Kg/cm). Information about the command signal and connection can be found on the [datasheet](docs/sg90-datasheet.pdf).

<p align="center">
  <img src="img/servomotor_scheme.png" width="70%">
</p>

To write a program that manages the motion of the servomotor it is important to keep in mind the relationship for which:

`DutyCycle = (MAX – MIN) * DESIRED_DEGREES / 180 + MIN`
in which `MAX` is 2ms and `MIN` is 1ms;

Based on this, it will be possible to map a desired value in degrees to a value in *ms* to assign as the *duty cycle* of the *PWM* signal, which in any case must have a period of 50Hz.

The configuration of the *timer* for the generation of the *PWM* signal is done taking into consideration the formula

<p align="center">
    <img src="https://render.githubusercontent.com/render/math?math=%5Ctext%7BUpdateEvent%7D%20%3D%20%5Cfrac%7B%5Ctext%7BTimer%7D_%7Bclk%7D%7D%7B(Prescalar%2B1)*(Period%2B1)%7D" width="37%">
</p>

and, after having set the system *clock* at 84MHz, choose:
* `Prescalar` : 8400
* `Counter` : 200

in order to obtain a 50Hz (20ms) period *PWM* signal

<p align="center">
  <img src="img/servo_control_MXconfig.png">
</p>

With this configuration, the *duty cycle* value (which will be configured in the `Pulse` variable of the` sConfigOC` structure or in the `CCR2` register) will have to vary approximately between 10 and 20 to actually be between 1ms and 2ms. However, it will be necessary to carry out empirical tests to determine the minimum and maximum angle of the motor, which often do not fully correspond to what is reported on the datasheets.

A first test can be performed with a very simple `main`, which changes the angle of the servo motor every second:

```c
...
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

/* Infinite loop */
while (1){
  for(int i = 10; i < 20; i+=1){
    HAL_Delay(1000);
    htim2.Instance->CCR2 = i;
  }
}
```
Note that to change the *duty cycle* value it is possible to act directly on the value contained in the `CCR2` register.

For the development of a more interesting software, a small library is implemented which provides the basic functions to manage the servomotor, mapping the desired angle into suitable *duty cycle* values.

#### :blue_book: servo.h // servo.c

The `servo.h` header file exports the main elements useful for managing the servo motor:
```c
/* Constants */
// MIN pulse width = 1*Counter/20 if 1ms if MIN for the motor
#define SERVO_MIN_MS	10
// MAX pulse width = 2*Counter/20 if 2ms if MAX for the motor
#define SERVO_MAX_MS	20

/* Structure for SERVO Motor */
typedef struct {
    TIM_HandleTypeDef* PWM_timer_handler;
} SERVO_t;

/* Functions */
void SERVO_init(SERVO_t* servo_struct, TIM_HandleTypeDef* PWM_timer);

void SERVO_set_degree(SERVO_t* servo_struct, float degree);

void SERVO_set_ms(SERVO_t* servo_struct, float pulse_width);
```
which are implemented in the `servo.c` file.

We define a structure that keeps, for now, only the pointer to the *timer* instance used for the *PWM* signal.

For the method `SERVO_set_degree`, the imlementation follows the formula already reported above (the one setting the values in terms of milliseconds). It is also added a simple system with saturation so that the servo never exceeds the `SERVO_MIN_MS` and `SERVO_MAX_MS`. These two constants are manually specified in the header file have been calculated exploiting the empirical tests performed on the engine and the` Counter` value of the *timer* that generates the *PWM*.
```c
/*
 * Set degree for SERVO
 * 		pulse_width = (MAX – MIN) * degree/180 + MIN
 */
void SERVO_set_degree(SERVO_t* servo_struct, float degree){
	float pulse_width = (SERVO_MAX_MS - SERVO_MIN_MS)*degree/180 + SERVO_MIN_MS;
	SERVO_set_ms(servo_struct, pulse_width);
}

/*
 * Set millisecond for SERVO
 * 		pulse_wdith must be between MIN and MAX
 */
void SERVO_set_ms(SERVO_t* servo_struct, float pulse_width){
	if(pulse_width < SERVO_MIN_MS){
		pulse_width = SERVO_MIN_MS;
	}else if(pulse_width > SERVO_MAX_MS){
		pulse_width = SERVO_MAX_MS;
	}
	TIM_HandleTypeDef* htim = servo_struct->PWM_timer_handler;
	htim->Instance->CCR2 = pulse_width;
}
```
With this simple basic library, it is possible to write software that works directly by imposing the desired angles to the servomotor, which is generally what is required by a motor of this type.
