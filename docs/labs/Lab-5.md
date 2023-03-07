---
layout: default
title:  "Lab 5: Motors and Open Loop Control"
date:   2023-02-07 17:55:39 -0500
categories: robotics lab
video_1: 1CSaYN53IaiteCX568Oh8CzHBT-GoRRX7
video_2: 12d4m6fgjbb8HI3X5n_epn8glnePEv1Yc
video_3: 12lszITCfYq4XE-bPJ1jp3BMb_Dyjl93w
---
# Lab Objective

In this lab we will connect our Artemis to the toy car and control it with a 
pre-programmed sequence of moves. 

# Prelab

The plan for attaching the Artemis to the motor drivers:

<img src="/img/Lab5/lab5_plan.png">

I chose pins 6, 7, 11, and 12 because they can generate PWM signals, and they're
split evenly between the left and right sides of the Artemis.

I'll be powering the Artemis and motors from separate batteries to give the 
motors more power, and to reduce power supply noise to the Artemis.

# Testing the Drivers

I soldered both drivers to the Artemis, braiding the wires to reduce 
electromagnetic interference. For testing, I hooked up one driver to an 
oscilloscope and power supply, as shown:

<img src="/img/Lab5/wiring_edit_2.jpg">

I set the power supply at 3.7 volts and 5 amps, since this mimics what the 
battery can supply.

To test PWM power regulation I ran this code:
```c
void setup(){
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  analogWrite(6,0);
  analogWrite(7,100); // lower power
  // analogWrite(7,200); // higher power
}
```

Oscilloscope with `analogWrite(7,100)`:

<img src="/img/Lab5/osc_100_crop.jpg">

Oscilloscope with `analogWrite(7,200)`:

<img src="/img/Lab5/osc_200_crop.jpg">

As expected, writing a higher value to pin 7 created an output wave with a 
higher duty cycle. This confirmed that PWM control was working properly.

# Connecting the Motors

I disassembled the car, connected the driver output to one of the motors, and
ran the code above with a PWM value of 100. I did this for both motors.

{% include googleDrivePlayer.html id=page.video_1 %}

After confirming both motors worked, I moved all components into the chassis and
connected the motors and Artemis to their respective batteries.

<img src="/img/Lab5/chassis_labeled.jpg">

# Remote Control

### Code
To remotely control the robot and make testing easier, I implemented more 
Bluetooth commands to wrap the ```analogWrite``` commands.
Python sends arguments in a Bluetooth command string:
```python
def moveStraight(value, duration):
  right = value
  left = round(value*0.866)
  ble.send_command(CMD.POWER_WHEEL, f"{right}|{left}|{duration}")
moveStraight(255, 1000)
```

Arduino extracts the arguments from the string and calls other functions...
```c
powerAllWheels(valRight, valLeft);
delay(msTime);
stopAllWheels();
```
... which I implemented in a separate header file:
```c
void powerAllWheels(int rightVal, int leftVal) {
  powerWheel(RIGHT, rightVal, FAST);
  powerWheel(LEFT, leftVal, FAST);
}

void stopAllWheels() {
  powerWheel(RIGHT, 0, SLOW);
  powerWheel(LEFT, 0, SLOW);
}

void powerWheel(WheelSide side, int value, DecayMode decay) {
  int frwdPin = side == RIGHT ? 11 : 7;
  int bkwdPin = side == RIGHT ? 12 : 6;
  int frwdVal = decay == FAST ? max(0, value) : 255-max(-value,0);
  int bkwdVal = decay == FAST ? max(-value,0) : 255-max(0, value);
  analogWrite(frwdPin, frwdVal);
  analogWrite(bkwdPin, bkwdVal);
}
```

```powerWheel``` is modeled after the specification tables in the 
[driver datasheet][drv_docs].

[drv_docs]:https://www.ti.com/lit/ds/symlink/drv8833.pdf#%5B%7B%22num%22%3A143%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C423.5%2C0%5D

### Lower Limit
I found that a lower limit on the PWM value I can send to the wheels is around
50, and anything below this doesn't consistently move the robot forward and
on-axis.

### Calibration
To move the robot straight I had to add a calibration factor, scaling the
power to the left motor down by 13% (multiply by 0.866). I found this value by 
finding pairs of PWM values for the left and right motors which moved the robot 
straight, and doing a linear regression.

<img src="/img/Lab5/regression.png">

Here is the robot successfully traveling in a straight line for 6 feet:

{% include googleDrivePlayer.html id=page.video_2 %}

### Open Loop Untethered Control

I implemented turns by changing the Bluetooth command slightly:
```python
def rotateInPlace(value, duration):
  right = value
  left = -1*round(value*0.866) # multiply by -1
  ble.send_command(CMD.POWER_WHEEL, f"{right}|{left}|{duration}")
```
To test open-loop untethered control I ran this sequence of commands:
```python
moveStraight(100, 1000)
rotateInPlace(100, 500)
moveStraight(100, 1000)
rotateInPlace(-100, 500)
moveStraight(100, 1000)
```
This is demonstrated in the video below:

{% include googleDrivePlayer.html id=page.video_3 %}

# Additional Tasks

### analogWrite frequency

TODO

### Start-up speed vs. slowest speed

TODO
