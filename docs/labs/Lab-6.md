---
layout: default
title:  "Lab 6: Closed-loop control (PID)"
date:   2023-03-13 14:00:00 -0500
categories: robotics lab
---
# Lab Objective

In this lab I am implementing a position controller using PID control. The
robot will drive towards a wall as fast as possible and stop once it's 1
foot away from the wall.

# Prelab
  Clearly describe how you handle sending and receiving data over Bluetooth
  Consider adding code snippets as necessary to showcase how you implemented this on Arduino and Python

I'm sending/receiving Bluetooth data with a few new robot commands.

On the Arduino side, debug data is collected in arrays and sent periodically.

(code)

(list implemented commands)
  command to start the controller
    run PID for 5s while storing debug data
      (hard stop implemented on Arduino side)
      (data: sensors + timestamps; PID branch outputs; motor inputs)
      (ram limit 384KB)
    send debug data in response
  command to tweak gains

# TOF Settings (Sensor range and sampling time)
  Range/Sampling time discussion
  Be sure to include a discussion of sensor sampling rate and how this affects the timing of your control loop. 
  Consider the range and sampling time you choose for your TOF sensor; it may be worth lowering the accuracy for faster updates. Note that the medium range is only available if you are using the (ToF Pololu library).
  Also note that the sensor has a programmable integration time. If this is set too high, you will see large jumps in your data as the robot drives and you can no longer assume that the measurements are independent. You can lower the integration time (trading off accuracy for speed) using: proximitySensor.setProxIntegrationTime(4); //A value of 1 to 8 is valid. Again this function is only available in the Tof Pololu library.

The TOF's API allows us to decrease sampling time at the cost of range.

It also lets us 

# PID Parameters
  P/I/D discussion (Kp/Ki/Kd values chosen, why you chose a combination of controllers, etc.)
  Given the range of motor input values and the output from your tof sensor, discuss what a reasonable range of the proportional controller gain will be.

After lots of testing and tweaking, I arrived on these values for the PID
controller:

$$ K_p = 1 \\ K_i = 2 \\ K_d = 3 $$

# Demonstration
  To demonstrate reliability, please upload videos of at least three repeated (and hopefully successfull) experiments.
  Graph of TOF vs time
  Graph of Motor input vs time
  Any other graphs to help with debugging

# Speed
  Please clearly document the maximum linear or angular speed you are able to achieve (you can use your sensors to compute this). 

# Additional Task
## Integrator Wind-up Protection
implementation and discussion
