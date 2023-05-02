---
layout: default
title:  "Lab 7: Kalman Filter"
date:   2023-03-13 14:00:00 -0500
categories: robotics lab
---
# Lab Objective

In this lab, I will be implementing a Kalman Filter to effectively speed up the
slow time-of-flight sensor, allowing the robot to do the [Lab 6][Lab6] task 
(speeding towards a wall and stopping 1 foot away) faster.

[Lab6]:Lab-6

# Step response

To estimate the drag and mass of the robot, I powered the wheels with a step 
function. The max PWM value was set to 250, which is around the value I got out
of my PID controller. The robot started 4m away from a wall.

The measured distance to the wall and PWM value is shown below:

<p align="center">
<img src="/img/Lab7/step_distance.png">
</p>

The velocity (time derivative of distance to wall) is plotted below:

<p align="center">
<img src="/img/Lab7/step_speed.png">
</p>

The maximum speed reached by the robot was about 3 m/s. It didn't reach a 
steady-state speed, since I couldn't start the robot further away from the wall
than 4 meters or else my sensor readings wouldn't be accurate. I can estimate 
the steady-state speed of the robot is around 3 m/s.

Based on this, the drag value I used was 
$$ d = u/x' = 1/(3000 mm/s) = 0.000333 $$. 

The time taken to reach 90% of the steady state speed, starting from 0 velocity,
was around 0.7 seconds:

<p align="center">
<img src="/img/Lab7/speed_annotated.png">
</p>

Based on this, I estimated the inertia as 
$$ m = -d*t_{0.9}/ln(1-0.9) = 0.000101 $$.

# Kalman Filter

After finding these values I plugged them into this code written by Anya last 
year to get 