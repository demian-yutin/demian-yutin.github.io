---
layout: default
title:  "Lab 6: Closed-loop control (PID)"
date:   2023-03-13 14:00:00 -0500
categories: robotics lab
video_1: 134nRQVghJBRp3DjOLhagf0yGUI2CUfhW
video_2: 136GJ0iypYg3nCPplixtlqJcOyJfFo_tI
video_3: 137eoygt2MOP-OTDiFxfYzkEeuNhgxfaR
video_4: 135e19lEJB8-Y4yMzsobbpKGZ_I6SmUbL
---
# Lab Objective

In this lab I am implementing a position controller using PID control. The
robot will drive towards a wall and stop once it's 1 foot away, using PID to 
make this motion as fast as possible.

# Prelab

To send/receive Bluetooth data, I added new commands to the system used in 
previous labs. Debug data is collected in arrays and sent back in large chunks
in string format. On the Python side, sending commands and processing data is 
mostly the same as in [Lab 4][Lab4]. 

[Lab4]:Lab-4

The new commands added were `CONTROL_DISTANCE`, which executes the main task, 
`SET_PID_PARAMS`, which modifies the PID controller parameters, and 
`WHEEL_SETTINGS`, which modifies wheel calibration values used in [Lab 5][Lab5].

[Lab5]:Lab-5

Because PID control and sensor usage are complicated, I created custom classes 
to encapsulate these systems. ```pid``` handles PID control, and ```tof``` 
handles the time-of-flight sensor.

`SET_PID_PARAMS` simply sets the values stored inside ```pid``` to the ones
supplied over Bluetooth:

```c
// parse values from command
float Kp, Ki, Kd, alpha, setPoint;
if (!(robot_cmd.get_next_value(Kp)
    && robot_cmd.get_next_value(Ki)
    && robot_cmd.get_next_value(Kd)
    && robot_cmd.get_next_value(alpha)
    && robot_cmd.get_next_value(setPoint)))
    return;
// set internal values
pid.setValues(Kp, Ki, Kd, alpha, setPoint);
```

These values are used inside of the ```pid.step()``` function, which updates the
```pid``` object's stored integrator and derivative values, and calculates a new
control variable value, which ```pid.PID()``` returns:

```c
void step(float currentMeasurement, float dt) {
    // update history
    nPoints += 1;
    yPrev2 = yPrev1;
    yPrev1 = currentMeasurement;
    // calculate error
    e = setPoint - currentMeasurement;
    // add error to integrator
    I += e * dt;
    // calculate derivative
    if (nPoints < 2) {
        dF = 0.0; // too few points to estimate derivative
    } else {
        float d = -(yPrev1 - yPrev2) / dt;
        dF = alpha * d + (1.0 - alpha) * dF; // low-pass filter
    }
    controlVariable = Kp * e + Ki * I + Kd * dF; // PID formula
}

float PID() {
    return controlVariable;
}
```

The ```tof``` object handles TOF sensor initialization and usage. The 
```tof.getDistance()``` method checks if a new sensor reading is available, and 
if not, returns the latest reading immediately:

```c
int getDistance() {
    if (sensor.checkForDataReady()) {
        distance = sensor.getDistance();
        sensor.clearInterrupt();
    } // if not ready, don't wait, just return the latest reading
    return distance;
}
```

Both ```tof``` and ```pid``` are used inside `CONTROL_DISTANCE`, which powers 
the wheels using the PID controller for a given time interval, and is
implemented as follows:

```c
// parse command arguments
int timeLimit;
if (!(robot_cmd.get_next_value(timeLimit))) return;
// initialize
pid.resetFields(); // resets integrator and derivative
int numData = 0;
// gather data until time limit is exceeded
long currentTime = millis();
long endTime = currentTime + timeLimit;
while (currentTime < endTime) {
    // get latest distance reading
    int distance = tof.getDistance();
    // update time variables
    long previousTime = currentTime;
    currentTime = millis();
    long dt = currentTime - previousTime;
    // update PID controller with new distance and time, and retrieve value
    pid.step(distance, dt * 0.001);
    float value = pid.PID();
    // store debug data
    times[numData] = (int)(currentTime - startTime);
    distances[numData] = distance;
    values[numData++] = value;
    // power wheels
    powerWheelsAdjusted(value);
}
stopAllWheels(); // stop the robot once we've stopped using PID control
// send all the data we've stored
estring.clear();
for (int i = 0; i < numData; i++) {
    // write stuff to estring...
    estring.append( /* ... */ );
    // send a string
    if (i == numData - 1 || estring.get_length() >= 120) { 
        characteristic_string.writeValue(estring.c_str());
        estring.clear();
    }
}
```

# Starting Values

The motor input values range from -255 to 255, and the TOF sensor can give 
readings up to 4000 mm (4 meters). So, a reasonable starting value for $$ K_p $$
is $$ 255 / 4000 \approx 0.06 $$.

Initially I chose $$ K_p = 0.1 $$, $$ K_i = 0 $$, and $$ K_d = 0 $$. This 
made the robot move closer to the wall, but led to oscillation:

{% include googleDrivePlayer.html id=page.video_1 %}

Plotting the distance and PID value over time, we can see the oscillation
clearly. We also see that the sensor only reads a new value 11 times per second,
as shown by the many flat regions in the graph.

<p align="center">
<img src="/img/Lab6/oscillation.png">
</p>

To eliminate the oscillation I tried decreasing $$ K_p $$ and increasing 
$$ K_i $$ for better speed. However, the integrator value was causing the robot
to keep moving after reaching the set-point, so I decided to implement wind-up
protection before moving further.

# Wind-up Protection (5000-level task)

I implemented wind-up protection in two parts. Firstly, I clamp the integrator
by not incrementing ```I``` if the control variable exceeds the maximum motor
input value. Second, if the distance measurement error is close to 0, or 
changed sign since the last measurement, ```I``` is reset:

```c
// ... (inside pid.step)
// add error to integrator, but only if not saturated
if (controlVariable < maxOutput && e > 0
    || controlVariable > -maxOutput && e < 0) {
    I += e * dt;
}
// zero-out the integrator if error is 0 or crosses 0
if (abs(e) <= almostZero || SIGN(e) != SIGN(setPoint - yPrev2)) {
    I = 0.0;
}
// ...
```

This was necessary to prevent the integrator from causing excess overshoot, and
allows us to set $$ K_i $$ to higher values without worry.

# TOF Settings

To ensure accurate readings, I used ```setDistanceModeLong()``` to set the TOF
sensor's range up to 4 meters. To fix the sensor's slow readings, I used
```setTimingBudgetInMs()``` and ```setIntermeasurementPeriod()``` to decrease 
the time between samples from 100 ms to 33 ms, the minimum which can be used in 
long-distance mode.

According to the [documentation][tof_doc], this timing budget decreases the max 
range from 4 meters to 3.1 meters, and increases error by a millimeter, but I 
decided this isn't a big deal, and it's worth it to get faster sampling.

[tof_doc]:https://cdn.sparkfun.com/assets/8/9/9/a/6/VL53L0X_DS.pdf#page=11

<!-- 
The ```setProxIntegrationTime()``` function was not available in my version of
the TOF library. -->

# PID Parameters

After lots of parameter tuning, I ended up with these values:

$$\begin{aligned}
K_p &= 0.18 \\
K_i &= 0.12 \\
K_d &= 0.15 
\end{aligned}$$

To make the derivative estimate work properly, I got rid of the 20ms delay in 
the PID control loop and made the loop wait for readings from the TOF sensor.
This ensured that the derivative wouldn't jump between 0 and a non-zero value.

I also set alpha to 0.08 and maxSpeed to 250, and decreased power to the left 
wheel by 10% relative to the right wheel.

# Demonstration

Here is the robot driving towards a wall:

{% include googleDrivePlayer.html id=page.video_2 %}

A graph of the TOF distance and motor input during this run:

<p align="center">
<img src="/img/Lab6/finalgraph.png">
</p>

By taking the time derivative of the distance reading, I can estimate that the
robot had a top speed of 2 meters/second:

<p align="center">
<img src="/img/Lab6/speed.png">
</p>

Driving towards a bin:

{% include googleDrivePlayer.html id=page.video_3 %}

Driving on carpet:

{% include googleDrivePlayer.html id=page.video_4 %}

The robot sometimes slides and has a tendency to rotate while braking, but still
manages to stop roughly 1 foot away from the wall in every situation.

For this assignment I referenced [Krithik's solution from last year][Krithik]
for how he encapsulated sensors into classes, and got help from [Tobi][Tobi] on
implementation details.

[Krithik]:https://kr397.github.io/ece4960-labs/lab6.html
[Tobi]:https://abioticfactor.github.io/fast-robots/