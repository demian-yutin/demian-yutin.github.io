---
layout: default
title:  "Lab 7: Kalman Filter"
date:   2023-03-13 14:00:00 -0500
categories: robotics lab
video_1: 138w3-cH30Y5YoKYQkn_MraUbFvlysF8V
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

After finding these values I plugged them into [this code written by Anya last 
year][Anya] to get a Kalman filter:

[Anya]:https://anyafp.github.io/ece4960/labs/lab7/

```python
# Initial state uncertainty 
sig = np.array([[5**2,0],[0,5**2]])

d = 0.000333 # drag
m = 0.000101 # mass

# A, B, C matrices
A = np.array([[0,1],[0,-d/m]])
B = np.array([[0],[1/m]])
C = np.array([[-1,0]])

# Process and sensor noise
sig_u = np.array([[10**2,0],[0,10**2]])
sig_z = np.array([[20**2]])

# Discretize A and B
delta_t = times[1] - times[0]
Ad = np.eye(2) + delta_t * A
Bd = delta_t * B

# Initial state
x = np.array([[-distancesMM[0]],[0]])

# KF estimation
def kf(x,u,sig,y):
    
    x_p = Ad.dot(x) + Bd.dot(u)                      # predicted state
    sig_p = Ad.dot(sig.dot(Ad.transpose())) + sig_u  # predicted state uncertainty
    
    y_m = y-C.dot(x_p)
    sig_m = C.dot(sig_p.dot(C.transpose())) + sig_z
    
    kf_gain = sig_p.dot(C.transpose().dot(np.linalg.inv(sig_m)))  # KF gain

    x_n   = x_p + kf_gain.dot(y_m)                   # new state 
    sig_n = (np.eye(2) - kf_gain.dot(C)).dot(sig_p)  # new state uncertainty

    return x_n, sig_n
```

I ran this code to test what the Kalman filter would output, and compared this
to the raw measurements from my TOF sensor:

```python
# Run code
kf_state = []
for u, d in zip(values, distancesMM):
    x, sig = kf(x, [[u/80]], sig, [[d]])
    kf_state.append(x[:,0])
# then plot kf_state...
```

<p align="center">
<img src="/img/Lab7/kf.png">
</p>

# Extrapolation

To increase the effective sampling speed of the TOF sensor without implementing
a Kalman filter on the Arduino, I decided to use extrapolation to predict 
distance sensor readings based on the previous 2 actual readings.

To do this, I changed the code in my ```tof``` class:

```c
int getDistanceEstimate() {
// update distance history if new data is available
if (sensor1.checkForDataReady()) {
    int newDist = distance1 * (1 - alpha) + alpha * sensor1.getDistance();
    sensor1.clearInterrupt();
    distance2 = distance1;
    distance1 = newDist;
    time2 = time1;
    time1 = millis();
    nPoints++;
}
if (nPoints >= 2) {
    // extrapolate from previous 2 points for estimate
    long time = millis();
    float slope = (float)(distance1 - distance2)/(time1 - time2);
    long timeDelta = time - time1;
    estimatedDistance = (int)(0.5 + distance1 + slope * timeDelta);
    return estimatedDistance;
} else {
    return distance1;
}
}
```

After adjusting the debug code to collect the raw distance data as well as the 
interpolated distance data, we can see that the interpolation successfully
smooths out the slow sampling rate of the TOF:

(Raw distance in red, extrapolated distance in orange)

<p align="center">
<img src="/img/Lab7/interp.png">
</p>

The PID controller works with these extrapolated distance values just as well:

{% include googleDrivePlayer.html id=page.video_1 %}