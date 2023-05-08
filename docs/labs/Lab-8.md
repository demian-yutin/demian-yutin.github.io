---
layout: default
title:  "Lab 8: Stunts"
date:   2023-03-13 14:00:00 -0500
categories: robotics lab
video_1: 139SBs2BbaSA2osQQPeRxIXrRUOo0nmWf
video_2: 13Hg9bq1kL2crJPWhAHWYTC_RFlyY8KsN
video_3: 13HxLfqFl_j7Y8BneD4NwvzX6eO8L_ZEj
---
# Lab Objective

In this lab, I use the robot's improved sensor and control capabilities to 
perform a stunt, where the robot drives towards a wall, flips over vertically
when 0.5m away, and reverses. (Task A)

# Procedure

To perform the stunt I added a new robot command ```STUNT``` with arguments for
speed, flip distance, and other parameters that I could change on the fly in
Jupyter.

I used the same system of sending debug data as before, so I could see when the
robot was deciding to flip, and what the distance readings were.

For distance readings I used the extrapolated distance estimate from 
[Lab 7][Lab7].

[Lab7]:Lab-7

On the Arduino side, the code was basically this:

```c
// reset / initialize
tof.resetSensor();
bool hasFlipped = false;
long flipTimestamp = -1;
int power = maxPower;
// loop until end
while(timeElapsed < totalTime) {
    // time update
    currentTime = millis();
    timeElapsed = currentTime - startTime;
    // distance update
    int distance = tof.getDistanceEstimate();
    // check if we need to flip
    if (!hasFlipped && distance <= flipDistance) {
        power = -maxPower; // full reverse power
        hasFlipped = true;
        flipTimestamp = currentTime;
    } else if (currentTime - flipTimestamp >= 1000){
        power = 0; // stop after reversing for 1 second, so the robot doesn't run off
        timeElapsed = totalTime + 1; // break out of loop after gathering data
    }
    powerWheels(power);
    // data collection
    intArr1[numData] = (int)timeElapsed;
    intArr2[numData] = distance;
    intArr3[numData++] = power;
}
stopAllWheels();
// send data
sendArrays(numData);
```

# Unsuccessful runs

In this run, the robot successfully flipped over before hitting the wall.
However, it turned 90 degrees about the vertical axis while flipping, so
when it went into "reverse", it actually went off to the side instead. 
Immediately after flipping, the back wheels bounced and caused to robot
to turn onto its side, losing control completely.

{% include googleDrivePlayer.html id=page.video_1 %}

In this video, the bounce after flipping isn't as extreme, so the robot doesn't
turn onto its side. However it turns more than 90 degrees about the vertical 
axis while flipping, so when it goes in "reverse" it immediately hits the wall 
instead.

{% include googleDrivePlayer.html id=page.video_2 %}

# Attempted solutions

The list of things I tried to make this work includes the following:

- Using a fully charged battery
- Getting a new battery
- Completely stopping the motors in between the full-forward and full-reverse
- Putting extra weight into the battery compartment to shift the robot's center of mass
- Starting the flip further away from the wall
- Using slow decay mode on motor drivers to increase torque
- Just powering the motors as normal without using a special decay mode
- Decreasing power to the left wheel to keep the robot pointed straight
- Using a different left-right motor calibration factor for forward and reverse movement
- Putting rubber bands on the wheels to try to improve the friction

The next thing I was going to try was to use some kind of PID orientation 
control to make sure the robot stayed straight while flipping, but I was advised
by a TA to just submit this and finish the other labs, since this lab is already
very late.

# Blooper

{% include googleDrivePlayer.html id=page.video_3 %}