---
layout: default
title:  "Lab 2: Bluetooth"
date:   2023-02-07 17:55:39 -0500
categories: robotics lab
video_blink: 11_769IODiWqZN7U74EjghTWnUuh7zY8u
video_serial: 11e3-5Za_DhC-U5CAu6Up4OsqBPxKUzj9
video_temperature: 11l_eeKYUDDXU0so1iFlNH9vaW5P26nLX
video_microphone: 11fo6EB3Ca3jYI2HFA9W5BQnIfxkLgv-T
video_extra: 11Nam0bVGNSJD-VXX7yY8hlN1awn58kgJ
---
# Lab Objective

In this lab, we used Bluetooth to communicate wirelessly with the Artemis board 
via Python commands. This is the framework we will be using for all 
communication in future labs.

# Pre-lab Setup

Python and pip were already installed on my lab computer. I installed venv using
the command `python3 -m pip install --user virtualenv`, created a folder for the
project, and ran `python3 -m venv FastRobots_ble` to make a virtual environment.

After making sure everything needed was installed and starting the virtual 
environment with `.\FastRobots_ble\Scripts\activate`, I started the Jupyter
server with `jupyter lab`.

On the Arduino side, I installed ArduinoBLE with the library manager, and burned
the given ble_arduino.ino sketch onto my board. The board printed its MAC 
address in the serial monitor:

<img src="/img/Lab2/ss1.png">

# Codebase

The ble_arduino folder contains the C header files and .ino files which are 
uploaded to the board. These tell the board to broadcast itself over Bluetooth,
receive messages, and send responses.

The ble_python folder contains the Jupyter notebook and Python files 
which are run from a computer, and a .yaml config file. These tell the computer 
to connect to the board over Bluetooth, send messages, and receive responses.

To set up the connection, some parts of the 
code have to have parity. On the Arduino side, ble_arduino.ino specifies four 
BLE UUIDs which define the board and different data types, and on the Python 
side, connections.yaml has the exact same UUIDs:

<img src="/img/Lab2/ss2.png">
<img src="/img/Lab2/ss3.png">

Similarly, ble_arduino.ino has a list of robot commands, and cmd_types.py has 
the same list in the same order:

<img src="/img/Lab2/ss4.png">
<img src="/img/Lab2/ss5.png">

This parity is critical to ensure correct communication, so that both the 
computer and Artemis are "speaking the same language".

# Configurations

As shown in the above images, I modified connections.yaml and ble_arduino.ino to
include the Artemis's MAC address (c0:83:31:6a:b8:3c) and to use a randomly 
generated BLEService UUID (4e429e32-1ccd-4ef0-972b-8b3fd4cddbdd).

Since I was running on Windows, I changed line 53 in base_ble.py to `if True:`, 
but I later changed this to `if False:`. This change technically could make my 
computer connect to someone else's Artemis board, but it was necessary to make 
the Bluetooth connection stable, and was suggested by a TA.

# Demo

I was able to run all cells in demo.ipynb. These cells generated outputs:

<img src="/img/Lab2/demo1.png">
<img src="/img/Lab2/demo2.png">
<img src="/img/Lab2/demo3.png">
<img src="/img/Lab2/demo4.png">
<img src="/img/Lab2/demo5.png">

The Artemis printed appropriate outputs to the serial monitor:

<img src="/img/Lab2/demo5b.png">

# Task 1: ECHO 

Inside of the `handle_command()` function in ble_arduino.ino, in the `case ECHO:`
case in the switch statement, I added the following:

<img src="/img/Lab2/task1a.png">

In the Jupyter notebook, I added the following in new cells:

<img src="/img/Lab2/task1b.png">

I ran the code and got the above output, confirming that the Artemis received a
message over Bluetooth and was able to send it back with changes. To make sure 
the response was from my board, the Artemis printed the following to the serial 
monitor:

<img src="/img/Lab2/task1c.png">

# Task 2: GET_TIME_MILLIS 

I added `GET_TIME_MILLIS` as a command in ble_arduino.ino's `CommandType` enum,
and also in cmd_types.py. This is a screenshot of both files with all the 
commands I ended up adding:

<img src="/img/Lab2/comms1.png">
<img src="/img/Lab2/comms2.png">

To implement and test the command, I added the following in `handle_command()`
and the Jupyter notebook:

<img src="/img/Lab2/task2a.png">
<img src="/img/Lab2/task2b.png">

This correctly outputs the time in milliseconds since the board was last reset,
prepended with "T:".

# Task 3: Notification Handler

I added the following to the Jupyter notebook:

<img src="/img/Lab2/task3b.png">

Using a notification handler allows us to process a response as soon as we
receive it, instead of having to call `ble.receive_string()` and possibly 
getting an older response than we were expecting. The above code also prints the
time in seconds instead of milliseconds, although the board still sends the same
message as it did before.

# Task 4: GET_TEMP_5S 

I added the following:

<img src="/img/Lab2/task4a.png">
<img src="/img/Lab2/task4b.png">

This reads the board's temperature (in Celcius) once a second, for 5 seconds, 
and sends the result to the computer with timestamps. The Artemis
receives 1 message, and sends back 5 messages. I decided to do this instead of
combining all the readings into one message because the message size limit of
150 bytes means I would need to use this method for the next task anyways.

I had to restart the Jupyter notebook to run this cell properly, since the
previous task created a notification handler which remained in effect for this 
task. Doing this repeatedly got annoying, since I had to reconnect to the 
Artemis every time I changed the code, so I added the following cell, which I 
ran whenever I needed to change a callback function:

<img src="/img/Lab2/stop_notif.png">

I later find a better fix for this issue (see "Effective Data Rate".)

# Task 5: GET_TEMP_5S_RAPID 

I added the following:

<img src="/img/Lab2/task5a.png">
<img src="/img/Lab2/task5b.png">

I also declared `int command_start_time;` at the start of `handle_command()`. 
This code continuously takes temperature readings as fast as possible for 5
seconds. Running this produced 160 temperature readings:

<img src="/img/Lab2/task5c.png">

The temperature stayed mostly constant over the 5 seconds, but it is interesting
that the samples weren't spaced uniformly in time. For example, the first 4 
samples are all taken within 3 milliseconds, but the 4th and 5th samples are 
taken 61 milliseconds apart.

This might be because one of the functions being called in the loop occasionally
takes a while to execute, or it could be caused by lag in the Bluetooth 
connection.

# Limitations

To reduce Bluetooth communication overhead and lag, we want to package data into
larger chunks before sending it to the computer. The Artemis has 364 KB of 
memory we can use for local variables, as we see whenever we upload a sketch:

<img src="/img/Lab2/memory.png">

If each value we want to store is 16 bits (2 bytes), then we can store about 
182,000 values. If we sample data at 150 Hz, then we can store about 20 minutes
worth of data on the board. This means that as long as we send data to the 
computer at least once every 20 minutes, we won't run out of space on the board.

It seems like we have a lot of space, but we could run into issues if we were
collecting more data. For example, if were sampling data at a higher frequency, 
or the program had a higher memory overhead, or our values were more than 
2 bytes each, or we were storing data from multiple sources.

# Effective Data Rate and Overhead

To test the effective data rate of Bluetooth, I added the ECHO_TIMESTAMP
command, which sends back exactly what the Artemis received and does nothing
else:

<img src="/img/Lab2/atask1a.png">

This is separate from the regular ECHO command because I wanted to minimize 
overhead, so I could be measuring only the time spent on Bluetooth communication
and not string building operations.

In Jupyter, I wrote code which calls this command and sends the Artemis a 
message which is either 1, 5, 25, or 120 bytes long, and tracks how long it 
takes to respond:

<img src="/img/Lab2/atask1b.png">

I use `ble.stop_notify` wrapped in a try-except block so that I don't have to
worry about older versions of the notification handler persisting in the 
Jupyter kernel. 

To test this rigorously, I ran 30 trials for each message size:

<img src="/img/Lab2/atask1c.png">

I then copied this into Excel to produce a plot of response times for
each message size, and calculate median times:

<img src="/img/Lab2/atask1d.png">
<img src="/img/Lab2/atask1e.png">

The median response times for 1-byte messages, 5-byte messages, and 25-byte
messages were nearly identical at 59-60 ms. 120-byte messages took twice as long
to send and receive, with a median time of 119 ms.

The effective data rate in bytes/second was higher for larger message sizes. 
5-byte messages could be sent at 84 bytes/second, whereas 120-byte messages 
could be sent at 1004 bytes/second. This confirms that sending data in larger 
chunks is good for reducing Bluetooth overhead.

# Reliability

To test whether the Bluetooth connection could lose data, I added the 
RELIABILITY_TEST command, which sends 500 120-byte packets from the board to
the computer:

<img src="/img/Lab2/atask2a.png">

The computer sends the command and receives the messages, counting to make sure
that all 500 messages are received with exactly the right data:

<img src="/img/Lab2/atask2b.png">

The connection was perfectly reliable, and all the data was received.