---
layout: default
title:  "Lab 1: The Artemis Board"
date:   2023-01-26 17:55:39 -0500
categories: robotics lab
video_blink: 11_769IODiWqZN7U74EjghTWnUuh7zY8u
video_serial: 11e3-5Za_DhC-U5CAu6Up4OsqBPxKUzj9
video_temperature: 11l_eeKYUDDXU0so1iFlNH9vaW5P26nLX
video_microphone: 11fo6EB3Ca3jYI2HFA9W5BQnIfxkLgv-T
video_extra: 11Nam0bVGNSJD-VXX7yY8hlN1awn58kgJ
---
# Lab Objective

The purpose of this lab was to set up the Artemis board and Arduino IDE which we
will be using for the rest of the semester, and to learn how to use the on-board
LED, the serial message channel, and various on-board sensors.

# Part 1: Arduino Installation

I was able to install the Arduino IDE and connect it to the Artemis Nano without
any issues, except that the [setup instructions][setup_instr] linked in the 
assignment were slightly outdated, so the example programs I needed were named
differently in my IDE.

[setup_instr]: https://learn.sparkfun.com/tutorials/artemis-development-with-arduino

# Part 2: Blink

I uploaded the Blink example (Examples > 01.Basics > Blink) to the board. The 
on-board blue LED labeled "19" began blinking on and off every 1 second:

{% include googleDrivePlayer.html id=page.video_blink %}

# Part 3: Serial

I uploaded the Serial example (Examples > Apollo3 > Example04_Serial) to the
board and opened the serial monitor on my computer. I had to change the baud 
rate in the serial monitor to 115200 baud for the monitor to be readable, after
which I could type messages and receive the same message in response from the 
board:

{% include googleDrivePlayer.html id=page.video_serial %}

# Part 4: Temperature Sensor

I uploaded the AnalogRead example (Examples > Apollo3 > Example02_AnalogRead) to
the board, and opened the serial monitor, which began printing sensor readings,
shown in the video below.

The `external` reading is the analog voltage on the `EXTERNAL_ADC_PIN`;
the `temp` reading is from the temperature sensor on the board;
`vcc/3` is the voltage measured on a 1/3 voltage divider;
`time` is the time in milliseconds since the board was last reset.

Shown in the video, holding my thumb over the chip causes the temperature 
reading to gradually increase from 33600 to 34000, and releasing my thumb causes
it to gradually decrease again.

{% include googleDrivePlayer.html id=page.video_temperature %}

# Part 5: Microphone

I uploaded the Microphone example (Examples > PDM > Example1_MicrophoneOutput) 
to the board and opened the serial monitor. The board began printing out the 
loudest frequency it picked up on its microphone, displayed in Hertz. 

I tried changing the reading by whistling onto the chip. In this video I 
whistled at around 1809 Hz, 1041 Hz, 2059 Hz, and 930 Hz. One of the 
difficulties in doing this was that people nearby were trying to do the same 
thing with their boards, so my chip was sometimes picking up other people's
whistles.

{% include googleDrivePlayer.html id=page.video_microphone %}

# Additional Task

I programmed the board to detect a musical “A” note by modifying the example
microphone code used above. I added the following lines of code to the end of
the example's `printLoudest(void)` method:

{% highlight c %}
uint32_t targetFrequency = 440;
uint32_t freqBandwidth = 10;
if (ui32LoudestFrequency <= targetFrequency + freqBandwidth && ui32LoudestFrequency >= targetFrequency - freqBandwidth) {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
}
else {
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
}
{% endhighlight %}

This made the board's built in LED light up whenever the loudest frequency it 
detected was within 10 Hz of 440 Hz, which is a standard tuning for a musical 
“A” note. The tone generator used in the video is linked [here][tone].

[tone]:https://www.szynalski.com/tone-generator/

{% include googleDrivePlayer.html id=page.video_extra %}

The reason I made the bandwidth 10 Hz is that the website I was using was slightly
off in the frequency of the tones it produced, or the microphone was slightly
off in the frequency it measured. When I played a 440 Hz note, the serial 
monitor would print it out as 434 Hz instead, and would produce a false 
negative unless the bandwidth was at least 7 Hz.