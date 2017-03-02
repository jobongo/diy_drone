# DIY_Drone
<h2>Introduction</h2>
"From Scratch" Drone Using Teensy 3.2 and Arduino.
I wanted to build a drone frome scratch to see if it was possible (or better yet if I could do it). I not only 
wanted to build the code but also the frame, circuit board and pretty much anything that could go into it. 
This was a way to learn C++ and Autocad but also to get into circuit board design and some of the electrical engineering aspects.
<h2>The Hardware</h2>
I used the Teensy 3.2 Microcontroller for this build. 3.1 should be fine as well. The only difference is the current rating on the 3.3v output.

<h4>Flight Controller</h4>
The flight controller, at the moment, consists of the following:<br>
Teensy 3.2<br>
NRF24L01+ Radio (PA LNA Version)<br>
MPU6050 Motion Controller<br>
<h4>Controller</h4>
The controller at the moment consists of the following:<br>
Teensy 3.2<br>
2 PS2 Thumb Joysticks<br>
NRF24L01+ Radio (PA LNA Version)<br>
<h4>Microcontrollers</h4>
It should be possible to use different microcontrollers at this stage. Even the libraries should be the same, only specifying the board when compiling. 
<h2>The Software</h2>
Some of the code is taken from  from from different sources. I used the I2C code from Jeff Rowberg at one point but decided that the Arduino wire library works. Still doing testing though.
Also, the code for extracting the MPU data was taken from
<a href="https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_DMP6"> MPU 6050 DMP6</a> (Jeff Rowberg i2cdevlib Git Repo)
<h4>Flight Controller</h4>
The flight controller, at the moment, consists of the following:<br>
Teensy 3.2<br>
NRF24L01+ Radio (PA LNA Version)<br>
MPU6050 Motion Controller<br>

<h4>Controller</h4>
The controller at the moment consists of the following:<br>
Teensy 3.2<br>
2 PS2 Thumb Joysticks<br>
NRF24L01+ Radio (PA LNA Version)<br>
<h2>The Frame</h2>
<h4>Design</h4>
<h2>The Circuitry</h2>
<h4>Cirucit Design</h4>
<h2>Current Issues</h2>
Pretty Much Everything :)
