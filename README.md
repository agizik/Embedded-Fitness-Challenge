# Embedded-Fitness-Challenge using a STM32 micro-controller for workout

The device detects and counts 4 different exercises which are; ○ Situps ○ Pushups ○ Jumping Jacks ○ Squats  In the beginning, all the LEDs are off, when an exercise is in progress an LED blinks when it is completed the LED is on.  Orange LED shows the progress for Jumping Jacks. Green LED shows the progress for Squats. Red LED shows the progress for Situps. Blue LED shows the progress for Pushups.


Real-time embedded systems

• Used C, Assembly and PlatformIO to program a STM32 board and its accelerometer
• The device indicates what specific exercise has occurred with the 4 LEDs and keeps count for all four exercises
• I used the accelerometer on the STM32F407G-DISC1 board to get the readings along the X, Y, Z axis and the angles around plane X (roll) and around plane Y (pitch).

Below, you can find the Youtube videos where I demonstrate how the board works:

sequence -->1.squats 2.jumping jacks 3.situps 4.pushups
https://youtu.be/Vp1cIwzdjmM 

sequence -->1.situps 2.pushups 3.squats 4.jumping jacks
https://youtu.be/3HaBhXi6ol0
