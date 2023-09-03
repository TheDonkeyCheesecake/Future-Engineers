# Engineers Documentation

Repository containing the engineering elements and details of Team XXXXXX from Canada, participating in the 2023 WRO Future Engineers competition.
***
## Repository Content

- `chassis` - details of our robot chassis
- `models` - 3D printable files designed by us
- `schemes` - schematics of our robot's electrical systems
- `src` - main and other programs to run/control software
- `t-photos` - one serious and one funny photo
- `v-photos` - photos of our robot from each angle
- `video` - YouTube link of our robot running each program on the game field
- `others` - other essential files
***
## Team Members

- Weston Law, 15, [westonlaw@hotmail.com](mailto:westonlaw@hotmail.com)
- Brian Yin, 16, [brianyin256@gmail.com](mailto:brianyin256@gmail.com)
- Eric Rao, 15, [ericrao08@gmail.com](mailto:ericrao08@gmail.com)


![Serious](https://github.com/TheDonkeyCheesecake/Future-Engineers/blob/main/t-photos/Official.jpg)
*Weston(left), Brian(middle), Eric(right)*

![Funny](https://github.com/TheDonkeyCheesecake/Future-Engineers/blob/main/t-photos/Funny.jpg)
***
## Hardware List and Assembly

### Parts List

- [LaTrax Rally RC Car Chassis](https://latrax.com/products/rally)
- [Arduino UNO R3 Controller Board + wires/cables](https://www.amazon.ca/gp/product/B01D8KOZF4/ref=ppx_yo_dt_b_search_asin_image?ie=UTF8&psc=1)
- [Vemico Raspberry Pi 4 Board + Fan](https://www.amazon.ca/gp/product/B09WXRCYL4/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
- [SanDisk 64GB Ultra microSD Card](https://www.amazon.ca/gp/product/B073JYVKNX/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&th=1)
- [PiSugar S Pro 5000 mAh Lithium Battery + PiSugar Module Platform Attachment](https://www.amazon.ca/Portable-Platform-Raspberry-Accessories-handhold/dp/B097RCFHD2)
- [GOOLSKY 2435 3300KV Brushless Motor and 25A ESC](https://www.amazon.ca/dp/B0BTH3ZRDH?psc=1&ref=ppx_yo2ov_dt_b_product_details)
- [Hosim 1600mAh Li-Polymer Battery](https://www.amazon.ca/gp/product/B098QSLJKV/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
- [LoveRPi Wide Angle Camera + CSI Cable](https://www.amazon.ca/gp/product/B07KF6QVPY/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
- [DKARDU DPST Switch](https://www.amazon.ca/gp/product/B09TKMR8J4/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
- [Kallaudo 90 Degree USB Printer Cable](https://www.amazon.ca/dp/B0C39V3J9T?ref=ppx_yo2ov_dt_b_product_details&th=1)

Our 3D-printed parts weigh roughly 75 grams, which is around $2.50 worth of filament. They consist of a platform and pieces to create a camera mount, printed by a Bambu Lab P1P 3D printer.

![3D Printed Parts](image-link.png)

Total Cost: `$182.84` + `$57.99` + `$229.99` + `$13.36` + `$39.99` + `$39.99` + `$39.99` + `$13.40` + `$14.49` + `$2` respectively = `$634.04` and `$716.47` with tax. (prices in CAD)
***
### Assembly

#### Mobility

We used a prebuilt RC car chassis and made modifications to integrate the rest of our components onto the body of the car, including a new DC motor, a 3D printed platform, and circuit boards (all parts can be found in the Parts List above). In addition, the receiver that allows the car to be controlled remotely was removed, as it was unnecessary and took up space.

The robot moves through two different motors; a DC motor for driving and a servo motor for steering. The servo motor we use came attached to the prebuilt base, but we changed out the DC motor. The new DC motor is attached similarly to the original, under the 3D printed platform, and takes up roughly the same amount of space. It is held tightly with a metal bracket and screw to prevent it from moving.

We chose this motor because there are various YouTube videos of it being implemented on our exact RC car chassis. It is also a similar size to the original motor, making it easier to replace. The new motor is also a brushless motor, as compared to the original which is a brushed motor. Brushless motors work similarly to brushed motors using the same principles of magnetic attraction/repulsion but don't have the "brushes" that would normally cause friction and reduce the motor's efficiency.

The other component under the platform is the ESC (electronic speed controller), which is wired to and controls the speed of the newly added DC motor. It is also connected to the Hosim 1600mAh Li-Polymer Battery, placed right next to it, supplying a sufficient 7.4V.

The motors are controlled by the Arduino UNO R3 board, placed on top of the platform using standoffs.

To connect the Arduino, ESC, DC motor, and Servo motor together, we use a custom-designed perfboard which all the wires plug into to form a circuit. The Arduino uses two PWM (Pulse Width Modulation) ports to control the signals sent to both of the motors and an additional port to ground the circuit. The motors use 3 ports/wires each for power, signal, and ground. 

The DC motor is wired to and receives power directly from the ESC (which is connected to the battery), but the servo motor receives it from a BEC (Battery Eliminator Circuit) inside the ESC, which is designed to provide a regulated and constant voltage supply to power low-power electronics, such as the servo motor.
***
#### Power and Sensing

Beside the Arduino Board is the Vemico Raspberry Pi 4 Board, along with a fan for cooling and a SanDisk 64GB Ultra microSD Card to hold our programs. Wired and mounted on top of it is the PiSugar Module Platform Attachment, which also is fed power from the PiSugar S Pro 5000 mAh Lithium Battery. The battery provides 3.7V, which is then converted to 5V using a voltage regulator to meet the operating voltage requirement of the Raspberry Pi.

We use two separate batteries for the ESC and Raspberry Pi to manage their battery lives independently, as they have different power/voltage requirements. It avoids the problem of having one circuit taking too much power from the battery and leaving an inadequate amount for the other.

The last component is the camera, which is mounted with the help of a custom 3D-printed stand and is wired to the Raspberry Pi through a CSI Cable. Although it is more complicated to use, we decided it would be more power-efficient than multiple ultrasonic/color sensors.

The Raspberry Pi is placed on top of a black box frame specifically made for it, holding it squarely in place, while also housing the fan inside to prevent the Raspberry Pi from overheating.

A DPST switch connects to the circuits of both the Raspberry Pi and the ESC, making it possible to turn on both the Raspberry Pi and ESC with one switch. This was done to satisfy the condition of having one power switch to turn on all the components of the car.

#### Photos
![Photos](image-link.png)

#### Schematic
![Schematic](image-link.png)
***
## Initialization and Connection Process

Our Raspberry Pi setup involves downloading the Raspberry Pi OS, writing it to a microSD card using Raspberry Pi Imager ([Download Here](https://www.raspberrypi.com/software/)), and configuring SSH and Wi-Fi. After inserting the SD card, powering up the Pi, and identifying its IP address on the network, we can remotely access it via PuTTY ([Download Here](https://www.putty.org)) for SSH and VNC Viewer ([Download Here](https://www.realvnc.com/en/connect/download/viewer/)) for remote desktop control.
***
## Program Arrangement and Software Elements

Our SBC (Single Board Computer) is the Vemico Raspberry Pi 4 Board, which operates on Raspberry Pi OS. Our SBM (Single Board Microcontroller) is the Arduino UNO R3 Controller Board. 

Put simply, the Arduino controls the movement, and the Raspberry Pi handles the algorithms and provides the Arduino with the signals to write to the motors. The Raspberry Pi is connected to the Arduino through a USB-A to USB-B cable and communicates with the Arduino through serial communication.

We use Python for the Raspberry Pi code and a variant of C++ designed for Arduino for the Arduino code.
***
### Image Processing

#### Libraries in use:

- `OpenCV` (for computer vision)
- `picamera2` (for camera control)
- `serial` (for serial communication)
- `RPi.GPIO` (for GPIO control on the Raspberry Pi)
- `numpy` (for numerical operations)

The camera captures an image which is first converted from BGR (color) to grayscale using OpenCV. Binary thresholding is applied to create a binary image, which emphasizes the lanes by setting pixel values to black for areas of interest (walls) and white (other). The script detects the contours of the left and right lanes using the thresholded binary image. The contours are extracted within specific regions of interest (ROIs) defined earlier in the code.

The following algorithms are only used in the obstacle challenge, which requires slightly more camera vision. A key difference between these programs is the use of HSV (hue, saturation, and vibrance, a method of color representation similar to RGB) to detect color rather than using grayscale
***
#### Signal Pillar Detection

The program detects the contours of signal pillars (red and green) using color masks based on their respective HSV color ranges. The contours are then extracted within a specified ROI for obstacles.
***
#### Corner Detection

Similar to the signal pillar detection, the program detects the blue and orange line contours within another specified ROI using HSV color ranges and uses this to turn. Since the obstacle challenge requires the robot to frequently navigate close to walls in order to avoid obstacles, using the same method of turning as the open challenge (mentioned in the next topic) is very inconsistent and can lead to turns at the wrong times.
***
### Open Challenge

Fundamentally, the Open Challenge only has two parts to it; wall steering and turning 90 degrees.

Our wall steering program utilizes our camera to differentiate between the black walls and everything else. However, we don't actually look at the entire camera feed and rather only focus on three specific areas, which are ROIs as mentioned earlier. Two of the three regions of interest are used for wall following while the third region of interest is used for counting turns. These two regions of interest were distinctively chosen so that when the robot is placed in the center of the track, the ROIs will both detect a similar amount of black pixels from the wall. This way, we know we are centered on the track when the two ROIs have similar amounts.

In addition, we also use a PD (Proportional and Derivative) follower algorithm to ensure that we can properly and smoothly adjust back on track based on the difference of pixels between the ROIs. Although the standard is to use a PID follower (Proportional, Integral, and Derivative), the "I" is not necessary for a closed and predictable environment as the game field is.

The second part, turning is pretty simple. When we detect that one ROI has a significantly low number of black pixels, then we turn in that direction, as it means that we are nearing a corner. Once that region of interest returns to a larger number of pixels, we end the turning program and return to the wall-following algorithm.

To keep track of our laps, we count each time the turning program is run, up until 12 (4 corners x 3 laps), and stop the robot once its orientation is relatively straight. We also check to see if our camera actually sees an orange line while turning through the third region of interest, to ensure that we are at the corner.

[Link to Video](https://www.youtube.com/watch?v=yourvideo)
***
### Obstacle Challenge

The obstacle challenge brings the addition of one more segment of code; obstacle avoidance, along with a change in the turning algorithm.

Avoiding the obstacles first involves finding the signal pillars through the signal pillar detection as explained above. 

We utilize both the x and y coordinates of the signal pillar to determine how to navigate. Both the green and red signal pillars have a different set x-coordinate target on the frame that the x-coordinate of the currently detected signal pillar must reach in order for the car to pass on the correct side. The target of the red pillar is on the left side of the frame as we need to pass on the right and the target of the green pillar is on the right side of the frame as we need to pass on the left. 

We calculate the error by taking the difference between this target x-coordinate and its current x-coordinate. Similar to wall following, we use a PD controller with different P and D values optimized for object avoidance as well as another value that changes the adjustment based on how close the pillar is to the car by tracking the y-coordinate, to adjust the car to avoid the obstacle. 

Once the obstacle exits the region of interest and is no longer detectable. The car will go back to the wall following as explained in the Open Challenge Section until it locates another signal pillar or an orange or blue line to turn. 

During turns, when the camera detects a new signal pillar while not currently detecting a blue or orange line, the number of turns is updated by one with the car switching back to obstacle avoidance. This is done to avoid miscounting turns as our car sometimes sees the next pillar before passing the orange or blue line. 

[Link to Video](https://www.youtube.com/watch?v=yourvideo)
***




