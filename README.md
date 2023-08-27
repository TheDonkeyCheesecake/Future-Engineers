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

- Brian Yin, 16, [brianyin256@gmail.com](mailto:brianyin256@gmail.com)
- Eric Rao, 15, [ericrao08@gmail.com](mailto:ericrao08@gmail.com)
- Weston Law, 15, [westonlaw@hotmail.com](mailto:westonlaw@hotmail.com)

![Team Members](image-link.png)
***
## Hardware List and Assembly

### Parts List

- [LaTrax Rally RC Car Chassis](https://latrax.com/products/rally)
- [Arduino UNO R3 Controller Board + wires/cables](https://www.amazon.ca/gp/product/B01D8KOZF4/ref=ppx_yo_dt_b_search_asin_image?ie=UTF8&psc=1)
- [Vemico Raspberry Pi 4 Board + Fan](https://www.amazon.ca/gp/product/B09WXRCYL4/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
- [SanDisk 64GB Ultra microSD Card](https://www.amazon.ca/gp/product/B073JYVKNX/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&th=1)
- [PiSugar S Pro 5000 mAh Lithium Battery + PiSugar Module Platform Attachment](https://www.amazon.ca/Portable-Platform-Raspberry-Accessories-handhold/dp/B097RCFHD2)
- [GoolRC 2435 Brushless Motor + 25A ESC](https://www.amazon.ca/gp/product/B01CCRZHNI/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
- [LoveRPi Wide Angle Camera + CSI Cable](https://www.amazon.ca/gp/product/B07KF6QVPY/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)
- [DKARDU DPST Switch](https://www.amazon.ca/gp/product/B09TKMR8J4/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)

Our 3D printed parts weigh roughly 40 grams, which is around 2 dollars of filament cost. They consist of a platform and camera mount, printed by the Bambu Lab P1P printer.

![3D Printed Parts](image-link.png)

Total Cost: `$182.84` + `$57.99` + `$229.99` + `$13.36` + `$39.99` + `$32.99` + `$13.40` + `$14.49` + `$2` respectively = `$587.05` and `$663.37` with tax. (prices in CAD)
***
### Assembly

#### Mobility

We used a prebuilt RC car chassis and made modifications to integrate the rest of our components onto the body of the car, including a new DC motor, a 3D printed platform, and circuit boards (all parts can be found in Parts List above). In addition, the receiver that allows the car to be controlled remotely was removed, as it was unnecessary and took up space.

The robot moves through two different motors; a DC motor for driving and a servo motor for steering. The servo motor we use came attached to the prebuilt base, but we changed out the DC motor. The new DC motor is attached similarly to the original, under the 3D printed platform, and takes up roughly the same amount of space. It is held tightly with a metal bracket and screw to prevent it from moving.

We chose this motor because there are various YouTube videos of it being implemented on our exact RC car chassis. It is also a similar size to the original motor, making it easier to replace. The new motor is also a brushless motor, as compared to the original which is a brushed motor. Brushless motors work similarly to brushed motors using the same principles of magnetic attraction/repulsion but don't have the "brushes" that would normally cause friction and reduce the motor's efficiency.

The other component under the platform is the ESC (electronic speed controller), which is wired to and controls the speed of the newly added DC motor. It is also connected to the 1600mAh battery, placed right next to it, supplying a sufficient 7.4V.

The motors are controlled by the Arduino UNO R3 board, placed on top of the platform using standoffs.

To connect all the components together, we use a custom-designed perfboard. The Arduino uses two PWM (Pulse Width Modulation) ports to control the speed of the two motors and an additional port to ground the circuit. The motors use 3 ports/wires each for power, signal, and ground.

The DC motor is wired to and receives power directly from the ESC (which is connected to the battery), but the servo motor receives it from a BEC (Battery Eliminator Circuit) inside the ESC, which is designed to provide a regulated and constant voltage supply to power low-power electronics, such as the servo motor.
***
#### Power and Sensing

Beside the Arduino Board is the Vemico Raspberry Pi 4 Board, along with a fan for cooling and a SanDisk 64GB Ultra microSD Card to hold our programs. Wired and mounted on top of it is the PiSugar Module Platform Attachment, which also is fed power from the PiSugar S Pro 5000 mAh Lithium Battery. The battery provides 3.7V, which is then converted to 5V using a voltage regulator to meet the operating voltage requirement of the Raspberry Pi.

We use two separate batteries for the ESC and Raspberry Pi to manage their battery lives independently, as they have different power/voltage requirements. It avoids the problem of having one circuit taking too much power from the battery and leaving an inadequate amount for the other.

The last component is the camera, which is mounted with the help of a custom 3D printed stand and is wired to the Raspberry Pi through a CSI Cable. Although it is more complicated to use, we decided it would be more power-efficient than multiple ultrasonic/color sensors.

The Raspberry Pi is placed on top of a black box frame specifically made for it, holding it squarely in place, while also housing the fan inside to prevent the Raspberry Pi from overheating.

A DPST switch controls the power of the Raspberry Pi and Arduino to satisfy the condition of having one power button.

#### Photos
![Photos](image-link.png)

#### Schematic
![Schematic](image-link.png)
***
## Initialization and Connection Process

Our Raspberry Pi setup involves downloading the Raspberry Pi OS, writing it to a microSD card using Raspberry Pi Imager ([Download Here](https://www.raspberrypi.com/software/)), and configuring SSH and Wi-Fi. After inserting the SD card, powering up the Pi, and identifying its IP address on the network, we can remotely access it via PuTTY ([Download Here](https://www.putty.org)) for SSH and VNC Viewer ([Download Here](https://www.realvnc.com/en/connect/download/viewer/)) for remote desktop control.
***
## Program Arrangement and Software Elements

Our SBC (Single Board Computer) is the Vemico Raspberry Pi 4 Board, which operates on Raspberry Pi OS. Our SBM (Single Board Microcontroller) is the Arduino UNO R3 Controller Board. Put simply, the Arduino controls the movement, and the Raspberry Pi handles the algorithms.

We use Python for the Raspberry Pi code and a variant of C++ designed for Arduino for the Arduino code.
***
### Image Processing

#### Libraries in use:
***
- OpenCV (for computer vision)
- picamera2 (for camera control)
- serial (for serial communication)
- RPi.GPIO (for GPIO control on the Raspberry Pi)
- numpy (for numerical operations)

The camera captures an image which is first converted from BGR (color) to grayscale using OpenCV. Binary thresholding is applied to create a binary image, which emphasizes the lanes by setting pixel values to black for areas of interest (walls) and white (other). The script detects the contours of the left and right lanes using the thresholded binary image. The contours are extracted within specific regions of interest (ROIs) defined earlier in the code.

The following algorithms are only used in the obstacle challenge, which requires slightly more camera vision. A key difference between these programs is the use of HSV (hue, saturation, and vibrance, a method of color representation similar to RGB) to detect color rather than using grayscale
***
#### Signal Pillar Detection
***
The program detects the contours of signal pillars (red and green) using color masks based on their respective HSV color ranges. The contours are then extracted within a specified ROI for obstacles.
***
#### Corner Detection
***
Similar to the signal pillar detection, the program detects the blue and orange lines using HSV color ranges and uses this to turn. Since the obstacle challenge requires the robot to turn very frequently, using the same method of turning as the open challenge (mentioned in the next topic) is very inconsistent.
***
### Open Challenge
***
Fundamentally, the Open Challenge only has two parts to it; wall steering and turning 90 degrees.

Our wall steering program utilizes our camera to differentiate between the black walls and everything else. However, we don't actually look at the entire camera feed and rather only focus on two specific areas, which are ROIs as mentioned earlier. These two regions of interest were distinctively chosen so that when the robot is placed in the center of the track, they will have a similar amount of black pixels from the wall. This way, we know we are on track when the two ROIs have similar amounts.

In addition, we also use a PD (Proportional and Derivative) follower algorithm to ensure that we can properly and smoothly adjust back on track based on the difference of pixels between the ROIs. Although the standard is to use a PID follower (Proportional, Integral, and Derivative), the "I" is not necessary for a closed and predictable environment as the game field is.

The second part, turning is pretty simple. When we detect that one ROI has significantly fewer black pixels than the other, then we turn in that direction, as it means that we are nearing a corner. Once that region of interest returns to its usual amount of pixels, we end the turning program and return to wall following.

To keep track of our laps, we count each time the turning program is run, up until 12 (4 corners x 3 laps), and stop the robot. We also check to see if our camera actually sees an orange line while turning, to ensure that we are at the corner.

[Link to Video](https://www.youtube.com/watch?v=yourvideo)
***
### Obstacle Challenge

The obstacle challenge brings the addition of one more segment of code; obstacle avoidance, along with a change in the turning algorithm.

Avoiding the obstacles involves changing the values of P and D in our PD movement control. Rather than following the middle of the track as we usually do, a change in these values allows us to follow closer to either side of the track, avoiding the obstacle.

In order to turn back to the middle of the track, we wait until the obstacle nears the edge of our camera view. Once it reaches a given point on the length of our camera feed, we reset the P and D values to re-center the robot.

We changed the turning method in this challenge because the previous one had difficulty working consistently with the number of other turns we do to avoid the obstacles in this challenge. When the robot is on an angle, usually while turning, the previous method can trigger when it isn't supposed to.

[Link to Video](https://www.youtube.com/watch?v=yourvideo)
***




