||   -Engineers Documentation-   ||


Repository containing the engineering elements and details of Team XXXXXX from Canada, participating in the 2023 WRO Future Engineers competition.


|   -Repository Content-   |

`chassis` - details of our robot chasis
`models` - 3D printable files designed by us
`schemes` - schematics of our robot's electrical systems
`src` - main and other programs to run/control software
`t-photos` - one serious and one funny photo
`v-photos` - photos of our robot from each angle
`video` - youtube link of our robot running each program on the game field
`others` - other essential files




|   -Hardware List and Assembly-   |


-Parts List-

 - pluh
 - pluh
 - bruh

**add 3d printed parts + total cost later**

-Assembly-

We used a prebuilt RC car chassis and made modifications to integrate the rest of our components onto the body of the car, including a new DC motor, a 3D printed platform, and circuit boards (all parts can be found in *Parts List* above). In addition, the reciever that allows the car to be controlled remotely was removed, as it was unnecessary
and took up space. 

The robot is moves through two different motors; a DC motor for driving and a servo motor for steering. The servo motor we use came attached with the prebuilt base, but we changed out the DC motor. The new DC motor is attached similarly to the original, under the 3D printed platform, and takes up roughly the same amount of space. It is held tightly with a metal bracket and screw to prevent it from moving. 

The other component under the platform is the ESC (electronic speed controller), which, with the help of a *thingy that has the wire stick thingys and the sautered stuff*, is wired to and controls the speed of the newly added DC motor and the original servo motor.

The platform itself is mounted via 4 standoffs. One end (female) is held to the base of the car by superglue, while the other end (female), protrudes through the platform and is held in place by a screw (*i think i gotta see the robot*). 

On top of the platform are the circuit boards, one of which is a *Raspberry Pi while the other is an *Arduino (*i gotta check the actual names of them*). Put simply, the Arduino controls the movement of the robot, sending commands to the motors, while the Rasperry Pi controls all the image processing and complex algorithms. In addition, there is also a fan for the *Raspberry Pi for cooling, a switch for the ESC and a DPST switch that controls the power of both the Raspberry Pi and Aruduino to satisfy the condition of having one power button. The last component is the camera, which is mounted with the help of a custom 3D printed stand and is wired to the Raspberry Pi through a *funky white strip wire thing.

The camera and Arudino Board are mounted with the same standoffs. Four legs hold the Arduino up, which are then screwed into more standoffs on top of them. These standoffs support the camera mount while also fastening the Arduino in place.

The Raspberry Pi is placed ontop of a black box frame that came with it and holds it squarely in place, while also housing the fan inside to prevent the Raspberry from overheating.

The ESC switch is placed on top of the platform for accessibilty, mainly while debugging.

 
