# EDUKIT Inverted Pendulum

This repository contains a modified version of the firmware for the UCLA/STM EDUKIT inverted pendulum apparatus.
The revised application restricts the operation of the motor controller to accept external requests
over a serial connection, from a controller that is also reading back the rotor encoder position
from the same serial connection.

Original author of the modifications : Jawad Muhammad, software engineer at UKAEA.

## github fork 

Adam Stephen (github handle AdamVStephen) has created this fork of Jawad Muhammad's work
mostly to enable ease of collaboration with colleagues from other institutes.

## Overview

This is one of two repositories which can be used to explore the functionality provided
by the excellent UCLA/STM EDUKIT01 apparatus and firmware.  

This repository holds the code that runs on the STM32 controller.  This has been adapted so
that control commands are provided by an external application running on a separate computer
and connected to the STM32 over a serial link.

The [companion repository](https://github.com/AdamVStephen/inverted-pendulum)
provides implementations of the serial protocol support and the control algorithm.
These are written using the [MARTe2](https://github.com/ukaea/MARTe2.git) C++
real-time framework (coming out of fusion energy research).

## Original STM and UCLA Documentation

The original team provide
information on the two sites below.  The UCLA site has links to a number of good YouTube
videos.  They also provide comprehensive tutorials that explain control theory design,
how to work with the STM32 tools and more.

1. [STM Main Site](https://www.st.com/en/evaluation-tools/steval-edukit01.html)[]
1. [UCLA Samueli School of Engineering Site](https://sites.google.com/view/ucla-st-motor-control/home)

The original source code is on github courtesy of one of the [UCLA team](https://github.com/wjkaiser).

1. [Edukit Source](https://github.com/wjkaiser/Edukit_Rotary_Inverted_Pendulum_Project)

## Install Instructions

The repository includes a pre-built image for the STM32 board to allow ease of use for anyone
who is more interested in the MARTe2 control code than the embedded programming side.

The binary image `STM32F401RE-Nucleo.bin` is provided in a deeply nested folder structure
(this is imposed by the STM32Cube project layout).  Either use `find` or navigate to 

```
Edukit_Rotary_Inverted_Pendulum_Project/
	Projects/
		Multi/
			Examples/
				MotionControl/
					IHM01A1_ExampleFor1Motor/
						SW4STM32/
							STM32F401RE-Nucleo/
								Debug/
```

Use the STM32CubeProgrammer to download the image to the board.

Now follow the instructions from the 
[companion repository](https://github.com/AdamVStephen/inverted-pendulum)
to install the MARTe2 libraries and to compile the control application.
Connect a suitable serial connection.  This can work with Windows/WSL given 
suitable use of the `usbipd` tools to map the com port to WSL.  Alternatively
use a native Linux machine (x86_64 is supported, as is arm64 on raspberry pi)
which is the more typical home for MARTe2.

The control application should be started, and then the STM32 application restarted
from the blue reset button on the board.

If all goes well, the pendulum will swing into life and the control application
will keep it there.
## License / Copyright

In respect of the original work by STM/UCLA, those entities respectively.

In respect of the additions, Jawad Muhammad and his employer.  UKAEA will look to 
make a public release of this code as soon as their internal process for reviewing
and publishing code has been completed.
