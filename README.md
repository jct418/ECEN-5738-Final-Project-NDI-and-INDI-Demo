# ECEN 5738 Final Project NDI and INDI Demo
Group Members:
- Matticus Brown
- Josh Taylor
- Quinn Gossett

This demo requires the following Matlab packages:
- Control Systems Toolbox
- Signal Processing Toolbox
- Communications Toolbox

This repo contains the code used for our in-class demo of an inverted pendulum cart system.

# How To Use
To run the demo, open and run Pendulum_Demo.m. Alternatively, this script can be opened and then run in separate sections. The "Setup" section handles all variable definitions, with each of the following sections handling the simulation of the inverted pendulum cart. The three cases currently implemented are NDI with an accurate plant, NDI with an inaccurate plant, and INDI with an inaccurate plant. When run, each section will produce a plot showing each state's progression overtime, along with an animation of the system simulated. Additionally, the L2 signal norm of the cart position error and pendulum angle error will be printed in the command window, to allow easy comparisson between cases. Note that noise can be added to the system by setting the noise variable in "Setup" to true.

To replicate the results presented during the in-class demo, keep all variables as is. Noise can be toggled on and off, as we did demonstrate that case during our demo.

# Description of Files
Pendulum_Demo.m:
This is the main script, which handles execution of all sub-tasks, and handles the declaration of key variables, such as the initial condition (x0) vector and the time (tspan) vector.

GetLinDyn.m:
This file contains a function, which returns the linearized A and B matrices for the system, give the system's physical parameters. Primarily used for the LQR formulation.

InvPenINDI.m:
This file contains the function representing the system being controlled by an INDI controller.

InvPenNDI.m:
This file contains the function representing the system being controlled by an NDI controller.

rk4.m:
This file contains a slightly customized implementation of a traditional Runge-Kutta 4 solver. This custom RK4 solver allowed us the solve the simulate the system with a fixed timestep. Additionally, this RK4 imp[lementation passes the previous force input through to the next iteration of the system. Both of these are crucial for running an INDI controller effectively.

PenPlot.m:
This file handles the plotting and animation of the system. The computation and display of the L2 signal norm of the state errors is also handled by this file.