# NLC-Project-Part-B
This repository holds an LQR controller, a gain scheudling controller, and a (partially completed) feedback linearization controller designed for simulating a quadcopter on PX4 through jMAVSim.

# Steps to Run Controllers
In order to use the lqr controller (lqr_control), please complete the following steps:
1) Place the lqr_control folder unzipped in PX4-Autopilot\src\examples.
2) Inside PX4-Autopilot\boards\px4\sitl\default.cmake, add lqr_control to the examples list.
3) Insdie PX4-Autopilot\src\modules\mc_rate_control\MulticopterRateControl.cpp, comment out lines 303, 308, 309, and 310.
4) Inside PX4-Autopilot\src\examples\lqr_control\lqr_control_main.cpp, change the file location in lines 99 and 100 to match with your local directory.

In order to use the gain scheudling controller (gs_control), please complete the following steps:
1) Place the gs_control folder unzipped in PX4-Autopilot\src\examples.
2) Inside PX4-Autopilot\boards\px4\sitl\default.cmake, add lqr_control to the examples list.
3) Insdie PX4-Autopilot\src\modules\mc_rate_control\MulticopterRateControl.cpp, comment out lines 303, 308, 309, and 310.
4) Inside PX4-Autopilot\src\examples\gs_control\gs_control_main.cpp, change the file location in lines 114-142 to match with your local directory.

As alluded to above, the feedback linearization controller is not ready to be excuted, feel free to add to it!

# Report
A report was created summarizing and analyzing the controller performance through matlab simulation and PX4 simultion
OVERLEAF Report Link: https://www.overleaf.com/4463832759kznjknvbtdtb
