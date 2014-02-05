dumbo_cart_vel_controller
=========================

Cartesian velocity controller for CVAP's Dumbo robot.

Takes as input twists of the **<left or right>_arm_7_link** (the wrist) expressed in the base frame and the joint positions of the arm and outputs joint velocity commands calculated through damped least squares (DLS).
