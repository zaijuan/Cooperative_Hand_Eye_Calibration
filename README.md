# Cooperative_Hand_Eye_Calibration
These scripts are about performing cooperative hand-eye calibration, that is recovering the transform between the robot base and the
camera frame that is mounted on it.
Two robots cooperates to collect measurement data from chessboard placed in the world reference as well as 
the marker board mounted on the robot.
In the end we solve the equation AXBX_=YCY_D, where X and Y are unknowns, X_ and Y_ represent the inverse of X and Y, A, B, C and D
are measurements.
