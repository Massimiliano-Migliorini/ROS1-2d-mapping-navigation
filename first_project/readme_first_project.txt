Theoretical assumption: in both nodes the Odom reference frame corresponds to the static position of the robot, whereas the local reference frames linked to the car, precisely vehicle
 			and gps have the x axis pointing to the car heading.

Consequences of the convention:
1) To integrate the odometry in node 1 we have selected a theta_init = 0, in order to overlap Odom and Vehicle at the starting pose and be consistent with the odometry formulas. 
2) To compute odometry from gps data, after having transformed (lat, lon, alt) into ENU coordinates, it has been necessary to perform a rotation from ENU to Odom equal to the static
   orientation of the car.



          