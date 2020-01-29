# Simulation of Soccerbots
<br>
<h2> Instructions to get it running:</h2>
<p> Install ROS kinetic on Ubuntu 16.04 (Developed and tested) </p>
<p> Install pygame, numpy for python 2.7 </p>
<p> Inside location_of_catkin_ws/src/ mkdir sbsim</p>
<p> Inside sbsim clone this repository</p>
<p> go to location_of_catkin_ws and catkin_make</p>

<br>
<h2> Work done </h2>
<img src="rosgraph.png">
<br>
<h3>3 controller implementation</h3>
<p>Point to point control</p>
<img src="p2p.gif">
<p>Trajectory Tracking control</p>
<img src="traj_tracking.gif">
<p>3 wheel independent control</p>
<img src="extremal.gif">

<h3> Proposed Rules For the game:</h3>
    * Robot's ball possession is legal only for x distance travelled. (x is tbd) <br/>
    * If x is reached, bot must pass, shoot or kick ball and regain possession. <br/>
    * Incase of collision, bot possesing the ball given freekick <br/>
    * if ball goes out outside field of play, possesion is given to the opposition at the line. <br/>

<h3> physics, controller and stateserver </h3>
    * Discrete time kinematics was used to model bot and ball. <br/>
    * Used oblique collision model with suitable coefficient of restitution and inertias for collision physics <br/>
    * Created a cascaded P controller for go to goal commands to bot <br/>
    * added dribbler and kicker physics <br/>
    * publishing goalmsgs to a particular bot triggers bot to go to goal <br/>
    * publishes data as geometry_msgs/Pose<br/>

<h3> Took data from stateserver and was able display as a simulation of the soccebots </h3>
    * Subscribes to geometry_msgs/Pose messages of ball pose, robot poses <br/>
    * Used pygame to simulate the environment <br/>

