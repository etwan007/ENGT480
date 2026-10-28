# ENGT481

Repository for ENGT 480/481 Senior Design Capstone (Triple Inverted Pendulum project)

**Raspberry Pi setup**
 - The python libraries are installed in a venv virtual environment called 'PythonLibraries'.
 - path: /home/engt481/PythonLibraries
 - Thonny is set up to use the python libraries from this environment, if you want to use a different editor you'll need to set this up.
 - Activation command for running scripts in terminal (from engt481 directory): source PythonLibraries/bin/activate
 - Deactivation command: deactivate

**Single pendulum LQG (Linear Quadratic Gaussian) setup:**
 - Run communication test program and hold up pendulum to get a rough estimate of the value when straight up.
 - If the angle is close to the rollover point (near 0 or 16383), rotate the pendulum about the shaft so the rollover point will be near pointing down.
 - replace the value for the 'correction' variable with the measured value for straight up.
 - start with the 'ANGLE CORRECTION', and 'ANGLE TUNING' lines enabled, and run the program until the pendulum centers near the middle (it may oscillate a bit).
 - Comment out the 'ANGLE CORRECTION' line, and replace the 'correction' variable with the final printed correction value. Run again until the pendulum is centered.
 - Comment out the 'ANGLE TUNING' lines, and run again with the new correction value. Make sure that it centers itself very close to the starting point.
 - The first pendulum angle should be tuned at this point, and be usable for the any of the control scripts.

**Double pendulum LQG setup:**
 - Use the single pendulum script first to tune the first pendulum angle
 - Run the communication test program, and let the pendulums hang. Record the second pendulum angle to use for the 'correction2' variable value.
 - The control script should be ready to run at this point.
 - To adjust tuning, or physical measurements of the pendulum parts, update the values in the math script, and run it. this will update the control_matrices.txt file.
 - To adjust the abstract dynamics, make sure to delete the .pkl file, and then run the math script to generate a file for the new dynamics. This will take a long time to run (only when re-generating the .pkl file), so run this on a personal computer. it will take from 15-30 minutes to calculate.
 - The pendulum will likely not stabilize on the first test, so methodically mess with the 'correction2' value until the pendulum is able to stay upright (Increasing the value brings it left, decreasing brings it right)
 - If the pendulum is slowly drifting off, use a finger to lightly touch the 2nd pendulum prevent it from going off the rail while it tunes. It should slowly correct 'correction' until the pendulum begins to center itself. It will oscillate at first, but should stabilize eventually in the center.

**Triple Pendulum LQG:**
 We had not attempted this as of writing this. Good luck! This will require a more sophisticated control system than linear LQG. Here are some ideas we had:
  - Could try re-linearizing the system every time step to approximate a non-linear system with LQG. (Raspberry pi may not be powerful enough if not well optimized)
 - Could try model predictive control (MPC). (Raspberry pi may not be powerful enough if not well optimized)
 - Could try a machine learning approach. 


**Things to be careful of:**
 - Printing to terminal takes a surprising amount of time. Minimize print statements in the control loop, and round printed float values. Sometimes angles being dropped and triggering the filter messages can be because the control loop took too long (because of printing), not actually because a bad angle was received from the encoder.
 - Make sure to power down the raspberry pi properly. If you just cut the power, it could cause it to corrupt the github. (This is fixable, just annoying to deal with)

**Description of the Control System**

The current control system uses LQG (Linear Quadratic Gaussian). This is just LQR (Linear Quadratic Regulator) with a Kalman Filter. The control loop starts by taking the previous measurements of the system, and estimating the current states. the states are a matrix that contain all the necessary information to describe the pendulum's motion in a given instant. For the single pendulum this contains the angle, angular velocity, cart position, and cart velocity. After this step, the raspberry pi waits for the angles to be sent over serial by the esp32. These actual measurements are used to update the kalman filter estimations and make them more accurate. Then the control loop calculates the control force to apply, and pulls the control velocity from the kalman filter estimation of the velocity. The kalman filter is useful because it not only filters out noise in the measurements, but it also estimates the states that aren't measured based on the dynamics of the system. The control loop uses the control velocity 'xDiv' to calculate the frequency to pulse the stepper motor, then converts that to control the 2560 hardware timer. It sends this to the arduino, which then outputs the control frequency. After sending this, the arduino returns the current position of the cart (It tracks this by counting steps sent to the motor with an interrupt), and the loop restarts. The order of the loop means that the position value will always be a loop old, but this doesn't really matter since the position is far less critical than the angles to proper balancing.
    
The first part of the single pendulum code (or the math script for the double) start with the differential equations that describe the system. We are using lagrange derivations since they scale much more easily than dealing with forces and torques directly, and produce the exact same end result. T represents the total kinetic energy of the system, and U represents the potential energy. We found the double pendulum equations in a paper (linked under helpful resources), and triple pendulum equations are available in papers also. The script converts these equations into a state space model that is set up like this: Y_derivative(matrix of state derivatives) = a matrix of equations with variables for the states. The state space model is then linearized around the stabilization point (all angles, positions and velocities = 0), and the A and B matrix are created. The C and D matrices are input manually, and represent which states are estimated, and which are measured. The C matrix is and identity matrix with each row that represents an estimated state removed. The D matrix has an element for each measured state. 
    
These linearized matrices are discretized. This converts them from 'Y_dot = A x Y + B x u' to 'Y_new = A x Ylast + B x u'. This slightly simplifies control loop calculations to keep from having to integrate Y_dot. These matrices are also used to calculate the kalman filter matrices and the LQR gain matrix.
    
Kd, Sd, Ed = ctrl.dlqr(ssDisc, sp.diag(100,1,100,1,20,10), 0.1)
Kd is the LQR gain matrix, this is multiplied by the current state matrix to get the control force. the sp.diag(theta1_weight, w1_weight, theta2_weight, w2_weight, x_weight, xDiv_weight) contains values that represent how much the controller cares about keeping each state close to zero. The last value (0.1 in the example line) represents how much the controller tries to limit the control force (bigger values try to limit more). Smaller values mean the controller doesn't care much about managing that state.
   
Ld, Pd, Edkalm = ctrl.dlqe(ssDisc.A, sp.diag(1,1,1,1,1,1), ssDisc.C, sp.diag(0.2,0.001,0.2,0.001,0.2,0.001), 0.01*sp.diag(1,1,1))
This line calculates the kalman filter matrices. sp.diag(theta1_weight, w1_weight, theta2_weight, w2_weight, x_weight, xDiv_weight) determines how much the controller trusts the measured values for each. The values for estimated states don't matter (these are set to 0.001 in the example line). A value of 1 fully trusts the estimated state, and a value of 0 fully trusts the measurement. You likely won't have a reason to mess with the tuning of this.
    
Because this control system is linearized, the current system cannot just be scaled up for the triple pendulum, it won't be accurate enough.

**Helpful resources:**
 - Matlab pendulum simulation tutorial:
https://www.mathworks.com/help/symbolic/derive-and-simulate-cart-pole-system.html
 - Link to double pendulum paper where we found dynamic equations:
https://www.researchgate.net/publication/348468205_Modeling_and_Control_of_a_Double_Inverted_Pendulum_using_LQR_with_Parameter_Optimization_through_GA_and_PSO

**Suggestions:**
 - A way to tune the angles more precisely could be helpful. Maybe a program could be designed to help find the second pendulum angle more easily and precisely?
 - Bearing friction and air resistance are not included in the model. These may need to be modeled for the triple pendulum.
 - This is probably uneccesary, but if a more powerful control computer is needed (replacing the raspberry pi 4) it would likely be easy to switch it out. The python code is not raspberry pi specific (at least not that I am aware), and should be easy to adapt.

**Contact**

If you have a question about the system that we forgot to address:
 - Samuel Whitesell: whitesell.samuel@gmail.com
 - Ryan Hanshew: hanshew.ryan@gmail.com
 - Patrick Clark: Patrickgclark09@gmail.com
 - Edward Morris: emmorris853@gmail.com
 - Cooper Dlugos: Coopersd036@gmail.com
