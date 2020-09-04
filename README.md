# Building a Controller #

## Udacity Autonomous Flight Engineer Nanodegree ##

## Introduction ##

TODO: Include my diagram. 
TODO: Describe the diagram.
TODO: Sketch the drone itself and its orientation.
TODO: Include a GIF up front of the end result.

## Rubric Points ##

Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

The rubric can be found here: <https://review.udacity.com/#!/rubrics/1643/view>

---

### Writeup ###

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf ####

This document is the Writeup. Below I describe how I addressed each rubric point.

### Implemented Controller ###

#### 1. Implemented body rate control in C++ ####

> The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.

The BodyRate controller is a simple "P" controller for the body rates (p, q, r).

The only quirk is that the controller needs to convert the commanded body 
accelerations to commanded moments for the GenerateMotorCommands() function.

To do this conversion, we use the formula given at the very beginning of the
controls lectures. Lesson 1.2.

    Moment about a given axis = (Moment of Inertia for that axis) * (Acceleration about that axis)

    M_x = I_xx * u_bar_p

This results in the following code:

    pqr_error = pqrCmd - pqr;
    u_bar_pqr = kpPQR * pqr_error;
    momentCmd = moments_of_inertia * u_bar_pqr;

#### 2. Implement roll pitch control in C++ ####

> The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles.

The Roll-Pitch Controller is a simple "P" controller.

There are a few complications though:

- The inputs and outputs aren't the same. The inputs are total thrust, current attitude,desired acceleration in X and Y directions. But the outputs are angular velocities.
- The inputs are in the world frame, but the outputs are in the body frame.

First, we must get the collective acceleration.

This can be done with a=F/m, which is derived from F=ma.

The thrust must be negated because of the NED frame. It is supplied
from the Altitude Controller as a magnitude without a sign.

    collAccel = -collThrustCmd / mass;

Next, we need to get the target R13 and R23 values.

As the lecture describes, R13 and R23 are our "control knobs" for controlling the X and Y positions.

To do this, we use the formula presented in 4.17 and the Feed Forward Parameter Identification paper:
(https://www.dynsyslab.org/wp-content/papercite-data/pdf/schoellig-acc12.pdf).

Note: Per the paper: R13, R23, R33 "represent the direction of the collective thrust in the inertial frame O"

x_dot_dot = total_acceleration * R13
y_dot_dot = total_acceleration * R23

Rearranged:

target_R13 = x_dot_dot / total_acceleration
target_R23 = y_dot_dot / total_acceleration

Note: I had some confusion on these formulas. See https://knowledge.udacity.com/questions/277700.

    target_R13 = accelCmd.x / collAccel;
    target_R23 = accelCmd.y / collAccel;

The target R13/R23 values must be limited otherwise the tilt will be commanded to such a large value that the drone will flip over.

    target_R13 = CONSTRAIN(target_R13, -maxTiltAngle, maxTiltAngle);
    target_R23 = CONSTRAIN(target_R23, -maxTiltAngle, maxTiltAngle);

The "P" error can now be calculated.

    error_R13 = target_R13 - actual_R13;
    error_R23 = target_R23 - actual_R23;

With the error, we can now get the rates in the body frame. These commanded rates are the final output for this controller.

To do this, we use the formula presented in 4.17 and the 
Feed Forward Parameter Identification paper, formula #6:
(https://www.dynsyslab.org/wp-content/papercite-data/pdf/schoellig-acc12.pdf).

    p_cmd = (R(1,0) * (kpBank * error_R13) - R(0,0) * (kpBank * error_R23)) / R(2,2);
    q_cmd = (R(1,1) * (kpBank * error_R13) - R(0,1) * (kpBank * error_R23)) / R(2,2);

#### 3. Implement altitude controller in C++ ####

> The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.
> Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4.

The Altitude Controller is a full "PID" controller.

The following are the complications for this controller:

- The function must return a z force, not an z acceleration.
- Our inputs are in NED. The function should actually output a thrust, not in the NED frame.

The commanded velocity can be limited immediately, before PID calculation.

    velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);

After limiting the velocity, the standard PID calculations can be performed.

    pos_error_z = posZCmd - posZ;
    vel_error_z = velZCmd - velZ;
    integratedAltitudeError += (pos_error_z * dt);

    accel_z = (kpPosZ * pos_error_z) + (kpVelZ * vel_error_z) + (KiPosZ * integratedAltitudeError) + accelZCmd;

Now the acceleration (an acceleration) needs to be converted to thrust (a force).

Using the equation from 4.17:

    z_dot_dot = c * b_z + g

or

    z_dot_dot = collective_acceleration * R33 + g

Solve for collective_acceleration.

    collective_acceleration = (z_dot_dot - g) / R33

Note: The rotation matrix is needed as an input to this controller for R33, which helps us figure out how much of the collective acceleration contributes to the z direction.

    c = (accel_z - CONST_GRAVITY) / R(2,2);

With the collective acceleration, we can use F=ma to find the collective thrust (not in the NED frame).

    thrust_z = -(c * mass);

#### 4. Implement lateral position control in C++ ####

> The controller should use the local NE position and velocity to generate a commanded local acceleration.

The Lateral Position Controller is a straightforward "PD" controller. It also includes a feed forward term.

Calculate the PD commands.

    x_vel_cmd = kpVelXY * (velCmd.x - vel.x);
    y_vel_cmd = kpVelXY * (velCmd.y - vel.y);
    accelCmd.x = kpPosXY * (posCmd.x - pos.x) + x_vel_cmd + accelCmdFF.x;
    accelCmd.y = kpPosXY * (posCmd.y - pos.y) + y_vel_cmd + accelCmdFF.y;

The commanded acceleration can also be limited.

    accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
    accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);

#### 5. Implement yaw control in C++ ####

> The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).

The Yaw Controller is a simple "P" controller.

The only quirk is that you must wrap the yaw values to normalize them
as 0 to 2PI, or -PI to PI.

The commanded yaw can be wraped to 0 to PI as follows.

    yawCmd = fmodf(yawCmd, 2*M_PI);

The yaw error can then be calculated.

    yaw_error = yawCmd - yaw;

The yaw error should also be wrapped, but -PI to PI since the subtraction can make the value negative. This was a little confusing to me. See https://knowledge.udacity.com/questions/270792.

    yaw_error = AngleNormF(yaw_error);

Finally, the yaw command can be calculated.

    yawRateCmd = kpYaw * yaw_error;

#### 6. Implement calculating the motor commands given commanded thrust and moments in C++ ####

> The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

Keep in mind this function is going from thrust (N) to thrust (N).
The only thing we are doing is assigning the amount of thrust to each
propellor to achieve the desired 3-axis moment.

First, the "len" must be calculated for the torque calculations.

The term "L" is given to us, but it is the length of the motors to the
origin of the vehicle. For our torque calculations, we need the length
of the motors to the given axis we are rotating around.

Our drone's frame is an X pattern, and the x axis is sticking out the
nose of the drone, between the front two propellors.

    x     X
     *   *
       *
     *   *
    X     X

When you form a right triangle between the "L" length and the x axis (or y axis),
The angle is 45 degrees.

This allows us to use a little trig to get the value of the other side of the triangle.

Multiply by cos(45), or 1/sqrt(2).

    float len = L / (sqrtf(2.f));


We are given the 3 axis moment, which using the symbology from class,
that is tau_x, tau_y, tau_z.

We are also given the collective thrust, or F_total.

We want to get the F1, F2, F3, F4 values.

Note: In the following equations I have already converted the propellor numbering
to the correct numbering for this project (swap 3 and 4).

We know some equations from the Lesson 4, Full 3D Control notebook:

    F_total = F1 + F2 + F3 + F4   
    tau_x = (F1 + F2 + F3 + F4) * len
    tau_y = (F1 + F2 + F3 + F4) * len
    tau_z = -tau_1 + tau_2 + -tau_3 + tau_4

But we don't have tau_1, 2, 3, 4.
However, using kappa, we do have an alternative.

    tau_z = (-F1 + F2 + -F3 + F4) * kappa

"kappa" is derived from the two core equations we have been using for
force and torque of the propellors:

    F = kf * omega^2
    tau = km * omega^2

    kappa = F/tau = kf/km

Also, 

    tau = kappa * F

This ratio becomes useful when we want to solve our set of linear equations
for the F1/F2/F3/F4.

We have the following equation, assuming propellors 1 and 3 are rotating clockwise:

    tau_z = -tau_1 + tau_2 + -tau_3 + tau_4

Where tau_1 is the reactive moment around the z axis caused by propellor one rotating.

We can replace those tau_1/tau_2/tau_3/tau_4 using kappa!

    tau_z = -(kappa*F1) + (kappa*F2) + -(kappa*F3) + (kappa*F4)
    tau_z = (-F1 + F2 + -F3 + F4) * kappa

Now with "kappa", we can restate the set of linear equations mentioned previously.

    F_total = ( F1 +  F2 +  F3 +  F4)
    tau_x   = ( F1 + -F2 +  F3 + -F4) * len
    tau_y   = ( F1 +  F2 + -F3 + -F4) * len
    tau_z   = (-F1 +  F2 + -F3 +  F4) * kappa

Also, apply a negation to tau_z. This was a point I was confused about. See https://knowledge.udacity.com/questions/269627.

Or:

    F_total     =  F1 +  F2 +  F3 +  F4
    tau_x/len   =  F1 + -F2 +  F3 + -F4
    tau_y/len   =  F1 +  F2 + -F3 + -F4
    tau_z/kappa = -F1 +  F2 +  F3 + -F4

Solving the linear equations gives the following formulas, which were essentially converted directly to code:

    F1 = (F_total + (tau_x/len) + (tau_y/len) - (tau_z/kappa)) / 4
    F2 = (F_total - (tau_x/len) + (tau_y/len) + (tau_z/kappa)) / 4
    F3 = (F_total + (tau_x/len) - (tau_y/len) + (tau_z/kappa)) / 4
    F4 = (F_total - (tau_x/len) - (tau_y/len) - (tau_z/kappa)) / 4


### Flight Evaluation ###

#### 1. Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory ####

> Ensure that in each scenario the drone looks stable and performs the required task. Specifically check that the student's controller is able to handle the non-linearities of scenario 4 (all three drones in the scenario should be able to perform the required task with the same control gains used).

My controller passed all required scenarios. See the following captures for proof.

TODO: Insert each gif. 
TOOD: Explain what each gif is showing.