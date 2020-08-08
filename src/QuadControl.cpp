#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#include "Angles.h" // for AngleNormF()

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // Loaded by BaseController::Init(). Value set in QuadControlParams.txt.
  // "L" - arm length parameter. 
  // "kappa" - ratio between thrust [N] and torque due to drag [N m]

  /*
    "kappa" is derived from the two core equations we have been using for
    force and torque of the propellors:
      F = kf * omega^2
      tau = km * omega^2
    
    kappa = F/tau = kf/km

    Also, tau = kappa * F
    
    This ratio becomes useful when we want to solve our set of linear equations
    for the F1/F2/F3/F4.

    We have the following equation, assuming propellors 1 and 3 are rotating clockwise:
      tau_z = -tau_1 + tau_2 + -tau_3 + tau_4
    Where tau_1 is the reactive moment around the z axis caused by propellor one rotating.

    We can replace those tau_1/tau_2/tau_3/tau_4 using kappa!
      tau_z = -(kappa*F1) + (kappa*F2) + -(kappa*F3) + (kappa*F4)
      tau_z = (-F1 + F2 + -F3 + F4) * kappa
  */

  /* 
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
  */
  float len = L / (sqrtf(2.f));

  /* 
    Units
    Keep in mind this function is going from thrust (N) to thrust (N).
    The only thing we are doing is assigning the amount of thrust to each
    propellor to achieve the desired 3-axis moment.
  */

  /*
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
    However, using kappa, we do have an alternative. See the derivation in
    the earlier comment block.
      tau_z = (-F1 + F2 + -F3 + F4) * kappa

    Solving the following set of linear equations:
      F_total = ( F1 +  F2 +  F3 +  F4)
      tau_x   = ( F1 + -F2 +  F3 + -F4) * len
      tau_y   = ( F1 +  F2 + -F3 + -F4) * len
      tau_z   = (-F1 +  F2 + -F3 +  F4) * kappa

    TODO: Now apply the unexplained negation to tau_z.
    
    Or:
      F_total     =  F1 +  F2 +  F3 +  F4
      tau_x/len   =  F1 + -F2 +  F3 + -F4
      tau_y/len   =  F1 +  F2 + -F3 + -F4
      tau_z/kappa = -F1 +  F2 +  F3 + -F4

    Solving the linear equations gives the following formulas:
      F1 = (F_total + (tau_x/len) + (tau_y/len) - (tau_z/kappa)) / 4
      F2 = (F_total - (tau_x/len) + (tau_y/len) + (tau_z/kappa)) / 4
      F3 = (F_total + (tau_x/len) - (tau_y/len) + (tau_z/kappa)) / 4
      F4 = (F_total - (tau_x/len) - (tau_y/len) - (tau_z/kappa)) / 4
  */

  float F1, F2, F3, F4;
  float F_total = collThrustCmd;
  float tau_x = momentCmd.x;
  float tau_y = momentCmd.y;
  float tau_z = momentCmd.z;

  F1 = 0.25 * (F_total + (tau_x/len) + (tau_y/len) - (tau_z/kappa));
  F2 = 0.25 * (F_total - (tau_x/len) + (tau_y/len) + (tau_z/kappa));
  F3 = 0.25 * (F_total + (tau_x/len) - (tau_y/len) + (tau_z/kappa));
  F4 = 0.25 * (F_total - (tau_x/len) - (tau_y/len) - (tau_z/kappa));

  cmd.desiredThrustsN[0] = F1;
  cmd.desiredThrustsN[1] = F2;
  cmd.desiredThrustsN[2] = F3;
  cmd.desiredThrustsN[3] = F4;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   Returns: V3F containing the desired roll moment, pitch moment, and
  //            yaw moment commands in Newtons*meters

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  V3F pqr_error;
  V3F u_bar_pqr;
  V3F moments_of_inertia = V3F(Ixx, Iyy, Izz);
  V3F momentCmd; // Output

  /* Units
    Inputs:
      p, q, r (both commanded and actual) are angular velocities in rad/s in the body frame.
      Ixx, Iyy, Izz are Moments of Inertia in kg*m^2.
    Intermediates:
      u_bar_p, u_bar_q, u_bar_r are accelerations in rad/s^2.
    Outputs:
      momentCmd is roll moment, pitch moment, yaw moment in N*m.
         Newtons are (kg*m)/s^2
  */

  /* Math
    The BodyRate controller is a simple "P" controller for the body rates (p, q, r).

    The only quirk is that the controller needs to convert the commanded body 
    accelerations to commanded moments for the GenerateMotorCommands() function.

    To do this conversion, we use the formula given at the very beginning of the
    controls lectures. Lesson 1.2.
      Moment about a given axis = (Moment of Inertia for that axis) * (Acceleration about that axis)
      M_x = I_xx * u_bar_p
  */
  pqr_error = pqrCmd - pqr;
  u_bar_pqr = kpPQR * pqr_error;
  momentCmd = moments_of_inertia * u_bar_pqr;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  /* Overview
  The Roll-Pitch Controller is a simple "P" controller.

  A few things make it kind of complicated though:
  - The inputs and outputs aren't the same. The inputs are total thrust, current attitude,
    desired acceleration in X and Y directions. But the outputs are angular velocities.
  - The inputs are in the world frame, but the outputs are in the body frame.
  */

  float target_R13, target_R23;
  float actual_R13, actual_R23;
  float error_R13, error_R23;
  float collAccel;
  float p_cmd, q_cmd;

  if (collThrustCmd <= 0.0)
  {
    // Thrust is zero or negative. Set rates as zero.
    printf("Warning: Negative thrust command.\n");
    pqrCmd = V3F(0.0, 0.0, 0.0);
  }
  else
  {
    /* 
      Get the collective acceleration. 

      This can be done with a=F/m, which is derived from F=ma.

      The thrust must be negated because of the NED frame. It is supplied
      from the Altitude Controller as a magnitude without a sign.
    */
    collAccel = -collThrustCmd / mass;

    /*
      Get the target R13 and R23 values.

      As the lecture describes, R13 and R23 are our "control knobs" for 
      controlling the X and Y positions.

      To do this, we use the formula presented in 4.17 and the 
      Feed Forward Parameter Identification paper:
      (https://www.dynsyslab.org/wp-content/papercite-data/pdf/schoellig-acc12.pdf).

      Note: Per the paper: R13, R23, R33 "represent the direction of the collective thrust in the inertial frame O"

      x_dot_dot = total_thrust * R13
      y_dot_dot = total_thrust * R23

      Rearranged:

      target_R13 = x_dot_dot / total_thrust
      target_R23 = y_dot_dot / total_thrust
    */
    //TODO: but why do we use acceleration then!!! See my mentor question.
    target_R13 = accelCmd.x / collAccel;
    target_R23 = accelCmd.y / collAccel;

    // Limit the target R13/R23 values.
    // Note: This limit is critical, otherwise the tilt will be commanded to such 
    //       a large value that the drone will flip over.
    target_R13 = CONSTRAIN(target_R13, -maxTiltAngle, maxTiltAngle);
    target_R23 = CONSTRAIN(target_R23, -maxTiltAngle, maxTiltAngle);


    // Name the matrix elements for clarity.
    actual_R13 = R(0,2);
    actual_R23 = R(1,2);

    // Calculate the error.
    error_R13 = target_R13 - actual_R13;
    error_R23 = target_R23 - actual_R23;

    /* 
      Get the rates in the body frame.

      To do this, we use the formula presented in 4.17 and the 
      Feed Forward Parameter Identification paper, formula #6:
      (https://www.dynsyslab.org/wp-content/papercite-data/pdf/schoellig-acc12.pdf).
    */
    p_cmd = (R(1,0) * (kpBank * error_R13) - R(0,0) * (kpBank * error_R23)) / R(2,2);
    q_cmd = (R(1,1) * (kpBank * error_R13) - R(0,1) * (kpBank * error_R23)) / R(2,2);

    pqrCmd = V3F(p_cmd, q_cmd, 0.0);
  }
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust_z = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  /* Overview
    The Altitude Controller is a "PID" controller.

    The only complication is that this function must return a z force, not an z acceleration.

    Our inputs are in NED. The function should actually output a thrust, not in the NED frame.
  */
  float pos_error_z, vel_error_z, rate, accel_z, c;

  // Limit the commanded velocity.
  //TODO: keep an eye on this, it is a different place to limit rate than other people.
  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);

  // Calculate the commanded z acceleration using standard PID controller math.
  pos_error_z = posZCmd - posZ;
  vel_error_z = velZCmd - velZ;
  integratedAltitudeError += (pos_error_z * dt);
  accel_z = (kpPosZ * pos_error_z) + (kpVelZ * vel_error_z) + (KiPosZ * integratedAltitudeError) + accelZCmd;

//TODO: convert all my "c" = collective thrusts to collective accelerations if I am right.
  /* 
    Convert the acceleration (an acceleration) to thrust (a force).

    Using the equation from 4.17:

    z_dot_dot = c * b_z + g

    or 

    z_dot_dot = collective_thrust * R33 + g

    Solve for collective_thrust.

    collective_thrust = (z_dot_dot - g) / R33

    Note: So the rotation matrix is needed for R33, which helps us figure out
    how much of the collective thrust contributes to the z direction.
  */
  c = (accel_z - CONST_GRAVITY) / R(2,2);

  //TODO: why am I doing this? I thought I already had a force! This is needed to work!
  // All my calculations would work out if "c" was actually collective acceleration!!!!
  thrust_z = -(c * mass);

  // From python:
  // thrust = DRONE_MASS_KG * acceleration_cmd / R33

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust_z;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  /* Overview
    The Lateral Position Controller is a "PD" controller.
  */
  float x_vel_cmd, y_vel_cmd;

  x_vel_cmd = kpVelXY * (velCmd.x - vel.x);
  y_vel_cmd = kpVelXY * (velCmd.y - vel.y);

  // Limit max horizontal velocity.
  // x_vel_cmd = CONSTRAIN(x_vel_cmd, -maxSpeedXY, maxSpeedXY);
  // y_vel_cmd = CONSTRAIN(y_vel_cmd, -maxSpeedXY, maxSpeedXY);

  accelCmd.x = kpPosXY * (posCmd.x - pos.x) + x_vel_cmd + accelCmdFF.x;
  accelCmd.y = kpPosXY * (posCmd.y - pos.y) + y_vel_cmd + accelCmdFF.y;

  // Limit max horizontal acceleration.
  // accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  // accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);

  //TODO: HOLY MOLY - why was this such a problem! Completely changes the drone behavior!

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  /* Math
    The Yaw Controller is a simple "P" controller.

    The only quirk is that you must wrap the yaw values to normalize them
    as 0 to 2PI, or -PI to PI.
  */
  float yaw_error;

  // Ensure the commanded yaw is within 0 to PI.
  yawCmd = fmodf(yawCmd, 2*M_PI);

  // Ensure the yaw error is within -PI to PI.
  // Note: The range is -PI to PI since the subtraction can make the value negative.
  yaw_error = yawCmd - yaw;
  yaw_error = AngleNormF(yaw_error);

  // Calculate the command.
  yawRateCmd = kpYaw * yaw_error;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

/* INPUTS 
  - simTime, dt
  - curTrajPoint - desired position (x, y, z), desired velocity (x_dot, y_dot, z_dot), desired accel
  - estPos (x, y, z)
  - estVel 
  - estAtt
*/
VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
