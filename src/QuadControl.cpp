#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

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

  //JAS - used.
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

    TODO: solve the linear equation.    
  */

  float F1, F2, F3, F4;
  float F_total = collThrustCmd;
  float tau_x = momentCmd.x;
  float tau_y = momentCmd.y;
  float tau_z = momentCmd.z;

  //TODO: Why do we need to negate the z moment?
  tau_z = -tau_z;

  F1 = 0.25 * (F_total + (tau_x/len) + (tau_y/len) + (tau_z/kappa));
  F2 = 0.25 * (F_total - (tau_x/len) + (tau_y/len) - (tau_z/kappa));
  F3 = 0.25 * (F_total + (tau_x/len) - (tau_y/len) - (tau_z/kappa));
  F4 = 0.25 * (F_total - (tau_x/len) - (tau_y/len) + (tau_z/kappa));

  cmd.desiredThrustsN[0] = F1;
  cmd.desiredThrustsN[1] = F2;
  cmd.desiredThrustsN[2] = F3;
  cmd.desiredThrustsN[3] = F4;

  //TODO: remove when I figure out the math.
  // Previous, working code:
  // Moment (M) = Linear Force (F) * perpendicular distance from axis (L).
  // Solve for F.
  // F = M / L
  // float p_bar = momentCmd.x / len;    // x axis
  // float q_bar = momentCmd.y / len;    // y axis 
  // float r_bar = -momentCmd.z / kappa; // z axis
  // float c_bar = collThrustCmd;
  // cmd.desiredThrustsN[0] = (c_bar + p_bar + q_bar + r_bar) / 4.f; // Front Left
  // cmd.desiredThrustsN[1] = (c_bar - p_bar + q_bar - r_bar) / 4.f; // Front Right 
  // cmd.desiredThrustsN[2] = (c_bar + p_bar - r_bar - q_bar) / 4.f; // Rear left 
  // cmd.desiredThrustsN[3] = (c_bar - p_bar - q_bar + r_bar) / 4.f; // Rear Right
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

  // p, q, r are angular velocities in rad/s.
  // u_bar_p, u_bar_q, u_bar_r are accelerations in rad/s^2.
  // Moments of Inertia have units kg*m^2
  // momentCmd is roll moment, pitch moment, yaw moment in N*m.
    // Newtons are (kg*m)/s^2
  
  //TODO: Ask the question about the units.

  pqr_error = pqrCmd - pqr;
  u_bar_pqr = kpPQR * pqr_error;
  momentCmd = u_bar_pqr * moments_of_inertia;

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

  V3F pqrCmd = V3F(0.0, 0.0, 0.0);
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // So why do we need those rotation matrices?
  // Why do we need those cells specifically?
  // Is this where we go from the world frame to the body frame? Yes I think so.
    // We need to do this when we go from attitudes to body rates! And we are doing that here.
  // This is a different rotation matrix than the one first introduced right? To apply pitch/roll/yaw?
  // Do we have a max/min tilt? If so, maybe "CONSTRAIN".

        // R = euler2RM(attitude[0], attitude[1], attitude[2])
        // c_d = thrust_cmd/DRONE_MASS_KG
        
        // target_R13 = -np.clip(acceleration_cmd[0].item()/c_d, -self.max_tilt, self.max_tilt) #-min(max(acceleration_cmd[0].item()/c_d, -self.max_tilt), self.max_tilt)
        //   target_R23 = -np.clip(acceleration_cmd[1].item()/c_d, -self.max_tilt, self.max_tilt) #-min(max(acceleration_cmd[1].item()/c_d, -self.max_tilt), self.max_tilt)
          
        //   p_cmd = (1/R[2, 2]) *
        //           (-R[1, 0] * self.Kp_roll * (R[0, 2]-target_R13) +
        //             R[0, 0] * self.Kp_pitch * (R[1, 2]-target_R23))
        //   q_cmd = (1/R[2, 2]) *
        //           (-R[1, 1] * self.Kp_roll * (R[0, 2]-target_R13) +
        //             R[0, 1] * self.Kp_pitch * (R[1, 2]-target_R23))

  float target_R13, target_R23;
  float actual_R13, actual_R23;
  float error_R13, error_R23;
  float collAccel, x_frac_of_accel, y_frac_of_accel;
  float p_cmd, q_cmd;

  // All these terms are in the *world* frame.
  // P Controller.

  if (collThrustCmd > 0.0)
  {
    //TODO: Double check the sign for thrust is appropriate here.
    collAccel = -collThrustCmd / mass; // F=ma
    x_frac_of_accel = accelCmd.x / collAccel;
    y_frac_of_accel = accelCmd.y / collAccel;

    target_R13 = x_frac_of_accel;
    target_R23 = y_frac_of_accel;

    actual_R13 = R(0,2);
    actual_R23 = R(1,2);

    //TODO: The units, and the Rotation matrix math is still confusing to me.
    error_R13 = target_R13 - actual_R13;
    error_R23 = target_R23 - actual_R23;

    //TODO: figure out how the math was different here.
    // Convert to rates in the body frame.
    p_cmd = (R(1,0) * (kpBank * error_R13) - R(0,0) * (kpBank * error_R23)) / R(2,2);
    q_cmd = (R(1,1) * (kpBank * error_R13) - R(0,1) * (kpBank * error_R23)) / R(2,2);
      
    //p_cmd = (1/R(2,2)) * (-R(1,0) * kpBank * (error_R13) + R(0,0) * kpBank * (error_R23));
    //q_cmd = (1/R(2,2)) * (-R(1,1) * kpBank * (error_R13) + R(0,1) * kpBank * (error_R23));

    pqrCmd = V3F(p_cmd, q_cmd, 0.0);
  }
  else
  {
    // Thrust is zero or negative. Leave rates as zero.
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
  float pos_error_z, vel_error_z, rate, accel_z, c;


  pos_error_z = posZCmd - posZ;
  vel_error_z = velZCmd - velZ;
  accel_z = (kpPosZ * pos_error_z) + (kpVelZ * vel_error_z) + accelZCmd;

  //TODO: Get the sign right.
  //TODO: What is attitude needed for? Not in my diagram.
  // It looks like it is needed to calculate thrust, since you have to grab
  // the component that is vertical.
  //TODO: Convert acceleration to a force.
  c = (accel_z - CONST_GRAVITY) / R(2,2);

  // rate = CONSTRAIN(rate, -maxAscentRate, maxDescentRate);
  thrust_z = -(c * mass);

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
  float x_vel_cmd, y_vel_cmd;

  //TODO: This is a PD controller right?

  x_vel_cmd = kpVelXY * (velCmd.x - vel.x);
  y_vel_cmd = kpVelXY * (velCmd.y - vel.y);

  // Limit max horizontal velocity.
  x_vel_cmd = CONSTRAIN(x_vel_cmd, -maxSpeedXY, maxSpeedXY);
  y_vel_cmd = CONSTRAIN(y_vel_cmd, -maxSpeedXY, maxSpeedXY);

  accelCmd.x = kpPosXY * (posCmd.x - pos.x) + x_vel_cmd + accelCmdFF.x;
  accelCmd.y = kpPosXY * (posCmd.y - pos.y) + y_vel_cmd + accelCmdFF.y;

  // Limit max horizontal acceleration.
  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);

  //printf("accel x: %f, accel y: %f\n", accelCmd.x, accelCmd.y);
  //printf("pos x: %f, pos y: %f\n", pos.x, pos.y);
  //printf("des pos x: %f, des pos y: %f\n", posCmd.x, posCmd.y);
  //printf("vel x: %f, vel y: %f\n", vel.x, vel.y);
  //printf("des vel x: %f, des vel y: %f\n", velCmd.x, velCmd.y);
  //printf("------\n");

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
  yawRateCmd = kpYaw * (yawCmd - yaw);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

// JAS: Ok, so this is really the entry point for my controller.
// JAS: I get the simTime/dt, the next desired trajectory point (x, y, z, velocities)
// JAS: I output 4 individual motor thrust amounts. Which can be straightforwardly converted to propeller speeds.
/* INPUTS 
  - simTime, dt
  - curTrajPoint - desired position (x, y, z), desired velocity (x_dot, y_dot, z_dot), desired accel
  - estPos (x, y, z)
  - estVel 
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
