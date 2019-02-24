/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

/**
 * Not really a subsystem.
 * A "RobotModel" class, that provides a "kinematic" (i.e. motion)
 * model of the robot.  Encapsulates all knowledge of
 * the specifics of the robot -- what kind of motors it's using, its gear
 * ratio and wheel sizes, etc. --  isolated in one place and easy
 * to change without changing any of the DriveTrain code
 */

 public class RobotModel {
  // Well-known constants
  // Max motor voltage (= 1.0 motor power)
  public static final double maxVoltage = 12.0d;          // volts
  public static final double maxMotorSpeed = (5300.0d/60d); // rev / sec, for CIM motor
  public static final double wheelDiam = 6.0d;             // in, for kitbot chassis
  public static final double gearReduction = 10.7d;        // for kitbot chassis
  public static final double secPerStep = 0.02d;          // time per "step" (sched run)
  public static final double distancePerRevolution = Math.PI*wheelDiam; // distance per revolution
  public static final double maxVelocity = (distancePerRevolution*maxMotorSpeed) / gearReduction;

  // Compute kV; can replace with empirical
  // value if needed
  public static final double kV = maxVoltage / maxVelocity;

  // Can replace startVoltage with empirical
  // value if needed
  public static final double startVoltage = 1.25d;         // volts

  // Time to accelerate from minimum velocity (startVoltage) to maximum
  // velocity (maxVoltage).  This can be tweaked some, but if too small
  // it won't be practical for the robot.  In practice 2 sec seems right.
  public static final double accelTime = 2.0d;             // sec

  // Number of steps required to accelerate from 0 to max velocity
  public static final double accelSteps = accelTime / secPerStep;


  // Utility routines for units conversion etc.
  // These should have reasonableness checks
  // eg. 0<=power<=1.0; 0<=voltage<=12.0 etc.

  public static double powerToVoltage(double power) {
    if ((power < 0.0) || (power > 1.0)) {
      throw new IllegalArgumentException("Power out of range " + power);
    }
    return maxVoltage * power;
  }

  public static double voltageToPower(double voltage) {
    if ((voltage < 0.0) || (voltage > maxVoltage)) {
      throw new IllegalArgumentException("Voltage out of range " + voltage);
    }
    return voltage / maxVoltage;
  }

  // Note: this only applies for an absolute voltage
  // To compute velocity delta for a voltage delta,
  // it's just voltage / kV
  public static double voltageToVelocity(double voltage) {
    if ((voltage < 0.0) || (voltage > maxVoltage)) {
      throw new IllegalArgumentException("Voltage out of range " + voltage);
    }
    if (voltage < startVoltage) {
      return 0.0d;
    } else {
      return (voltage - startVoltage) / kV;
    }
  }

  // Note: this only applies for an absolute velocity
  // To compute a voltage delta for a velocity delta,
  // it's just kV * velocity
  public static double velocityToVoltage(double velocity) {
    return (kV * velocity) + startVoltage;
  }

  public static double powerToVelocity(double power) {
    return voltageToVelocity(powerToVoltage(power));
  }

  public static double velocityToPower(double velocity){
      return voltageToPower(velocityToVoltage(velocity));
  }

  // Compute velocity per step -- approximation to acceleration
  // We want to accelerate from the startVoltage to the maxVoltage
  // in accelTime seconds
  // Velocity increase (or decrease) for a single step while
  // accelerating (or decelerating).  
  public static final double velocityPerStep = voltageToVelocity(maxVoltage) / accelSteps;

  // Power increase per step -- equal to velocity per step
  // converted to power (Note this is a delta, not absolute)
  public static final double powerPerStep = voltageToPower(kV * velocityPerStep);

  // Motor power required to start the robot i.e. overcome friction
  public static final double startPower = voltageToPower(startVoltage);

  // Calculate the distance travelled while accelerating stepwise
  // It is SUM[i=0 to n](i * delta-v * delta-t)
  // But as long as delta-v and delta-t are constants they can be
  // pulled outside the summation; SUM[i=0 to n](i) = (n)(n-1)/2
  public static double calculateAccelDistance(int n, double deltaV, double deltaT) {
      return (deltaV * deltaT * (double)n * ((double)n -1.0d)) / 2.0d;
  }

  // Calculate the number of acceleration steps to reach
  // (approximately) the specified distance.  This is the
  // rough inverse of calculateAccelDistance() above and
  // requires finding the positive root of the quadratic.
  public static double calculateAccelSteps(double dist, double deltaV, double deltaT) {
      double c = -(2.0d * dist) / (deltaV * deltaT);
      double root = (1.0d + Math.sqrt(1.0d - (4.0d*c))) / 2.0d;
      return Math.floor(root);
  }
 }
