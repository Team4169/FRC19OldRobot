/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.subsystems.DriveTrain;
import frc.subsystems.Nav;
import frc.subsystems.CollisionDetector;
import frc.subsystems.RobotModel;

/**
 * Command to drive the robot at a specified max velocity (units
 * per second) as straight as possible for as close to a specified
 * distance as possible.  This command uses an approximation to
 * a trapezoidal motion profile -- constant acceleration to the
 * specified velocity, constant velocity, then constant deceleration
 * back to zero -- to achieve the desired distance.  It uses the
 * RobotModel (which models approximately the behavior of the robot
 * drive train in response to power inputs) to determine how quickly
 * the robot can accelerate and delecerate; and it uses the nav
 * subsystem and encoders on the drive train (together with a PID
 * controller) to drive the robot straight and control the actual drive
 * distance.  It also attempts to detect any collisions (with other robots
 * or field elements) that may occur during the drive, and to abort the
 * drive if a collision occurs.
 */
public class DriveStraightForDistance extends Command {

  private DriveTrain driveTrain;
  private CollisionDetector colDet;
  private double distance;
  private double velocity;
  private boolean finished;
  private int accelSteps;
  private int runSteps, nSteps, nAccelSteps, nRunSteps;

  private boolean triangularAccel;

  enum RunState {
    eAccel, eRun, eDecel, eDone
  }

  private RunState runState;

  /**
   * Constructor given the subsystems we depend on.
   * @param dt The drive train
   * @param nav The navigation subsystem
   */
  public DriveStraightForDistance(DriveTrain dt, Nav nav) {
    super(dt);
    requires(nav);
    driveTrain = dt;
    colDet = new CollisionDetector(nav);
    this.distance = 0.0d;
    this.velocity = 0.0d;
    finished = false;
    runState = RunState.eDone;
  }

  /**
   * Set the desired distance and velocity.  Must be called
   * before this command is started!
   * @param dist The distance to run
   * @param vel The velocity to run at (units per sec)
   */
  public void setDistAndVel(double dist, double vel) {
    distance = dist;
    velocity = vel;
  }

  // Called just before this Command runs each time, as a
  // result of someone calling start() on the command (e.g.
  // from a button, or an enclosing command like VectorDrive).
  // Must calculate the acceleration and run distances and 
  // steps we're taking this time, based on the distance and
  // velocity given us in setDistAndVel(), and set up the
  // PID controller in the drive train for straight driving.
  @Override
  protected void initialize() {
    finished = ((velocity <= 0) || (velocity > RobotModel.maxVelocity));

    // assume trapezoid
    accelSteps = (int) Math.ceil(velocity / RobotModel.velocityPerStep);

    double accelDistance = RobotModel.calculateAccelDistance(accelSteps, RobotModel.velocityPerStep, RobotModel.secPerStep);

    if ((2.0d * accelDistance) < distance) {
      triangularAccel = false;
    } else {

      // This is the "triangular" (vs trapezoidal) case, where we don't
      // have room to accelerate all the way to the full velocity.  In
      // this case we're going to accelerate at a constant rate for half
      // the distance, and decelerate at the same rate for the rest.
      triangularAccel = true;
      
      accelDistance = Math.floor(distance / 2.0d);
      accelSteps = (int) RobotModel.calculateAccelSteps(accelDistance, RobotModel.velocityPerStep, RobotModel.secPerStep);
      accelDistance = RobotModel.calculateAccelDistance(accelSteps, RobotModel.velocityPerStep, RobotModel.secPerStep);
    }

    double runVelocity = accelSteps * RobotModel.velocityPerStep;
    double runDistance = distance - (2 * accelDistance);
    double runTime = runDistance / runVelocity;
    runSteps = (int) Math.floor(runTime / RobotModel.secPerStep);
    System.out.println("Triangular acceleration: " + Boolean.toString(triangularAccel));
    System.out.println("rdist " + runDistance + " rtime " + runTime + " rsteps " + runSteps);
    System.out.println("accSteps " + accelSteps + " accDist " + accelDistance);
    nSteps = 0;
    nAccelSteps = 0;
    nRunSteps = 0;
    runState = RunState.eAccel;
    if (finished) {
      System.out.println("Drive straight for distance failed - velocity not legitimate");
      end();
    } else {
      colDet.reinitialize();
      driveTrain.zeroEncoders();
      driveTrain.startDriveStraight();
    }
  }

  // Called repeatedly when this Command is scheduled to run.
  // We're either accelerating, running at constant power, or
  // decelerating (which includes the final "creep" to the target).
  @Override
  protected void execute() {
    colDet.checkForCollision();
    double motorPower;
    if (nSteps == 0) {
      motorPower = RobotModel.startPower;
    } else {
      motorPower = driveTrain.getCurrentPower();
    }

    nSteps++;
    if (runState == RunState.eAccel) {
      nAccelSteps++;
      motorPower += RobotModel.powerPerStep;
      if (motorPower > 1.0) {
        motorPower = 1.0;
      }
      if (nAccelSteps >= accelSteps) {
        runState = RunState.eRun;
      }
    } else if (runState == RunState.eRun) {
      nRunSteps++;
      if (nRunSteps >= runSteps) {
        runState = RunState.eDecel;
      }
    } else if (runState == RunState.eDecel) {
      if (motorPower > (RobotModel.startPower + RobotModel.powerPerStep)) {
        motorPower -= RobotModel.powerPerStep;
      }
    } else if (runState == RunState.eDone) {
      motorPower = 0.0;
    }

    double dist = driveTrain.getCurrentDistance();
    driveTrain.driveStraight(motorPower);

    System.out.println("exec state " + runState.toString() + " setting power to " + motorPower + " at dist " + dist);
  }

  

  // Make this return true when this Command no longer needs to run execute()
  // We're done if an error occurred during execute() or if we've reached
  // the desired distance!
  @Override
  protected boolean isFinished() {
    return finished || (driveTrain.getCurrentDistance() >= distance);
  }

  // Called once after isFinished returns true
  // Turn off the motors and the drive train's PID controller.
  @Override
  protected void end() {
    System.out.println("Ending drive straight for distance at dist " + driveTrain.getCurrentDistance());
    driveTrain.endDriveStraight();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
