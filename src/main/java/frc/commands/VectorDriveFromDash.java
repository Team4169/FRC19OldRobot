/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystems.DriveTrain;
import frc.subsystems.Nav;
import frc.util.Vec2d;

/**
 * Drive according to the 2-d Vector specified on the Smart Dashboard, in
 * the "Drive Angle" and "Drive Dist" fields.  This is a test class for
 * testing the VectorDrive code (and its subcommands TurnToAngle and
 * DriveForDistance).
 */
public class VectorDriveFromDash extends Command {

  private VectorDrive m_vecDriveCmd;

  /**
   * Constructor given required subsystems
   * @param driveTrain The robot's drive train
   * @param nav The robot's navigation subsystem
   */
  public VectorDriveFromDash(DriveTrain driveTrain, Nav nav) {
    super(driveTrain);
    requires(nav);

    m_vecDriveCmd = new VectorDrive(driveTrain, nav);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Get the desired angle, distance, and velocity from the dashboard
    double dist = SmartDashboard.getNumber("Drive Dist", 24.0);
    double angle = SmartDashboard.getNumber("Drive Angle", 90.0);
    double vel = SmartDashboard.getNumber("Drive Vel", 24.0);
    Vec2d vec = Vec2d.makePolar(dist, Math.toRadians(angle));

    // Tell the vector driver what to do
    m_vecDriveCmd.setVector(vec, vel);

    // And, kick it off!
    m_vecDriveCmd.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Nothing to do here, we're done!
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
