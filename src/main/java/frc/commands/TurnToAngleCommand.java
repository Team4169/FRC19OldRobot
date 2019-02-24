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

/**
 * Command to turn the robot to a specified field-relative
 * angle.  It uses the nav subsystem and a PID controller on
 * the drive train to turn the robot as closely as possible
 * to the specified angle on the field.  It also attempts to
 * detect any collision (with another robot or field element)
 * that may have happened during the turn, and abort if one
 * happens.
 */
public class TurnToAngleCommand extends Command {

  private DriveTrain m_driveTrain;
  private double m_desAngle;
  private CollisionDetector m_det;

  /**
   * Constructor give the needed subsystems.
   * @param driveTrain The robot's drive train
   * @param nav The navigation subsystem
   */
  public TurnToAngleCommand(DriveTrain driveTrain, Nav nav) {
    super(driveTrain);
    requires(nav);
    m_desAngle = 0.0d;
    m_det = new CollisionDetector(nav);
  }

  /**
   * Set the desired angle to turn to.  Should be called before
   * initialize (i.e. before anyone calls start on this instance)
   */
  public void setAngle(double angle) {
    m_desAngle = angle;
  }

  // Called just before this Command runs each time.
  // Must set up the drive train's PID controller for the
  // desired angle (set via setAngle(0) above).
  @Override
  protected void initialize() {
    m_det.reinitialize();
    m_driveTrain.startTurnToAngle(m_desAngle);
  }

  // Called repeatedly when this Command is scheduled to run.
  // Use the drive train to continue the turn, adjusting power
  // as needed to stop the turn at the desired angle.
  @Override
  protected void execute() {
    m_det.checkForCollision();
    m_driveTrain.turnToPIDAngle();
  }

  // Return true when this Command no longer needs to run execute()
  // We're done when we've reached the desired angle at low speed.
  @Override
  protected boolean isFinished() {
    return m_driveTrain.isTurnToAngleFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_driveTrain.stopAll();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
