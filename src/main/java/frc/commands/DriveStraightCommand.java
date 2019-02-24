/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.subsystems.DriveTrain;

public class DriveStraightCommand extends Command {

  public double MOTOR_POWER = 0.3d;

  private DriveTrain m_driveTrain;

  public DriveStraightCommand(double timeout, DriveTrain driveTrain) {
      super(timeout, driveTrain);
      m_driveTrain = driveTrain;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_driveTrain.startDriveStraight();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_driveTrain.driveStraight(MOTOR_POWER);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    m_driveTrain.endDriveStraight();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

}
