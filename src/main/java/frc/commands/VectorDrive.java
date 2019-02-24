/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.subsystems.DriveTrain;
import frc.subsystems.Nav;
import frc.util.Vec2d;

/**
 * Command to drive the robot according to the specified vector.
 * Turn to the vector's angle, and drive for the
 * vector's specified distance at the specified 
 * velocity (units per second).
 */
public class VectorDrive extends CommandGroup {

  private TurnToAngleCommand m_turnCmd;
  private DriveStraightForDistance m_driveCmd;

  /**
   * Constructor given the needed subsystems.
   * @param driveTrain The robot's drive train
   * @param nav The robot's navigation subsystem
   */
  public VectorDrive(DriveTrain driveTrain, Nav nav) {
    m_turnCmd = new TurnToAngleCommand(driveTrain, nav);
    m_driveCmd = new DriveStraightForDistance(driveTrain, nav);
    addSequential(m_turnCmd);
    addSequential(m_driveCmd);
  }

  /**
   * Set the vector and velocity for the turn to angle and drive straight
   * commands.  Must be called before this command is started!
   * @param vec The vector to drive to
   * @double vel The velocity to drive at
   */
  public void setVector(Vec2d vec, double vel) {
    m_turnCmd.setAngle(Math.toDegrees(vec.getTheta()));
    m_driveCmd.setDistAndVel(vec.getR(), vel);
  }
}
