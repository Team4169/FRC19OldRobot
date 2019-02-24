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
import frc.commands.VectorDrive;

/**
 * Command group to successively drive to one vector,
 * then a second.  Intended to be used as part of
 * (e.g.) driving to a target.  The vectors to be
 * driven to must be set by calling setVector() before
 * start() is called on the group.
 */
public class TwoVectorDrive extends CommandGroup {

  private DriveTrain m_driveTrain;
  private Nav m_nav;
  private VectorDrive m_intVecDrive;
  private VectorDrive m_normVecDrive;

  /**
   * Constructor given the required subsystems
   * @param driveTrain The robot's drive train
   * @param nav The navigation subsystem
   */
  public TwoVectorDrive(DriveTrain driveTrain, Nav nav) {
    requires(driveTrain);
    requires(nav);

    m_driveTrain = driveTrain;
    m_nav = nav;
    m_intVecDrive = new VectorDrive(m_driveTrain, m_nav);
    m_normVecDrive = new VectorDrive(m_driveTrain, m_nav);
    addSequential(m_intVecDrive);
    addSequential(m_normVecDrive);
  }

  /**
   * Should be called before start()!
   * Set up the vectors to be driven to in succession -- first
   * the "intercept vector", then the "normal vector".
   */
  public void setVectors(Vec2d vec1, double vel1, Vec2d vec2, double vel2) {
    m_intVecDrive.setVector(vec1, vel1);
    m_normVecDrive.setVector(vec2, vel2);
  }
}
