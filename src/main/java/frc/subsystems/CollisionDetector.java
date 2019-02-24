/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.subsystems.Nav;

/**
 * Based on code from Kauai Labs
 * Detect a collision (with a wall or another robot) and abort
 * all running commands if one happens.
 * Collision detection is done by detecting a high "jerk" (rate
 * of change of acceleration) in the X or Y directions.
 * We directly abort all running commands by calling the
 * scheduler ourselves to minimize delay.
 */

public class CollisionDetector {

  // Collision threshold (max jerk); adjust if needed
  final static double kCollisionThreshold_DeltaG = 0.5f;

  private Nav m_nav;
  private double m_lastWorldXAccel;
  private double m_lastWorldYAccel;

  /**
   * Construct a new collision detector, using the specified
   * nav subsystem.
   * @param Nav The nav subsystem (including accelerometer)
   */
  public CollisionDetector(Nav nav) {
    m_nav = nav;
    m_lastWorldXAccel = m_nav.getWorldLinearAccelX();
    m_lastWorldYAccel = m_nav.getWorldLinearAccelY();
    SmartDashboard.putString("Collision Detected", "None");
  }

  /**
   * Reinitialize the collision detector 
   */
  public void reinitialize() {
    m_lastWorldXAccel = m_nav.getWorldLinearAccelX();
    m_lastWorldYAccel = m_nav.getWorldLinearAccelY();
    SmartDashboard.putString("Collision Detected", "None");
  }

  /**
   * Check for a collision; on a collision, abort all currently
   * scheduled Commands.
   */
  public void checkForCollision() {
    
    double curr_world_linear_accel_x = m_nav.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - m_lastWorldXAccel;
    m_lastWorldXAccel = curr_world_linear_accel_x;
    double curr_world_linear_accel_y = m_nav.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - m_lastWorldYAccel;
    m_lastWorldYAccel = curr_world_linear_accel_y;
    
    if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
          ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
      
      // Collision!  Display on smart dashboard and abort!
      SmartDashboard.putString("Collision Detected", "Collision detected, aborting!");
      Scheduler.getInstance().removeAll();
    }
  }

}
