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
import frc.subsystems.Limelight;
import frc.util.RouteToTarget;
import frc.util.TargetCalculator;
import frc.util.Vec2d;
import frc.util.TargetVecMapper;

/**
 * Command to determine a route to the target currently in
 * view by the Limelight camera, then drive the robot to
 * that target (if any).  Four steps:
 *  - Use the Limelight's vision pipeline to find the target
 *  - Use the target calculator to determine the route to the
 *    target (intercept vector and normal vector)
 *  - Drive the intercept vector to position the robot directly
 *    in front of the target and a known distance away
 *  - Drive the normal vector to position the robot perpendicular
 *    to and at the target
 * Can time out if no target is visible.  Can abort if (e.g.) the
 * robot collides with another robot or other obstacle during the
 * trip.
 */

public class DriveRouteToTarget extends Command {

  public static final double NORM_DIST = 12.0d;
  public static final double TIMEOUT = 5.0d;
  public static final double INTERCEPT_POWER = 0.4d;
  public static final double NORMAL_POWER = 0.2d;

  private final DriveTrain m_driveTrain;
  private final Nav m_nav;
  private final Limelight m_cam;
  private final TargetCalculator m_calc;
  private final TwoVectorDrive m_driveCmd;
  private Vec2d m_roboVec;
  private Vec2d m_camVec;
  private Vec2d m_targNorm;
  private boolean m_seen;
  
  /**
   * Constructor given the drive train and nav unit.
   * @param driveTrain The drive train used to drive the robot
   * @param nav The navigation unit used to provide feedback
   * control to drive the robot straight and turn to angle
   */
  public DriveRouteToTarget(DriveTrain driveTrain, Nav nav, Limelight cam) {
    super(nav);
    m_driveTrain = driveTrain;
    m_nav = nav;
    m_cam = cam;
    m_calc = new TargetCalculator(Limelight.HEIGHT, Limelight.ANGLE_FROM_HORIZONTAL);

    // Create the CommandGroup that we will use to do the actual driving,
    // once we've found the target
    m_driveCmd = new TwoVectorDrive(m_driveTrain, m_nav);

    m_seen = false;
  }

  // Called just before this Command runs each time (typically as
  // a result of a button press).  Must put the camera into vision
  // mode and get the initialization vectors needed by the target
  // calculator at this time.
  @Override
  protected void initialize() {
    System.out.println("DriveRouteToTarget init");
    m_roboVec = m_nav.getRobotVec();
    m_camVec = m_cam.getCameraVector(m_roboVec);
    m_targNorm = TargetVecMapper.getStdTargNorm(m_nav.getYaw());
    m_seen = false;
    m_cam.visionMode();

    // If we don't find a target in the specified timeout, give up
    setTimeout(TIMEOUT);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    // It can take a while for the camera to 'lock onto' the target.
    // If no target lock yet, wait.  
    if (!m_cam.isTarget()) {
      return;
    }

    // Got a target!  Calculate the route to it
    m_seen = true;
    double tx = m_cam.getTx();
    double ty = m_cam.getTy();

    System.out.println("Target at [" + tx + "," + ty + "]");

    RouteToTarget rte = m_calc.getRouteToTarget(tx, ty, m_roboVec, m_camVec,
        m_targNorm, Limelight.targetHeight, NORM_DIST);
    
    System.out.println("Intercept: " + rte.getInterceptVec());
    System.out.println("Norm:" + rte.getNormalVec());
    System.out.println("Target:" + rte.getTargetDirectVec());

    // Set up and run the actual drive commands.  This will set
    // the angles and distances for the intercept and normal
    // vectors.  Note that we drive the normal vector at a different
    // (lower) power than the intercept vector, since it's the 'final
    // approach' to the target
    m_driveCmd.setVectors(rte.getInterceptVec(), INTERCEPT_POWER,
                          rte.getNormalVec(), NORMAL_POWER);
    m_driveCmd.start();
  }

  // Return true when this Command no longer needs to run execute() --
  // either we've seen the target, or timed out trying to find one
  @Override
  protected boolean isFinished() {
    if (isTimedOut()) {
      System.out.println("GetRouteToTarget: timed out with no target seen");
    }
    return m_seen || isTimedOut(); 
   }

  // Called once after isFinished returns true.  Put the camera back in
  // driver mode.
  @Override
  protected void end() {
    System.out.println("DriveRouteToTarget turning leds off");
    m_cam.driverMode();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
