/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.subsystems.Nav;
import frc.subsystems.Limelight;
import frc.util.RouteToTarget;
import frc.util.TargetCalculator;
import frc.util.Vec2d;
import frc.util.TargetVecMapper;

public class GetRouteToTarget extends Command {

  public static final double NORM_DIST = 12.0d;
  public static final double TIMEOUT = 5.0d;

  private final Nav m_nav;
  private final Limelight m_cam;
  private final TargetCalculator m_calc;
  private Vec2d m_roboVec;
  private Vec2d m_camVec;
  private Vec2d m_targNorm;
  private boolean m_seen;
  

  public GetRouteToTarget(Nav nav) {
    super(nav);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    m_nav = nav;
    m_cam = new Limelight();
    m_calc = new TargetCalculator(Limelight.HEIGHT, Limelight.ANGLE_FROM_HORIZONTAL);

    m_seen = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("GetRouteToTarget init");
    m_roboVec = m_nav.getRobotVec();
    m_camVec = m_cam.getCameraVector(m_roboVec);
    m_targNorm = TargetVecMapper.getStdTargNorm(m_nav.getYaw());
    m_seen = false;
    m_cam.setCameraMode(Limelight.CameraMode.eVision);
    m_cam.setLedMode(Limelight.LightMode.eOn);
    setTimeout(TIMEOUT);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!m_cam.isTarget()) {
      return;
    }

    m_seen = true;
    double tx = m_cam.getTx();
    double ty = m_cam.getTy();

    System.out.println("Target at [" + tx + "," + ty + "]");

    RouteToTarget rte = m_calc.getRouteToTarget(tx, ty, m_roboVec, m_camVec,
        m_targNorm, Limelight.targetHeight, NORM_DIST);
    
    System.out.println("Intercept: " + rte.getInterceptVec());
    System.out.println("Norm:" + rte.getNormalVec());
    System.out.println("Target:" + rte.getTargetDirectVec());


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (isTimedOut()) {
      System.out.println("GetRouteToTarget: timed out with no target seen");
    }
    return m_seen || isTimedOut(); 
   }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("GetRouteToTarget turning leds off");
    m_cam.setCameraMode(Limelight.CameraMode.eDriver);
    m_cam.setLedMode(Limelight.LightMode.eOff);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
