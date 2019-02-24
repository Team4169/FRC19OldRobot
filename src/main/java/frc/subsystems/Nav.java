/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDSource;
import frc.util.Vec2d;

/**
 * Navigation subsystem
 */


public class Nav extends Subsystem {

  private ADXRS450_Gyro m_gyro;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Nav(ADXRS450_Gyro gyro) {
    m_gyro = gyro;
    m_gyro.calibrate();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * Convert robot yaw from the compass (in degrees, 0 meaning pointing along the Y axis
   * field-relative) to a field-relative angle in degrees (0 pointing along the X axis).
   * Result will be in the range -180..180.
   * @param yaw Yaw in degrees
   * @return Field angle in degrees
   */
  public double yawToFieldAngle(double yaw) {

    double fieldAngle = 90.0 - yaw;
    if (fieldAngle > 180.0d) {
        fieldAngle = fieldAngle - 360.0;
    } else if (fieldAngle < -180.0d) { // shouldn't happen
        fieldAngle = fieldAngle + 360.d;
    }
    return fieldAngle;
  }

  /**
   * Convert robot yaw from the compass (in degrees, 0 meaining pointing along the Y axis
   * field-relative) to a field-relative normal vector.
   * @param yaw Yaw in degrees
   * @return Robot unit vector
   */
  private Vec2d yawToVec(double yaw) {
  
    double fieldAngle = yawToFieldAngle(yaw);
    return Vec2d.makePolar(1.0, Math.toRadians(fieldAngle));
  }

  public double getYaw() {
    return m_gyro.getAngle();
  }

  public double getWorldLinearAccelX() {
    return 0.0;       // Can't do with this gyro!
  }

  public double getWorldLinearAccelY() {
    return 0.0;        // Can't do with this gyro!
  }

  public Vec2d getRobotVec() {
    return yawToVec(getYaw());
  }

  public Vec2d getTargVec(Vec2d roboVec) {
    return Vec2d.makeCart(0.0d, -1.0d);
  }

  public PIDSource getPIDSource() {
    return m_gyro;
  }

}
