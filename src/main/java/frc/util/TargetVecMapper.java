/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import frc.util.Vec2d;

/**
 * Utility class for mapping a robot vector to the target normal vector
 * for the target the robot 'must' be pointing at -- assuming it's pointing
 * at a target.  In the 2019 game there are different mappings for the
 * 'orthogonal' targets -- those on the cargo ship and the loading station --
 * and the 'rocket' targets, which are offset at about 61 deg.
 */
public class TargetVecMapper {

  // These are the orthogonal standard target normal vectors
  static final Vec2d[] stdTargets = {
    Vec2d.makeCart(1.0d, 0.0d),
    Vec2d.makeCart(0.0d, 1.0d),
    Vec2d.makeCart(-1.0d, 0.0d),
    Vec2d.makeCart(0.0d, -1.0d) 
  };

  // This is the mapping from robot vector angle (in degrees) to
  // standard robot target vector
  static final double[][] stdTargetMapping = {
    {-45.0, 45.0, 3.0},
    {45.0, 135.0, 2.0},
    {135.0, 180.0, 1.0},
    {-180.0, -135.0, 1.0},
    {-135.0, -45.0, 0.0}
  };

  /**
   * Find the target normal for the standard target that's in view from the given yaw
   * angle.  Only one such target should be in view.
   */
  public static Vec2d getStdTargNorm(double yaw){
 
    for (double[] mpg: stdTargetMapping) {
      if ((yaw >= mpg[0]) && (yaw <= mpg[1])) {
        return stdTargets[(int)mpg[2]];
      }
    }
    throw new RuntimeException("Could not find standard target for yaw " + yaw);
  }

}
