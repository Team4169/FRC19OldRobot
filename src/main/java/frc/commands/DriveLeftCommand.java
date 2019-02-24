/*
 * The MIT License
 *
 * Copyright 2019 lwa.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package frc.commands;

import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.subsystems.DriveTrain;

public class DriveLeftCommand extends Command {

  private DriveTrain m_driveTrain;

  public DriveLeftCommand(double timeout, DriveTrain driveTrain) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super(timeout, driveTrain);
    m_driveTrain = driveTrain;
    System.out.println("DriveLeft const timeToRun " + timeout);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("DriveLeft init");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double power = SmartDashboard.getNumber("Drive Power", 0.2d);
    System.out.println("DriveLeft drive at " + power);
    m_driveTrain.driveLeft(power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !isTimedOut();
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
    m_driveTrain.stopAll();
  }
}
