// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.MathR;
import frc.robot.utils.VectorR;

public class JoystickTurnSpeedDriveCommand extends Command {

  final DriveSubsystem drive;

  // CONTROLLER DATA
  XboxController control;
  VectorR leftJoystick = new VectorR();
  double rightJoystick = 0.0;

  //INFORMATION
  double maxSpeed = 0.25;
  boolean isLocked = false;
  double lockedHeading;
  final double TURN_KP = 0.017;

  public JoystickTurnSpeedDriveCommand(DriveSubsystem drive, XboxController control) {
    this.drive = drive;
    this.control = control;
    
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    DriveSubsystem.resetGyro(0.0);
  }

  @Override
  public void execute() {
    maxSpeed = MathR.lerp(0.25, 1.2, 0.0, 1.0, control.getLeftTriggerAxis());

    leftJoystick.setFromCartesian(-control.getLeftY(), -control.getLeftX());
    leftJoystick.rotate(Math.toRadians(-90));
    leftJoystick.pow(2);
    rightJoystick = -1 * Math.pow(control.getRightX(), 2) * Math.signum(control.getRightX());

    if (leftJoystick.getMagnitude() < 0.1 && Math.abs(rightJoystick) < 0.2) {
      drive.stop();
      isLocked = false;
      return;
    }

    if (leftJoystick.getMagnitude() > 0.1 && Math.abs(rightJoystick) < 0.2) {
      if (!isLocked) {
        lockedHeading = DriveSubsystem.getYawDegrees();
        isLocked = true;
      }
    } 
    else if (leftJoystick.getMagnitude() < 0.1 && Math.abs(rightJoystick) > 0.2) {
      leftJoystick.setFromCartesian(0.0, 0.0);
      isLocked = false;
    }
    else isLocked = false;

    double angleToFace = isLocked ? lockedHeading : 1000;
    double turnPower;


    if (angleToFace != 1000){
      turnPower = MathR.limit(TURN_KP * MathR.getDistanceToAngle(DriveSubsystem.getYawDegrees(), angleToFace), -1, 1);
    }
    else{
      turnPower = MathR.lerp(-1, 1, -1, 1.0, rightJoystick);
    }

    leftJoystick.mult(maxSpeed);
    drive.move(leftJoystick, turnPower * maxSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}