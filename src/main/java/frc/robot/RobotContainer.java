// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.DriverControls;
import frc.robot.commands.OperatorControls;
import frc.robot.subsystems.RobotSubsystem;
import frc.robot.motor.Motors;
import frc.robot.subsystems.LimelightSubsystem;


public class RobotContainer {
  DriveSubsystem drive = new DriveSubsystem();
  RobotSubsystem robot = new RobotSubsystem();
  LimelightSubsystem limelight = new LimelightSubsystem();
  
  XboxController driverController = new XboxController(Constants.DRIVE_CONTROL_PORT);
  XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROL_PORT);
  GenericHID buttonBoard = new GenericHID(Constants.BUTTON_BOARD_PORT);
  
  DriverControls driver;
  OperatorControls operator;
  Motors encodeTest;

  public RobotContainer() {
    drive.setDefaultCommand(new DriverControls(drive, driverController));
    robot.setDefaultCommand(new OperatorControls(robot, operatorController, buttonBoard));
    configureBindings();
  }

  
  private void configureBindings() {
    SmartDashboard.putData(new InstantCommand(()->{DriveSubsystem.resetGyro(0);}));
  }

 
  public Command getAutonomousCommand() {
    return null;
  }

  public void displayDebug() {
    drive.modules.debugSmartDashboard();
    robot.pivot.debugSmartDashboard();
    limelight.LimelightWhere();
  }
}