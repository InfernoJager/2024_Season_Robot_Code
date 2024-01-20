// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.JoystickOrientedDriveCommand;
import frc.robot.commands.JoystickTurnSpeedDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.commands.Controls;


public class RobotContainer {
  DriveSubsystem drive = new DriveSubsystem();
  
  XboxController control = new XboxController(Constants.DRIVE_CONTROL_PORT);
  
  //Controls controls = new Controls();

  public RobotContainer() {
    drive.setDefaultCommand(new JoystickTurnSpeedDriveCommand(drive, control));
    configureBindings();
    //controls.DriverControls();
    //controls.OperatorControls();
  }

  
  private void configureBindings() {
    SmartDashboard.putData(new InstantCommand(()->{DriveSubsystem.resetGyro(0);}));
  }

 
  public Command getAutonomousCommand() {
    return null;
  }

  public void displayDebug() {
    drive.modules.debugSmartDashboard();
  }
}